#!/usr/bin/env python3
"""启动独立 MuJoCo，验证 MoveIt 场景同步和障碍碰撞。"""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from my_robot_interface.action import MoveToNamed, PickPlace
from my_robot_interface.msg import DetectedObject, DetectedObjectArray
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[2]


class SceneCheck(Node):
    def __init__(self, process, objects_topic):
        super().__init__('mujoco_scene_check')
        self.process = process
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.named = ActionClient(self, MoveToNamed, '/manipulation/move_to_named')
        self.pick = ActionClient(self, PickPlace, '/manipulation/pick_place')
        self.release = self.create_client(Trigger, '/manipulation/release_held_object')
        self.objects = self.create_publisher(DetectedObjectArray, objects_topic, 10)

    def wait(self, condition, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            if self.process.poll() is not None:
                raise RuntimeError('仿真提前退出，请查看启动日志')
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f'等待超时：{description}')

    def call(self, client, request):
        future = client.call_async(request)
        self.wait(future.done, 8, '规划场景服务')
        if future.result() is None:
            raise RuntimeError('规划场景服务失败')
        return future.result()

    def scene(self):
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                          PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        return self.call(self.get_scene, request).scene

    def publish_object(self, object_id, x, y, z):
        message = DetectedObjectArray()
        message.header.frame_id = 'base_link'
        # Zero timestamp denotes a static test observation, independent of simulation time.
        obj = DetectedObject()
        obj.object_id, obj.color = object_id, 1
        obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = x, y, z
        obj.pose.orientation.w = 1.0
        obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = 0.01
        message.objects = [obj]
        # The planner and executor subscribe independently; both must see the test observation.
        self.wait(lambda: self.objects.get_subscription_count() >= 2, 15, '视觉物块订阅者')
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.objects.publish(message)
            rclpy.spin_once(self, timeout_sec=0.1)
        return obj

    def action(self, client, goal, timeout=60):
        self.wait(client.server_is_ready, 30, '动作接口')
        future = client.send_goal_async(goal)
        self.wait(future.done, 10, '动作接收')
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('动作目标被拒绝')
        result = handle.get_result_async()
        self.wait(result.done, timeout, '动作完成')
        outcome = result.result()
        if outcome is None or outcome.status != GoalStatus.STATUS_SUCCEEDED or not outcome.result.success:
            raise RuntimeError(f'动作失败：{outcome}')

    def check_world_and_snapshot(self):
        self.wait(self.get_scene.service_is_ready, 30, 'MoveIt 场景服务')
        self.wait(self.named.server_is_ready, 40, '命名姿态接口')
        scene_objects = self.scene().world.collision_objects
        ground = next((obj for obj in scene_objects if obj.id == 'environment/ground'), None)
        if (ground is None or list(ground.primitives[0].dimensions) != [4.0, 4.0, 0.02]
                or abs(ground.pose.position.z + 0.01) > 1e-6):
            raise AssertionError('地面碰撞盒不存在或尺寸不正确')
        board = next((obj for obj in scene_objects if obj.id == 'environment/aruco_board'), None)
        if (board is None or list(board.primitives[0].dimensions) != [0.2, 0.01, 0.2]
                or abs(board.pose.position.x - 0.5) > 1e-6
                or abs(board.pose.position.z - 0.1) > 1e-6
                or abs(abs(board.pose.orientation.z) - math.sqrt(0.5)) > 1e-6):
            raise AssertionError('标记板碰撞盒不存在或位姿、尺寸不正确')

        self.publish_object('scene_probe_cube', 0.30, 0.20, 0.05)
        self.action(self.named, MoveToNamed.Goal(target='pose_1'))
        objects = self.scene().world.collision_objects
        cube = next((obj for obj in objects if obj.id == 'scene_probe_cube'), None)
        if cube is None:
            raise AssertionError(f'视觉物块未同步到 MoveIt 场景：{[obj.id for obj in objects]}')
        if abs(cube.pose.position.y - 0.20) > 1e-6:
            raise AssertionError(f'视觉物块坐标不一致：{cube.pose.position}')
        self.publish_object('scene_probe_cube', 0.30, 0.24, 0.05)
        self.action(self.named, MoveToNamed.Goal(target='pose_1'))
        cube = next(obj for obj in self.scene().world.collision_objects if obj.id == 'scene_probe_cube')
        if abs(cube.pose.position.y - 0.24) > 1e-6:
            raise AssertionError('下一次运动前没有更新视觉物块位置')
        print('通过：地面、标记板和视觉物块碰撞盒', flush=True)

    def check_collision(self):
        self.wait(self.apply_scene.service_is_ready, 15, '场景写入服务')
        self.wait(self.validity.service_is_ready, 15, '状态碰撞检测服务')
        request = GetStateValidity.Request()
        request.group_name = 'arm'
        request.robot_state.is_diff = True
        if not self.call(self.validity, request).valid:
            raise AssertionError('加入障碍前机器人状态已碰撞')

        blocker = CollisionObject()
        blocker.id, blocker.header.frame_id, blocker.operation = 'scene_probe_blocker', 'base_link', CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type, shape.dimensions = SolidPrimitive.BOX, [2.0, 2.0, 2.0]
        pose = Pose()
        pose.position.z, pose.orientation.w = 0.5, 1.0
        blocker.primitives, blocker.primitive_poses = [shape], [pose]
        update = ApplyPlanningScene.Request()
        update.scene.is_diff = update.scene.robot_state.is_diff = True
        update.scene.world.collision_objects = [blocker]
        if not self.call(self.apply_scene, update).success:
            raise RuntimeError('无法添加测试障碍')
        try:
            result = self.call(self.validity, request)
            if result.valid or not any('scene_probe_blocker' in (c.contact_body_1, c.contact_body_2)
                                       for c in result.contacts):
                raise AssertionError('MoveIt 未检测到测试障碍碰撞')
        finally:
            blocker.operation = CollisionObject.REMOVE
            self.call(self.apply_scene, update)
        if not self.call(self.validity, request).valid:
            raise AssertionError('移除障碍后机器人状态仍然碰撞')
        print('通过：障碍加入后碰撞、移除后恢复', flush=True)

    def check_attachment(self, cancel=False):
        obj = self.publish_object('red_cube', 0.32, -0.08, 0.005)
        goal = PickPlace.Goal()
        goal.object_id = obj.object_id
        goal.object_pose.header.frame_id = 'base_link'
        goal.object_pose.pose = obj.pose
        goal.object_dimensions = obj.dimensions
        goal.place_pose.header.frame_id = 'base_link'
        goal.place_pose.pose.position.x, goal.place_pose.pose.position.y = 0.26, -0.06
        goal.place_pose.pose.position.z, goal.place_pose.pose.orientation.w = 0.005, 1.0

        self.wait(self.pick.server_is_ready, 30, '抓放接口')
        sent = self.pick.send_goal_async(goal)
        self.wait(sent.done, 10, '抓放目标接收')
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('抓放目标被拒绝')
        result = handle.get_result_async()
        attached_seen = False
        cancel_sent = False
        deadline = time.monotonic() + 90
        while not result.done() and time.monotonic() < deadline:
            scene = self.scene()
            attached_seen |= any(item.object.id == obj.object_id and
                                 {'link7', 'link8'}.issubset(item.touch_links)
                                 for item in scene.robot_state.attached_collision_objects)
            if cancel and attached_seen and not cancel_sent:
                request = handle.cancel_goal_async()
                self.wait(request.done, 10, '取消抓放')
                if not request.result().goals_canceling:
                    raise RuntimeError('取消请求未被接受')
                cancel_sent = True
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result.done():
            raise RuntimeError('抓放动作超时')
        outcome = result.result()
        expected = GoalStatus.STATUS_CANCELED if cancel else GoalStatus.STATUS_SUCCEEDED
        if outcome is None or outcome.status != expected or (not cancel and not outcome.result.success):
            raise RuntimeError(f'抓放动作失败：{outcome}')
        scene = self.scene()
        if cancel:
            if not attached_seen or not any(item.object.id == obj.object_id
                                            for item in scene.robot_state.attached_collision_objects):
                raise AssertionError('取消后没有保留规划场景中的附着物')
            blocked = self.pick.send_goal_async(goal)
            self.wait(blocked.done, 10, '带载时的抓放请求')
            if blocked.result() is None or blocked.result().accepted:
                raise AssertionError('保留负载时不应接收新的抓放请求')
            self.wait(self.release.service_is_ready, 10, '负载恢复服务')
            response = self.call(self.release, Trigger.Request())
            if not response.success:
                raise RuntimeError(f'负载恢复失败：{response.message}')
            scene = self.scene()
        released = next((item for item in scene.world.collision_objects if item.id == obj.object_id), None)
        still_attached = any(item.object.id == obj.object_id
                             for item in scene.robot_state.attached_collision_objects)
        if not attached_seen or released is None or still_attached:
            raise AssertionError('抓放期间没有观察到附着，或释放后场景状态不正确')
        position = released.pose.position
        if not cancel and (abs(position.x - 0.26) > 0.03 or abs(position.y + 0.06) > 0.03):
            raise AssertionError(f'释放后的碰撞盒位置不在目标附近：{position}')
        print('通过：取消后保留并显式释放附着物' if cancel else
              '通过：抓放期间附着，释放后返回世界场景', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-domain-id', type=int, default=185, help='本次独立测试使用的 ROS 域')
    parser.add_argument('--objects-topic', default='/perception/objects', help='测试使用的视觉物体话题')
    checks = parser.add_mutually_exclusive_group()
    checks.add_argument('--grasp', action='store_true', help='额外执行单物块抓放，检查附着与释放')
    checks.add_argument('--recovery', action='store_true', help='在附着后取消并检查显式释放')
    args = parser.parse_args()
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    # 不能让测试客户端误连到已经运行的控制器或真实机器人。
    try:
        nodes = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], env=env,
                               capture_output=True, text=True, timeout=10, check=True)
    except (OSError, subprocess.SubprocessError) as error:
        print(f'无法检查测试 ROS 域：{error}', file=sys.stderr)
        return 1
    if nodes.stdout.strip():
        print(f'ROS 域 {args.ros_domain_id} 已有节点，请指定空闲域：\n{nodes.stdout}', file=sys.stderr)
        return 1
    log_dir = ROOT / 'log' / 'planning_scene'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{time.time_ns()}.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'my_robot_bringup', 'mujoco.launch.py',
             f'launch_config:={ROOT / "config/mujoco_launch.yaml"}', 'headless:=true',
             'with_camera:=false', 'with_perception:=false', f'objects_topic:={args.objects_topic}'],
            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            rclpy.init()
            node = SceneCheck(process, args.objects_topic)
            node.check_world_and_snapshot()
            node.check_collision()
            if args.grasp or args.recovery:
                node.check_attachment(cancel=args.recovery)
            print(f'规划场景测试通过；启动日志：{log_path}', flush=True)
            return 0
        except (AssertionError, OSError, RuntimeError) as error:
            print(f'规划场景测试失败：{error}；启动日志：{log_path}', file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            return 130
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)


if __name__ == '__main__':
    raise SystemExit(main())
