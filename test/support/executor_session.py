"""执行器测试客户端：ROS 连接与采样放这里，阶段步骤放 stages/。"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from my_robot_interface.action import PickPlace, MoveToNamed
from my_robot_interface.msg import DetectedObjectArray
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from test.support.ros import RosWait, active_controllers


class ExecutorSession:
    def __init__(self, args, output, report, process):
        self.args = args
        self.output = output
        self.report = report
        self.process = process
        self.joint_positions = {}
        self.contact_trace = []
        self.active_stage = None

    def __enter__(self):
        rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        self.node = rclpy.create_node('executor_validation_driver')
        self.waiter = RosWait(self.node, self.process)
        self.client = ActionClient(self.node, PickPlace, '/manipulation/pick_place')
        self.named_client = ActionClient(self.node, MoveToNamed, '/manipulation/move_to_named')
        self.get_scene = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_scene = self.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.release_held = self.node.create_client(Trigger, '/manipulation/release_held_object')
        self.objects_pub = self.node.create_publisher(DetectedObjectArray, '/validation/objects', 10)
        self.state_client = self.node.create_client(GetEntityState, '/validation/get_entity_state')
        self.spawn_client = self.node.create_client(SpawnEntity, '/spawn_entity')
        self.controllers = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.subscriptions = [
            self.node.create_subscription(JointState, '/joint_states', self.on_joints, 10),
            self.node.create_subscription(ContactsState, '/validation/cube_contacts', self.on_contacts, 10),
        ]
        return self

    def __exit__(self, *_):
        self.client.destroy()
        self.named_client.destroy()
        self.node.destroy_node()
        rclpy.try_shutdown()

    def wait(self, future, timeout=10):
        return self.waiter.result(future, timeout)

    def until(self, condition, timeout):
        return self.waiter.until(condition, timeout)

    def ready(self):
        self.waiter.require(self.client.server_is_ready, 60, '抓放 Action 未就绪')
        self.waiter.require(self.named_client.server_is_ready, 30, '命名运动 Action 未就绪')
        self.report['controllers'] = active_controllers(self.waiter, self.controllers)
        self.waiter.require(lambda: len(self.joint_positions) >= 8, 10, '关节状态未就绪')
        self.report['scene'] = {}
        print('执行器与控制器就绪', flush=True)

    def on_joints(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    def on_contacts(self, msg):
        for contact in msg.states:
            self.contact_trace.append({'time_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'stage': self.active_stage,
                'collisions': [contact.collision1_name, contact.collision2_name],
                'depth_m': max(contact.depths, default=0.0),
                'force_n': [contact.total_wrench.force.x, contact.total_wrench.force.y,
                            contact.total_wrench.force.z]})

    def state(self, name, reference='world'):
        request = GetEntityState.Request()
        request.name, request.reference_frame = name, reference
        response = self.wait(self.state_client.call_async(request))
        if not response.success:
            raise RuntimeError('Entity state unavailable: ' + name)
        position = response.state.pose.position
        return [position.x, position.y, position.z]

    def goal(self, name, pick, put):
        target = PickPlace.Goal()
        target.object_id = name
        target.object_dimensions.x = target.object_dimensions.y = target.object_dimensions.z = self.args.cube_size
        for pose, xyz in [(target.object_pose, pick), (target.place_pose, put)]:
            pose.header.frame_id = 'base_link'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            pose.pose.orientation.w = 1.0
        return target

    def planning_scene(self):
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        return self.wait(self.get_scene.call_async(request)).scene

    def scene_objects(self, objects):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = objects
        assert self.wait(self.apply_scene.call_async(request)).success

    def remove_objects(self, *names):
        self.scene_objects([CollisionObject(id=name, operation=CollisionObject.REMOVE) for name in names])

    def run_action(self, name, target, track=False):
        """执行一次抓放；可选采样物块真值与规划场景，不改变业务节点。"""
        stages = []

        def feedback(message):
            self.active_stage = message.feedback.stage
            stages.append(self.active_stage)

        self.active_stage = None
        self.contact_trace.clear()
        handle = self.wait(self.client.send_goal_async(target, feedback_callback=feedback))
        if not handle.accepted:
            self.report['actions'][name] = {'accepted': False, 'passed': False}
            return False
        future = handle.get_result_async()
        trace, snapshots = [], {}
        deadline = time.monotonic() + 150
        next_sample = 0
        try:
            while not future.done() and time.monotonic() < deadline:
                self.waiter.settle(0.05)
                if track and time.monotonic() >= next_sample:
                    self.sample_object(target.object_id, stages, trace, snapshots)
                    next_sample = time.monotonic() + 0.15
            if not future.done():
                raise TimeoutError(name + ' 超过执行时限')
        except BaseException:
            self.wait(handle.cancel_goal_async())
            self.wait(future, 10)
            raise

        result = future.result()
        unique = list(dict.fromkeys(stages))
        passed = (result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
                  and unique == [0, 1, 2, 3])
        self.report['actions'][name] = {
            'accepted': True, 'status': result.status, 'success': result.result.success,
            'message': result.result.message, 'stages': unique, 'passed': passed,
        }
        if track:
            self.verify_physical(target, trace, snapshots)
        print(name + ': ' + json.dumps(self.report['actions'][name]), flush=True)
        return passed

    def sample_object(self, name, stages, trace, snapshots):
        trace.append(self.state(name))
        scene = self.planning_scene()
        if any(obj.object.id == name for obj in scene.robot_state.attached_collision_objects):
            self.report['scene']['attached_during_transport'] = True
        if stages and str(stages[-1]) not in snapshots:
            snapshots[str(stages[-1])] = {
                'gripper_joints': {joint: self.joint_positions.get(joint) for joint in ['joint7', 'joint8']},
                'cube_in_link6': self.state(name, 'piper::link6'),
                'cube_in_link7': self.state(name, 'piper::link7'),
                'cube_in_link8': self.state(name, 'piper::link8'),
            }

    def verify_physical(self, target, trace, snapshots):
        # 释放后等物块落稳，避免用尚在下落的位置计算最终误差。
        for _ in range(10):
            self.waiter.settle(0.1)
            trace.append(self.state(target.object_id))
        final = trace[-1]
        desired = target.place_pose.pose.position
        error = math.dist(final, [desired.x, desired.y, desired.z])
        lift = max(point[2] for point in trace) - target.object_pose.pose.position.z
        contacts = {
            link: sum(any(('::' + link + '::') in collision for collision in sample['collisions'])
                      for sample in self.contact_trace)
            for link in ['link7', 'link8']
        }
        self.report['physical'] = {
            'final_xyz': final, 'placement_error_m': error, 'max_lift_m': lift,
            'lift_verified': lift > 0.04, 'placement_verified': error < 0.02,
            'snapshots': snapshots, 'finger_contact_samples': contacts,
            'passed': lift > 0.04 and error < 0.02,
        }
        (self.output / 'object_trace.json').write_text(json.dumps(trace))
        (self.output / 'contact_trace.json').write_text(json.dumps(self.contact_trace))
        scene = self.planning_scene()
        self.report['scene']['released_into_world'] = (
            any(obj.id == target.object_id for obj in scene.world.collision_objects)
            and not any(obj.object.id == target.object_id
                        for obj in scene.robot_state.attached_collision_objects))
        print(f"物理抓放：抬升 {lift * 1000:.2f} mm，放置误差 {error * 1000:.2f} mm", flush=True)
