#!/usr/bin/env python3
"""独立启动机器人控制仿真，检查状态、时间和小幅关节轨迹。"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM = [f'joint{i}' for i in range(1, 7)]
GRIPPER = ['joint7', 'joint8']


class ControlCheck(Node):
    def __init__(self):
        super().__init__('piper_control_check')
        self.positions = {}
        self.clock = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')

    def on_joints(self, message):
        self.positions = dict(zip(message.name, message.position))

    def on_clock(self, message):
        self.clock = message.clock.sec + message.clock.nanosec / 1e9


def wait_for(node, condition, process, timeout, message):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        if process.poll() is not None:
            raise RuntimeError(f'仿真启动进程提前退出：{process.returncode}')
        rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(message)


def check_controllers(node, process):
    wait_for(node, node.controllers.service_is_ready, process, 30, '控制器管理器未就绪')
    expected = ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')
    deadline = time.monotonic() + 20
    states = {}
    while time.monotonic() < deadline:
        future = node.controllers.call_async(ListControllers.Request())
        wait_for(node, future.done, process, 5, '无法查询控制器状态')
        response = future.result()
        if response is None:
            raise RuntimeError('控制器状态查询失败')
        states = {controller.name: controller.state for controller in response.controller}
        if all(states.get(name) == 'active' for name in expected):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        raise RuntimeError(f'控制器尚未激活：{states}')
    print('控制器已激活：' + ', '.join(expected), flush=True)


def send_trajectory(node, process, controller, names, targets, tolerance):
    client = ActionClient(node, FollowJointTrajectory, f'/{controller}/follow_joint_trajectory')
    wait_for(node, client.server_is_ready, process, 15, f'{controller} action 未就绪')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = targets
    point.time_from_start = Duration(sec=3)
    goal.trajectory.points = [point]
    goal.goal_time_tolerance = Duration(sec=3)
    sent = client.send_goal_async(goal)
    wait_for(node, sent.done, process, 10, f'{controller} 未接受轨迹')
    handle = sent.result()
    if handle is None or not handle.accepted:
        raise RuntimeError(f'{controller} 拒绝轨迹')
    result = handle.get_result_async()
    wait_for(node, result.done, process, 15, f'{controller} 轨迹未结束')
    if result.result() is None or result.result().status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f'{controller} 轨迹执行失败：{result.result()}')
    wait_for(node, lambda: all(name in node.positions and
             abs(node.positions[name] - target) < tolerance
             for name, target in zip(names, targets)), process, 4,
             f'{controller} 反馈未达到目标：{node.positions}')
    print(f'{controller} 轨迹与反馈通过：' +
          ', '.join(f'{name}={node.positions[name]:.4f}' for name in names), flush=True)
    client.destroy()


def stop_launch(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', action='store_true', help='显示 MuJoCo 窗口；默认无窗口')
    parser.add_argument('--ros-domain-id', type=int, default=184, help='专用于本次测试的 ROS 域')
    args = parser.parse_args()
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    process = subprocess.Popen(
        ['ros2', 'launch', 'my_robot_bringup', 'ros2_control.launch.py',
         f'headless:={str(not args.viewer).lower()}'], env=env, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = ControlCheck()
        wait_for(node, lambda: len(node.positions) >= 8 and node.clock is not None,
                 process, 40, '未收到 8 个关节或仿真时钟')
        wait_for(node, lambda: node.controllers.service_is_ready, process, 20, '控制器服务未就绪')
        # Spawners may be active slightly after the first joint state arrives.
        clients = [ActionClient(node, FollowJointTrajectory, f'/{name}/follow_joint_trajectory')
                   for name in ('arm_controller', 'gripper_controller')]
        try:
            wait_for(node, lambda: all(client.server_is_ready() for client in clients),
                     process, 30, '轨迹控制器未就绪')
        finally:
            for client in clients:
                client.destroy()
        check_controllers(node, process)
        if not all(name in node.positions and math.isfinite(node.positions[name]) for name in ARM + GRIPPER):
            raise RuntimeError(f'关节状态不完整或无效：{node.positions}')
        first_clock = node.clock
        wait_for(node, lambda: node.clock is not None and node.clock > first_clock + 0.1,
                 process, 5, '仿真时钟未推进')
        targets = [node.positions[name] for name in ARM]
        targets[0] += 0.15
        # joint2 and joint3 start at their mechanical limits.
        targets[1] = max(0.0, targets[1])
        targets[2] = min(0.0, targets[2])
        send_trajectory(node, process, 'arm_controller', ARM, targets, 0.08)
        send_trajectory(node, process, 'gripper_controller', GRIPPER, [0.01, -0.01], 0.004)
        return 0
    except (OSError, RuntimeError) as error:
        print(f'控制测试失败：{error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_launch(process)


if __name__ == '__main__':
    raise SystemExit(main())
