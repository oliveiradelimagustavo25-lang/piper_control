"""共享测试夹具：生命周期在这里，具体检查在 stages/ 中。"""

import threading
import time
import unittest

import rclpy
import yaml
from my_robot_interface.action import SortObjects
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from test.support.planner_environment import Environment, object_msg
from test.support.runtime import ROOT, Processes


class PlannerFixture(unittest.TestCase):
    needs_planner = True
    output = None  # 由入口在创建输出目录后设置。

    @classmethod
    def setUpClass(cls):
        # addClassCleanup 在初始化中途失败时也会执行，避免留下 ROS 线程。
        rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        cls.addClassCleanup(rclpy.shutdown)
        cls.environment = Environment()
        cls.addClassCleanup(cls.environment.destroy_node)
        cls.node = rclpy.create_node('planner_check_client')
        cls.addClassCleanup(cls.node.destroy_node)
        cls.client = ActionClient(cls.node, SortObjects, '/sorting/sort_objects')
        cls.addClassCleanup(cls.client.destroy)
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.environment)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()
        cls.addClassCleanup(cls.stop_executor)
        cls.processes = Processes(cls.output)
        cls.addClassCleanup(cls.processes.close)
        if cls.needs_planner:
            cls.start_planner()

    @classmethod
    def stop_executor(cls):
        cls.environment.release.set()
        cls.executor.shutdown()
        cls.thread.join()
        cls.environment.server.destroy()
        cls.environment.named_server.destroy()

    @classmethod
    def start_planner(cls):
        config = yaml.safe_load((ROOT / 'src/my_robot_planner/config/task_planner.yaml').read_text())
        config['task_planner']['ros__parameters'].update(
            observation_timeout=0.8, max_observation_age=0.3, observation_settle_time=0.03,
            default_color_order=[1, 2, 3],
            **{'red.place_position': [0.60, 0.0, 0.005]})
        # 0.60 m 仅为模拟执行端的距离排序测试点，不发送给机械臂。
        config_path = cls.output / 'planner.yaml'
        config_path.write_text(yaml.safe_dump(config))
        cls.start(['ros2', 'launch', 'my_robot_planner', 'task_planner.launch.py',
                   'ros_domain_id:=94', 'config:=' + str(config_path),
                   'use_sim_time:=false'], 'planner')
        if not cls.client.wait_for_server(timeout_sec=20):
            raise RuntimeError('任务规划器未启动，请查看 planner.log')

    @classmethod
    def start(cls, command, name):
        return cls.processes.start(command, name)

    def spin_until(self, condition, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.025)
            if condition():
                return
        self.fail('Condition timed out')

    def wait(self, future, timeout=5):
        self.spin_until(future.done, timeout)
        return future.result()

    def settle(self, duration=0.15):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.025)

    def setUp(self):
        self.environment.reset()
        self.handle, self.result_future = None, None
        self.feedback = []
        self.settle()

    def tearDown(self):
        if self.handle and self.handle.accepted and self.result_future and not self.result_future.done():
            self.environment.release.set()
            self.wait(self.handle.cancel_goal_async())
            self.wait(self.result_future)

    def send(self, strategy='color_order', colors=None):
        goal = SortObjects.Goal(strategy=strategy, color_order=[1, 2, 3] if colors is None else colors)
        self.handle = self.wait(self.client.send_goal_async(goal,
            feedback_callback=lambda msg: self.feedback.append(msg.feedback)))
        self.assertTrue(self.handle.accepted)
        self.result_future = self.handle.get_result_async()
        return self.result_future

    def colors_called(self):
        return [next(c for c in [1, 2, 3] if object_msg(c, 0).object_id == call.object_id)
                for call in self.environment.calls]

