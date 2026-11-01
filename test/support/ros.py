"""ROS 等待与控制器就绪检查，供测试和临时客户端复用。"""

import time

import rclpy
from controller_manager_msgs.srv import ListControllers


class RosWait:
    def __init__(self, node, process=None):
        self.node = node
        self.process = process

    def until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise KeyboardInterrupt
            if self.process is not None and self.process.poll() is not None:
                raise RuntimeError('仿真启动进程已退出，请查看 bringup.log')
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return True
        return False

    def require(self, condition, timeout, message):
        if not self.until(condition, timeout):
            raise TimeoutError(message)

    def result(self, future, timeout=10):
        self.require(future.done, timeout, 'ROS 请求超时')
        return future.result()

    def settle(self, duration=0.3):
        self.until(lambda: False, duration)


def active_controllers(wait, client, timeout=30):
    """控制器管理器可用后，继续等待三个控制器实际激活。"""
    wait.require(client.service_is_ready, timeout, '控制器管理器未就绪')
    states = {}

    def ready():
        response = wait.result(client.call_async(ListControllers.Request()))
        states.update((controller.name, controller.state) for controller in response.controller)
        return all(states.get(name) == 'active' for name in
                   ['arm_controller', 'gripper_controller', 'joint_state_broadcaster'])

    wait.require(ready, timeout, '控制器没有全部激活')
    return states
