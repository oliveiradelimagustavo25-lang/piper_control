"""任务测试使用的模拟执行端与可控视觉观测，不驱动机械臂。"""

import copy
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from my_robot_interface.action import MoveToNamed, PickPlace
from my_robot_interface.msg import DetectedObject, DetectedObjectArray
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def object_msg(color, x, y=0.0, z=0.005):
    obj = DetectedObject()
    obj.object_id = ['', 'red_cube', 'green_cube', 'blue_cube'][color]
    obj.color = color
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = float(x), float(y), float(z)
    obj.pose.orientation.w = 1.0
    obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = 0.01
    return obj


class Environment(Node):
    def __init__(self):
        super().__init__('planner_test_environment')
        self.publisher = self.create_publisher(DetectedObjectArray, '/perception/objects', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        camera = TransformStamped()
        camera.header.frame_id, camera.child_frame_id = 'base_link', 'check_camera'
        camera.transform.translation.x, camera.transform.translation.z = 0.26, 1.01
        camera.transform.rotation.x = 1.0  # 光轴朝下，物体上表面在 z=0.01。
        camera.transform.rotation.w = 0.0
        offset = TransformStamped()
        offset.header.frame_id, offset.child_frame_id = 'base_link', 'check_offset'
        offset.transform.translation.x = 0.1
        offset.transform.translation.y = 0.2
        offset.transform.translation.z = 0.3
        offset.transform.rotation.w = 1.0
        self.static_tf.sendTransform([camera, offset])
        self.reset()
        self.timer = self.create_timer(0.025, self.publish_scene)
        self.server = ActionServer(self, PickPlace, '/manipulation/pick_place',
            execute_callback=self.execute, goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.named_server = ActionServer(self, MoveToNamed, '/manipulation/move_to_named',
            execute_callback=self.observe, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def reset(self):
        self.scene = [object_msg(1, 0.21), object_msg(2, 0.40), object_msg(3, 0.30)]
        self.frame = 'base_link'
        self.stamp_offset = 0.0
        self.publish_enabled = True
        self.tcp = [0.20, 0.0, 0.005]
        self.calls = []
        self.fail_ids, self.reject_ids = set(), set()
        self.hold = False
        self.observation_calls = []
        self.observation_fail = False
        self.observation_hold = False
        self.release = threading.Event()
        self.cancel_seen = threading.Event()

    def observe(self, handle):
        self.observation_calls.append(handle.request.target)
        while self.observation_hold and not self.release.is_set():
            if handle.is_cancel_requested:
                handle.canceled()
                return MoveToNamed.Result(success=False, message='Mock observation canceled')
            time.sleep(0.01)
        if self.observation_fail:
            handle.abort()
            return MoveToNamed.Result(success=False, message='Mock observation failure')
        handle.succeed()
        return MoveToNamed.Result(success=True, message='Mock observation reached')

    def publish_scene(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'link6'
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = self.tcp[0]
        transform.transform.translation.y = self.tcp[1]
        transform.transform.translation.z = self.tcp[2] - 0.132
        self.tf.sendTransform(transform)
        if self.publish_enabled:
            msg = DetectedObjectArray()
            msg.header.frame_id = self.frame
            msg.header.stamp = (self.get_clock().now() +
                rclpy.duration.Duration(seconds=self.stamp_offset)).to_msg()
            msg.objects = self.scene
            self.publisher.publish(msg)

    def accept(self, goal):
        return GoalResponse.REJECT if goal.object_id in self.reject_ids else GoalResponse.ACCEPT

    def execute(self, handle):
        holding = self.hold
        self.calls.append(copy.deepcopy(handle.request))
        started = time.monotonic()
        while (holding and not self.release.is_set()) or time.monotonic() - started < 0.10:
            if handle.is_cancel_requested:
                self.cancel_seen.set()
                handle.canceled()
                return PickPlace.Result(success=False, message='Mock canceled')
            time.sleep(0.01)
        if handle.request.object_id in self.fail_ids:
            handle.abort()
            return PickPlace.Result(success=False, message='Mock execution failure')
        p = handle.request.place_pose.pose.position
        self.tcp = [p.x, p.y, p.z]
        handle.succeed()
        return PickPlace.Result(success=True, message='Mock motion completed')


