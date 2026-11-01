"""三色分拣流程：就绪 → 生成物块 → 发任务/采样 → 物理验收。"""

import logging
import math
import time

import cv2
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from my_robot_interface.action import MoveToNamed, PickPlace, SortObjects
from my_robot_interface.msg import DetectedObjectArray
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from test.support.ros import RosWait, active_controllers


NAMES = ['red_cube', 'green_cube', 'blue_cube']
TARGETS = dict(zip(NAMES, [[0.26, -0.06, 0.005], [0.26, 0.0, 0.005], [0.26, 0.06, 0.005]]))


class SortingCheck:
    """可在临时脚本中复用各步骤；调用者负责 node 和仿真的生命周期。"""

    def __init__(self, node, output, process):
        self.node = node
        self.output = output
        self.wait = RosWait(node, process)
        self.logger = logging.getLogger('sorting')
        self.client = ActionClient(node, SortObjects, '/sorting/sort_objects')
        self.pick_client = ActionClient(node, PickPlace, '/manipulation/pick_place')
        self.named_client = ActionClient(node, MoveToNamed, '/manipulation/move_to_named')
        self.controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.spawn = node.create_client(SpawnEntity, '/spawn_entity')
        self.states = node.create_client(GetEntityState, '/sorting_test/get_entity_state')
        self.bridge = CvBridge()
        self.vision = {'frames': 0, 'max_objects': 0}
        self.subscriptions = [
            node.create_subscription(Image, '/perception/color_debug', self.debug_frame,
                                     qos_profile_sensor_data),
            node.create_subscription(DetectedObjectArray, '/perception/objects', self.objects, 10),
        ]

    def close(self):
        for client in [self.client, self.pick_client, self.named_client]:
            client.destroy()

    def debug_frame(self, message):
        self.vision['frames'] += 1
        if self.vision['frames'] % 10 == 0:
            cv2.imwrite(str(self.output / 'vision_latest.png'),
                        self.bridge.imgmsg_to_cv2(message, 'bgr8'))

    def objects(self, message):
        self.vision['max_objects'] = max(self.vision['max_objects'], len(message.objects))
        self.vision['last_objects'] = {
            obj.object_id: [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z]
            for obj in message.objects
        }

    def ready(self):
        self.wait.require(self.client.server_is_ready, 60, '分拣 Action 未就绪')
        active_controllers(self.wait, self.controllers)
        # 控制器激活并不意味着 MoveIt 执行器已完成初始化。
        self.wait.require(self.pick_client.server_is_ready, 60, '抓放执行器未就绪')
        self.wait.require(self.named_client.server_is_ready, 60, '观察位接口未就绪')
        self.wait.require(self.spawn.service_is_ready, 10, 'Gazebo 生成服务未就绪')
        self.wait.require(self.states.service_is_ready, 10, 'Gazebo 状态查询未就绪')

    def spawn_objects(self, processes, domain):
        process = processes.start([
            'ros2', 'launch', 'my_robot_sim', 'spawn_sorting.launch.py',
            f'ros_domain_id:={domain}', 'x:=0.30', 'z:=0.006',
            'red_y:=-0.06', 'green_y:=0.0', 'blue_y:=0.06'], 'spawn')
        self.wait.require(lambda: process.poll() is not None, 60, '物块生成超时')
        if process.returncode != 0:
            raise RuntimeError('物块生成失败，请查看 spawn.log')
        # 初始高度应在物块落稳后采样，避免把落地距离计入抬升。
        self.wait.settle(0.5)

    def xyz(self, name):
        request = GetEntityState.Request(name=name, reference_frame='world')
        response = self.wait.result(self.states.call_async(request))
        if not response.success:
            raise RuntimeError('物块状态不可用：' + name)
        position = response.state.pose.position
        return [position.x, position.y, position.z]

    def feedback(self, message):
        update = message.feedback
        self.logger.info('当前物体=%s，已完成=%s',
                         update.current_object_id or '观察中', update.completed_count)

    def execute(self, strategy, order, report):
        # 只跟踪本次请求的颜色，避免把其他物块纳入验收。
        names = [NAMES[color - 1] for color in order]
        initial = {name: self.xyz(name) for name in names}
        max_z = {name: point[2] for name, point in initial.items()}
        goal = SortObjects.Goal(strategy=strategy, color_order=order)
        handle = self.wait.result(self.client.send_goal_async(goal, feedback_callback=self.feedback))
        if not handle.accepted:
            raise RuntimeError('分拣任务被拒绝')
        future = handle.get_result_async()
        deadline = time.monotonic() + 240
        try:
            while not future.done() and time.monotonic() < deadline:
                self.wait.settle(0.05)
                for name in names:
                    max_z[name] = max(max_z[name], self.xyz(name)[2])
            if not future.done():
                raise TimeoutError('分拣任务超过 240 s')
        except BaseException:
            # 包括 Ctrl+C；先请求取消，再交给调用者关闭仿真。
            self.wait.result(handle.cancel_goal_async())
            self.wait.result(future, 30)
            raise
        result = future.result()
        report.update(status=result.status, success=result.result.success, message=result.result.message)
        self.wait.settle(0.5)
        report['objects'] = self.verify(names, initial, max_z)
        report['passed'] = (result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
                            and all(item['passed'] for item in report['objects'].values()))

    def verify(self, names, initial, max_z):
        objects = {}
        for name in names:
            final = self.xyz(name)
            lift = max_z[name] - initial[name][2]
            error = math.dist(final, TARGETS[name])
            objects[name] = {'final_xyz': final, 'lift_m': lift, 'error_m': error,
                             'passed': lift > 0.04 and error < 0.02}
        return objects
