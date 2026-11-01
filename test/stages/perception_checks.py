"""使用合成 RGB 标签/深度检查表面点到物体中心的转换。"""

import time

import numpy as np
import yaml
from ament_index_python.packages import get_package_prefix
from my_robot_interface.msg import DetectedObjectArray
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from test.support.planner_environment import object_msg
from test.support.planner_fixture import PlannerFixture


class PerceptionChecks(PlannerFixture):
    needs_planner = False

    def test_depth_surface_to_center(self):
        self.environment.publish_enabled = False
        params = {'depth_localizer': {'ros__parameters': {
            'labels_topic': '/check/labels', 'depth_topic': '/check/depth',
            'camera_info_topic': '/check/info', 'objects_topic': '/check/objects',
            'points_topic': '/check/points'}}}
        (self.output / 'depth.yaml').write_text(yaml.safe_dump(params))
        process = self.start([get_package_prefix('my_robot_perception') +
            '/lib/my_robot_perception/depth_localizer', '--ros-args',
            '--params-file', str(self.output / 'depth.yaml'), '--log-level', 'depth_localizer:=debug'], 'depth')
        objects, points = [], []
        sub = self.node.create_subscription(DetectedObjectArray, '/check/objects', objects.append, 10)
        cloud_sub = self.node.create_subscription(PointCloud2, '/check/points', points.append, 10)
        pubs = [self.node.create_publisher(kind, topic, 10) for kind, topic in
                [(Image, '/check/labels'), (Image, '/check/depth'), (CameraInfo, '/check/info')]]
        try:
            self.settle(0.6)
            for encoding in ['32FC1', '16UC1', 'empty']:
                labels = np.zeros((64, 64), dtype=np.uint8)
                if encoding != 'empty':
                    for color, column in [(1, 20), (2, 30), (3, 40)]:
                        labels[30:34, column:column + 4] = color
                    labels[10:13, 10:13] = 1  # 同色小区域不覆盖最大的红色物块。
                depth = np.ones((64, 64), dtype=np.float32) if encoding != '16UC1' else np.full(
                    (64, 64), 1000, dtype=np.uint16)
                label_msg = Image(height=64, width=64, encoding='mono8', step=64,
                                  data=labels.tobytes())
                label_msg.header.frame_id = 'check_camera'
                label_msg.header.stamp = self.node.get_clock().now().to_msg()
                depth_msg = Image(height=64, width=64,
                    encoding='16UC1' if encoding == '16UC1' else '32FC1',
                    step=64 * depth.dtype.itemsize, data=depth.tobytes())
                depth_msg.header = label_msg.header
                info = CameraInfo(height=64, width=64, distortion_model='plumb_bob',
                                  k=[100.0, 0.0, 31.5, 0.0, 100.0, 31.5, 0.0, 0.0, 1.0])
                info.header = label_msg.header
                objects.clear()
                points.clear()
                deadline = time.monotonic() + 5
                expected = 0 if encoding == 'empty' else 3
                while (not objects or objects[-1].header.stamp != label_msg.header.stamp or
                       len(objects[-1].objects) != expected) and time.monotonic() < deadline:
                    pubs[2].publish(info)
                    pubs[0].publish(label_msg)
                    pubs[1].publish(depth_msg)
                    self.settle(0.10)
                self.assertTrue(objects)
                result = objects[-1]
                self.assertEqual(result.header.frame_id, 'base_link')
                self.assertEqual(result.header.stamp, label_msg.header.stamp)
                self.assertEqual(len(result.objects), expected,
                                 f'{encoding}: cloud widths={[p.width for p in points[-3:]]}')
                if encoding != 'empty':
                    for i, obj in enumerate(result.objects):
                        self.assertEqual(obj.object_id, object_msg(i + 1, 0).object_id)
                        self.assertAlmostEqual(obj.pose.position.x, 0.16 + i * 0.10, places=5)
                        self.assertAlmostEqual(obj.pose.position.y, 0.0, places=5)
                        self.assertAlmostEqual(obj.pose.position.z, 0.005, places=5)
                        self.assertAlmostEqual(obj.dimensions.z, 0.01)
                    self.assertTrue(points)
                    self.assertEqual(points[-1].header.frame_id, 'check_camera')
                    self.assertEqual(points[-1].width, 4)
        finally:
            self.processes.stop(process)
            self.node.destroy_subscription(sub)
            self.node.destroy_subscription(cloud_sub)
            for publisher in pubs:
                self.node.destroy_publisher(publisher)

