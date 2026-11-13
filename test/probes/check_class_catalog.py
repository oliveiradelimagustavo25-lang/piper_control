#!/usr/bin/env python3
"""验证 CV → 类别图 → depth 的正常数据流，以及类别编号与颜色/尺寸解耦。"""

import argparse
import copy
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import cv2
import rclpy
from my_robot_interface.msg import DetectedObjectArray
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'src/my_robot_perception/config'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-domain-id', type=int, default=196)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    graph = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True,
                           text=True, check=True, timeout=10)
    if graph.stdout.strip():
        raise RuntimeError('Test domain occupied')
    rclpy.init()
    node = Node('class_catalog_probe')
    rgb = node.create_publisher(Image, '/catalog/rgb', 10)
    depth = node.create_publisher(Image, '/catalog/depth', 10)
    info = node.create_publisher(CameraInfo, '/catalog/info', 10)
    labels, objects = [], []
    node.create_subscription(Image, '/catalog/labels', labels.append, 10)
    node.create_subscription(DetectedObjectArray, '/catalog/objects', objects.append, 10)
    try:
        with tempfile.TemporaryDirectory(prefix='piper_catalog_') as directory:
            directory = Path(directory)
            source = yaml.safe_load((CONFIG / 'object_classes.yaml').read_text())
            alternate = copy.deepcopy(source)
            for entry, class_id, height in zip(alternate['classes'], [11, 27, 42], [0.01, 0.02, 0.03]):
                entry['id'] = class_id
                entry['geometry']['dimensions'][2] = height
            for name, catalog in [('default', source), ('alternate', alternate)]:
                (directory / (name + '.yaml')).write_text(yaml.safe_dump(catalog))
            config = yaml.safe_load((CONFIG / 'perception.yaml').read_text())
            config['/**']['ros__parameters']['class_config'] = 'default.yaml'
            config['color_detector']['ros__parameters'].update(
                image_topic='/catalog/rgb', labels_topic='/catalog/labels',
                publish_debug=False, show_window=False)
            config['depth_localizer']['ros__parameters'].update(
                labels_topic='/catalog/labels', depth_topic='/catalog/depth',
                camera_info_topic='/catalog/info', objects_topic='/catalog/objects')
            config_file = directory / 'perception.yaml'
            config_file.write_text(yaml.safe_dump(config))
            # 检查总配置的相对路径和显式 CLI 覆盖，不启动机器人。
            spec = importlib.util.spec_from_file_location('catalog_bringup',
                ROOT / 'src/my_robot_bringup/launch/mujoco.launch.py')
            bringup = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(bringup)
            from launch import LaunchContext
            launch_file = directory / 'launch.yaml'
            launch_file.write_text('class_config: default.yaml\n')
            context = LaunchContext()
            context.launch_configurations['launch_config'] = str(launch_file)
            description = bringup.generate_launch_description()
            argument = next(a for a in description.entities if getattr(a, 'name', None) == 'class_config')
            argument.execute(context)
            assert context.launch_configurations['class_config'] == str(directory / 'default.yaml')
            context.launch_configurations['class_config'] = str(directory / 'alternate.yaml')
            argument.execute(context)
            assert context.launch_configurations['class_config'] == str(directory / 'alternate.yaml')

            for name, catalog in [('default', source), ('alternate', alternate)]:
                command = ['ros2', 'launch', 'my_robot_bringup', 'perception.launch.py',
                           'perception_config:=' + str(config_file), 'use_sim_time:=false']
                if name == 'alternate':
                    command.append('class_config:=' + os.path.relpath(directory / 'alternate.yaml'))
                with (directory / 'launch.log').open('w+') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    labels.clear()
                    objects.clear()
                    try:
                        for executable in ('color_detector', 'depth_localizer'):
                            client = node.create_client(GetParameters, '/' + executable + '/get_parameters')
                            assert client.wait_for_service(timeout_sec=10), executable
                            future = client.call_async(GetParameters.Request(names=['class_config']))
                            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                            assert future.result().values[0].string_value == str(directory / (name + '.yaml'))
                            node.destroy_client(client)
                        image = np.zeros((120, 160, 3), dtype=np.uint8)
                        for u, bgr in [(40, (0, 0, 255)), (65, (0, 0, 255)),
                                       (95, (0, 255, 0)), (125, (255, 0, 0))]:
                            image[56:65, u-4:u+5] = bgr
                        depth_image = np.full((120, 160), 0.5, dtype=np.float32)
                        if name == 'default':
                            # 同一 5 cm 绿盒的较低侧面被颜色分割成独立连通区域，
                            # 不能额外生成一个物体；两个同高度红盒仍须保留。
                            image[58:63, 83:88] = (0, 255, 0)
                            depth_image[58:63, 83:88] = 0.48
                        until = time.monotonic() + 12
                        sent = set()
                        label_frame = None
                        while time.monotonic() < until:
                            stamp = node.get_clock().now().to_msg()
                            sent.add((stamp.sec, stamp.nanosec))
                            camera = CameraInfo()
                            camera.header.frame_id = 'base_link'
                            camera.header.stamp = stamp
                            camera.width, camera.height = 160, 120
                            camera.k = [200., 0., 80., 0., 200., 60., 0., 0., 1.]
                            camera.distortion_model = 'plumb_bob'
                            info.publish(camera)
                            color = Image(header=camera.header, width=160, height=120,
                                          encoding='bgr8', step=480, data=image.tobytes())
                            distances = Image(header=camera.header, width=160, height=120,
                                              encoding='32FC1', step=640,
                                              data=depth_image.tobytes())
                            rgb.publish(color)
                            depth.publish(distances)
                            rclpy.spin_once(node, timeout_sec=0.04)
                            if process.poll() is not None:
                                log.seek(0)
                                raise AssertionError(log.read())
                            if objects and len(objects[-1].objects) == 4:
                                candidate = objects[-1]
                                current_stamp = (candidate.header.stamp.sec, candidate.header.stamp.nanosec)
                                if current_stamp in sent and {o.class_id for o in candidate.objects} == {c['id'] for c in catalog['classes']}:
                                    label_frame = next((m for m in reversed(labels)
                                                        if m.header.stamp == candidate.header.stamp), None)
                                    if label_frame is not None:
                                        break
                        assert label_frame is not None, 'No matching labels/objects from the current run'
                        message = objects[-1]
                        assert not message.issues, message.issues
                        assert message.header.frame_id == 'base_link'
                        assert (message.header.stamp.sec, message.header.stamp.nanosec) in sent
                        pixels = np.frombuffer(label_frame.data, dtype=np.uint8)
                        assert set(np.unique(pixels)) == {0, *(c['id'] for c in catalog['classes'])}
                        label_image = pixels.reshape(120, 160)
                        red_id, green_id = catalog['classes'][0]['id'], catalog['classes'][1]['id']
                        assert cv2.connectedComponents((label_image == green_id).astype(np.uint8))[0] - 1 == 1
                        assert cv2.connectedComponents((label_image == red_id).astype(np.uint8))[0] - 1 == 2
                        definitions = {c['id']: c for c in catalog['classes']}
                        assert len({o.object_id for o in message.objects}) == 4
                        assert sorted(o.color for o in message.objects) == [1, 1, 2, 3]
                        for obj in message.objects:
                            definition = definitions[obj.class_id]
                            dimensions = definition['geometry']['dimensions']
                            assert obj.object_id.startswith(definition['name'] + '_')
                            assert [obj.dimensions.x, obj.dimensions.y, obj.dimensions.z] == dimensions
                            assert math.isclose(obj.pose.position.z, 0.5 - dimensions[2] / 2, abs_tol=1e-6)
                        print(f'PASS {name}: two red + green + blue; class IDs, colors, dimensions, center and timestamp', flush=True)
                    finally:
                        if process.poll() is None:
                            os.killpg(process.pid, signal.SIGINT)
                            try:
                                process.wait(8)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
