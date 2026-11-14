#!/usr/bin/env python3
"""Verify independent ONNX execution and CV/YOLO -> the real shared depth node.

The tiny input-dependent ONNX graphs are protocol fixtures, not trained YOLO weights.
Requires onnx for fixture generation, plus the built external SDK and ROS adapter.
"""

import argparse
from collections import Counter
import copy
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'src/my_robot_perception/config'
NAMES = ['green_cube', 'red_cube', 'yellow_cube', 'blue_cube']
BOXES = [(3, 1), (21, 1), (39, 0), (57, 3), (84, 2)]  # Last mask touches the image edge.


def fixture(path, raw=False):
    # Original image 96x48 -> 64x32 + 16 pixels of top/bottom padding.
    rows = []
    for x, index in BOXES:
        rows.append([x * 2 / 3, 28, (x + 12) * 2 / 3, 36, .95, index, 10])
    if raw:
        rows.append([2, 28, 10, 36, .6, 1, 10])  # Same-class duplicate must be suppressed.
        converted = []
        for x1, y1, x2, y2, score, index, coefficient in rows:
            scores = [0.] * len(NAMES)
            scores[int(index)] = score
            converted.append([(x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1,
                              *scores, coefficient])
        predictions = np.array(converted, dtype=np.float32).T[None]
    else:
        predictions = np.array(rows, dtype=np.float32)[None]
    initializers = [numpy_helper.from_array(value, name) for name, value in [
        ('base', predictions), ('proto', np.ones((1, 1, 16, 16), np.float32)),
        ('channel', np.array(0, np.int64)), ('threshold', np.array(.9, np.float32))]]
    nodes = [
        helper.make_node('Gather', ['images', 'channel'], ['red'], axis=1),
        helper.make_node('ReduceMax', ['red'], ['maximum'], keepdims=0),
        helper.make_node('Greater', ['maximum', 'threshold'], ['visible']),
        helper.make_node('Cast', ['visible'], ['gate'], to=TensorProto.FLOAT),
        helper.make_node('Mul', ['base', 'gate'], ['predictions']),
        helper.make_node('Identity', ['proto'], ['prototypes']),
    ]
    graph = helper.make_graph(nodes, 'segmentation_protocol_fixture',
        [helper.make_tensor_value_info('images', TensorProto.FLOAT, [1, 3, 64, 64])],
        [helper.make_tensor_value_info('predictions', TensorProto.FLOAT, predictions.shape),
         helper.make_tensor_value_info('prototypes', TensorProto.FLOAT, [1, 1, 16, 16])],
        initializer=initializers)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 18)], ir_version=10)
    helper.set_model_props(model, {'task': 'segment', 'names': str(dict(enumerate(NAMES)))})
    onnx.checker.check_model(model)
    onnx.save(model, path)


def load_launch(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'src/my_robot_bringup/launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_priority(directory, config_file, model):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    launch = load_launch('yolo_launch_priority', 'mujoco.launch.py')
    file = directory / 'launch.yaml'
    file.write_text(yaml.safe_dump({'detector': 'yolo', 'yolo_model': model.name,
                                    'perception_config': config_file.name}))
    for explicit in ({}, {'detector': 'cv', 'yolo_model': '/explicit/model.onnx'}):
        context = LaunchContext()
        context.launch_configurations.update(launch_config=str(file), **explicit)
        for action in launch.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        assert context.launch_configurations['detector'] == explicit.get('detector', 'yolo')
        assert context.launch_configurations['yolo_model'] == explicit.get('yolo_model', str(model))
    control = load_launch('yolo_camera_priority', 'ros2_control.launch.py')
    config = yaml.safe_load(config_file.read_text())
    overrides = {'image_topic': '', 'depth_topic': '', 'camera_info_topic': ''}
    config['color_detector']['ros__parameters']['image_topic'] = '/cv/input'
    assert launch.camera_topics(config, 'cv', overrides.__getitem__)['image_topic'] == '/cv/input'
    values = {'camera_config': str(ROOT / 'src/my_robot_world/config/camera.yaml'),
              **launch.camera_topics(config, 'yolo', overrides.__getitem__)}
    runtime = tempfile.TemporaryDirectory(dir=directory)
    try:
        plugin = yaml.safe_load(Path(control.camera_plugin_file(runtime, values.__getitem__)).read_text())
        camera = plugin['/**']['ros__parameters']['mujoco_plugins']['camera_publisher']
        assert any(isinstance(v, dict) and v.get('image_topic') == '/yolo/input' for v in camera.values())
        overrides['image_topic'] = '/explicit/input'
        values.update(launch.camera_topics(config, 'yolo', overrides.__getitem__))
        plugin = yaml.safe_load(Path(control.camera_plugin_file(runtime, values.__getitem__)).read_text())
        camera = plugin['/**']['ros__parameters']['mujoco_plugins']['camera_publisher']
        assert any(isinstance(v, dict) and v.get('image_topic') == '/explicit/input' for v in camera.values())
    finally:
        runtime.cleanup()
    print('PASS launch: CLI > main YAML, model path base, selected camera topic', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-domain-id', type=int, default=197)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from my_robot_interface.msg import DetectedObjectArray
    from rcl_interfaces.srv import GetParameters

    graph = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True,
                           text=True, check=True, timeout=10)
    if graph.stdout.strip():
        raise RuntimeError('Test domain occupied')
    rclpy.init()
    node = Node('yolo_adapter_probe')
    rgb = node.create_publisher(Image, '/yolo/input', 10)
    depth = node.create_publisher(Image, '/yolo/depth', 10)
    info = node.create_publisher(CameraInfo, '/yolo/info', 10)
    labels, objects = [], []
    node.create_subscription(Image, '/yolo/labels', labels.append, 10)
    node.create_subscription(DetectedObjectArray, '/yolo/objects', objects.append, 10)
    try:
        with tempfile.TemporaryDirectory(prefix='piper_yolo_') as temp:
            directory = Path(temp)
            image = np.zeros((48, 96, 3), np.uint8)
            palette = [(0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 0, 0)]
            for x, index in BOXES:
                image[18:30, x:x+12] = palette[index]
            cv2.imwrite(str(directory / 'input.png'), image)
            cv2.imwrite(str(directory / 'empty.png'), np.zeros_like(image))
            cv2.imwrite(str(directory / 'blue.png'), np.full_like(image, (255, 0, 0)))
            models = {layout: directory / (layout + '.onnx') for layout in ('e2e', 'raw')}
            tool = ROOT / 'tools/yolo/.deps/install/bin/yolo_predict'
            expected_labels = np.zeros((48, 96), np.uint16)
            for x, index in BOXES:
                expected_labels[18:30, x:x+12] = index + 1
            for layout, model in models.items():
                fixture(model, raw=layout == 'raw')
                output = directory / 'labels.png'
                for name in ('input', 'empty', 'blue'):
                    result = subprocess.run([str(tool), str(model), str(directory / (name + '.png')),
                                             str(output)], capture_output=True, text=True, timeout=10)
                    assert result.returncode == 0, result.stderr
                    actual = cv2.imread(str(output), cv2.IMREAD_UNCHANGED)
                    assert np.array_equal(actual, expected_labels if name == 'input' else np.zeros_like(actual))
                    assert len(result.stdout.splitlines()) == (5 if name == 'input' else 0), result.stdout
                print(f'PASS standalone {layout}: preprocessing, letterbox, masks, class order, empty, NMS', flush=True)

            catalog = yaml.safe_load((CONFIG / 'object_classes.yaml').read_text())
            definitions = copy.deepcopy(catalog)
            for entry, class_id, height in zip(definitions['classes'], [11, 27, 42, 99], [.01, .02, .03, .04]):
                entry['id'] = class_id
                entry['geometry']['dimensions'][2] = height
            catalog_file = directory / 'classes.yaml'
            catalog_file.write_text(yaml.safe_dump(definitions))
            config = yaml.safe_load((CONFIG / 'perception.yaml').read_text())
            config['/**']['ros__parameters'].update(class_config='missing.yaml', detector='yolo')
            for executable in ('color_detector', 'yolo_detector'):
                config[executable]['ros__parameters'].update(image_topic='/yolo/input', labels_topic='/yolo/labels',
                    publish_debug=False, show_window=False)
            config['yolo_detector']['ros__parameters']['model_path'] = 'raw.onnx'
            config['depth_localizer']['ros__parameters'].update(labels_topic='/yolo/labels', depth_topic='/yolo/depth',
                camera_info_topic='/yolo/info', objects_topic='/yolo/objects')
            config_file = directory / 'perception.yaml'
            config_file.write_text(yaml.safe_dump(config))
            launch_priority(directory, config_file, models['e2e'])
            expected_counts = Counter({11: 2, 27: 1, 42: 1, 99: 1})
            for detector, model in [('cv', None), ('yolo', models['e2e']), ('yolo', None)]:
                labels.clear(); objects.clear()
                command = ['ros2', 'launch', 'my_robot_bringup', 'perception.launch.py',
                           'perception_config:=' + str(config_file), 'detector:=' + detector,
                           'class_config:=' + os.path.relpath(catalog_file), 'use_sim_time:=false']
                if model:
                    command.append('yolo_model:=' + os.path.relpath(model))
                with (directory / 'launch.log').open('w+') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        executable = 'color_detector' if detector == 'cv' else 'yolo_detector'
                        for name in (executable, 'depth_localizer'):
                            client = node.create_client(GetParameters, '/' + name + '/get_parameters')
                            assert client.wait_for_service(timeout_sec=10), name
                            names = ['class_config'] + (['model_path'] if name == 'yolo_detector' else [])
                            future = client.call_async(GetParameters.Request(names=names))
                            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                            assert future.result().values[0].string_value == str(catalog_file)
                            if name == 'yolo_detector':
                                assert future.result().values[1].string_value == str(model or models['raw'])
                            node.destroy_client(client)
                        assert {n for n, _ in node.get_node_names_and_namespaces()} & {
                            'color_detector', 'yolo_detector'} == {executable}
                        for empty in (False, True):
                            sent = set()
                            matched = None
                            until = time.monotonic() + 8
                            while time.monotonic() < until:
                                stamp = node.get_clock().now().to_msg()
                                sent.add((stamp.sec, stamp.nanosec))
                                camera = CameraInfo()
                                camera.header.frame_id = 'base_link'
                                camera.header.stamp = stamp
                                camera.width, camera.height = 96, 48
                                camera.k = [200., 0., 48., 0., 200., 24., 0., 0., 1.]
                                info.publish(camera)
                                rgb.publish(Image(header=camera.header, width=96, height=48, encoding='bgr8',
                                    step=288, data=(np.zeros_like(image) if empty else image).tobytes()))
                                depth.publish(Image(header=camera.header, width=96, height=48, encoding='32FC1',
                                    step=384, data=np.full((48, 96), .5, np.float32).tobytes()))
                                rclpy.spin_once(node, timeout_sec=.04)
                                if process.poll() is not None:
                                    raise AssertionError('Perception launch exited')
                                for message in reversed(objects):
                                    key = (message.header.stamp.sec, message.header.stamp.nanosec)
                                    if key in sent and Counter(o.class_id for o in message.objects) == (
                                        Counter() if empty else expected_counts):
                                        matched = message
                                        break
                                if matched:
                                    break
                            assert matched is not None, f'No expected objects ({detector}, empty={empty})'
                            assert not matched.issues, matched.issues
                            assert matched.header.frame_id == 'base_link'
                            definitions_by_id = {entry['id']: entry for entry in definitions['classes']}
                            for obj in matched.objects:
                                definition = definitions_by_id[obj.class_id]
                                assert obj.object_id.startswith(definition['name'] + '_')
                                assert math.isclose(obj.pose.position.z,
                                    .5 - definition['geometry']['dimensions'][2] / 2, abs_tol=1e-6)
                            frame = next((m for m in reversed(labels) if m.header.stamp == matched.header.stamp), None)
                            deadline = time.monotonic() + 2
                            while frame is None and time.monotonic() < deadline:
                                rclpy.spin_once(node, timeout_sec=.05)
                                frame = next((m for m in reversed(labels) if m.header.stamp == matched.header.stamp), None)
                            assert frame is not None, 'Matching label frame was not received'
                            assert frame.encoding == 'mono8' and frame.width == 96 and frame.height == 48
                            assert set(np.frombuffer(frame.data, np.uint8)) == ({0} if empty else {0, 11, 27, 42, 99})
                        print(f'PASS ROS {detector}/{model.name if model else "YAML"}: '
                              'same depth node, five instances, catalog override, original timestamps, empty', flush=True)
                    except Exception:
                        log.seek(0)
                        print(log.read(), flush=True)
                        raise
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
