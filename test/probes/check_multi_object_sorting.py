#!/usr/bin/env python3
"""通过唯一 .sh 入口验证任意总配置；颜色数量选项只用于生成本次测试的临时输入。"""
import argparse
from collections import Counter
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from my_robot_interface.action import SortObjects
from my_robot_interface.msg import DetectedObjectArray
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
import yaml

ROOT = Path(__file__).resolve().parents[2]
COLORS = ['red', 'green', 'blue']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/mujoco_launch.yaml')
    for color in COLORS:
        parser.add_argument('--' + color, type=int, help='测试数量；不填沿用总配置，0 表示不生成')
    parser.add_argument('--ros-domain-id', type=int, default=187)
    parser.add_argument('--timeout', type=float, default=420)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    if os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp' and not os.environ.get('CYCLONEDDS_URI'):
        os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><Discovery>'
            '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>63</MaxAutoParticipantIndex>'
            '</Discovery></Domain></CycloneDDS>')
    # 唯一入口自身也检查空闲域；探针节点必须等这个检查完成后再进入该域。
    directory = ROOT / 'log/multi_object_sorting' / str(time.time_ns())
    directory.mkdir(parents=True)
    config = yaml.safe_load(args.config.read_text()) or {}
    for name in ('camera_config', 'controllers_file', 'moveit_controllers_file', 'manipulation_config',
                 'planner_config', 'perception_config', 'scene_file', 'robot_model'):
        if config.get(name):
            config[name] = str((args.config.resolve().parent / config[name]).resolve())
    counts = {c: getattr(args, c) for c in COLORS if getattr(args, c) is not None}
    if counts:
        config.pop('scene_file', None)
        scene = dict(config.get('scenes') or {})
        total = sum(counts.values())
        # 通用测试排布：在当前已校准视野的中部均匀排布，而非按颜色组合写特例。
        columns = max(1, math.ceil(total / 5))
        rows = max(1, math.ceil(total / columns))
        i = 0
        for color, count in counts.items():
            if count < 0:
                parser.error('Object counts must be non-negative')
            scene[color] = []
            for _ in range(count):
                x = 0.32 + (0.04 * (i // rows) / (columns - 1) if columns > 1 else 0)
                y = -0.12 + 0.24 * (i % rows) / (rows - 1) if rows > 1 else 0.0
                scene[color].append([x, y, 0.03])
                i += 1
        config['scenes'] = scene
    launch_config = directory / 'mujoco_launch.yaml'
    launch_config.write_text(yaml.safe_dump(config))
    helper = importlib.util.spec_from_file_location('scene_config', ROOT / 'script/build/scene_config.py')
    module = importlib.util.module_from_spec(helper)
    helper.loader.exec_module(module)
    # 只用于推导验收数量；实际启动由 .sh -> launch 自动生成另一份场景，无 XML 参数。
    module.generate(config.get('scenes'), directory / 'expected', ROOT / 'src/my_robot_world')
    initial = ET.parse(directory / 'expected/objects.xml').getroot().findall('body')
    expected_colors = Counter(next(i+1 for i,c in enumerate(COLORS) if b.get('name').startswith(c)) for b in initial)
    process, node = None, None
    log_path = directory / 'entry.log'
    try:
        with log_path.open('w') as output:
            process = subprocess.Popen([str(ROOT / 'script/pick_three_colors.sh'), '--config', str(launch_config),
                '--ros-domain-id', str(args.ros_domain_id), '--headless', '--no-debug-window'],
                cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 30
            while '等待机械臂控制器' not in log_path.read_text():
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('Entry startup failed: ' + log_path.read_text()[-2000:])
                time.sleep(0.1)
            rclpy.init()
            node = Node('multi_object_sorting_probe')
            latest, samples = [], []
            completed = [0]
            def observe(msg):
                latest[:] = msg.objects
                samples.append([(o.object_id, o.color, o.pose.position.x, o.pose.position.y, o.pose.position.z)
                                for o in msg.objects])
            def feedback(msg):
                if msg.feedback.completed_count != completed[0]:
                    completed[0] = msg.feedback.completed_count
                    print(f'Verified {completed[0]} objects', flush=True)
            node.create_subscription(DetectedObjectArray, '/perception/objects', observe, 10)
            node.create_subscription(SortObjects.Impl.FeedbackMessage, '/sorting/sort_objects/_action/feedback', feedback, 10)
            def wait(condition, seconds):
                until = time.monotonic() + seconds
                while not condition():
                    if time.monotonic() >= until or process.poll() is not None:
                        raise RuntimeError('Entry test did not complete: ' + log_path.read_text()[-2500:])
                    rclpy.spin_once(node, timeout_sec=0.1)
            params = node.create_client(GetParameters, '/task_planner/get_parameters')
            wait(params.service_is_ready, 60)
            names = ([c + '.place_slots' for c in COLORS] + ['slot_tolerance', 'place_position_modes'] +
                     ['place_positions.' + c for c in COLORS])
            request = params.call_async(GetParameters.Request(names=names))
            wait(request.done, 10)
            values = request.result().values
            slots = {i+1: list(zip(*[iter(v.double_array_value)]*3)) for i,v in enumerate(values[:3])}
            tolerance = values[3].double_value
            modes = values[4].string_array_value
            rule_values = [v.double_array_value for v in values[5:]]
            if any(rule_values):
                slots = {i+1: [] for i in range(3)}
                expected_moves = 0
                for body in initial:
                    color = next(i for i,c in enumerate(COLORS) if body.get('name').startswith(c))
                    source = list(map(float, body.get('pos').split()))
                    # 本探针生成的是默认地面上的已知方块；初始高度经过落地，不是派发中心。
                    source[2] = float(body.find('geom').get('size').split()[2])
                    target = tuple(v + source[i] if modes[i] == 'relative' else v
                                   for i,v in enumerate(rule_values[color]))
                    slots[color+1].append(target)
                    expected_moves += math.dist(source, target) > tolerance
            else:
                expected_moves = sum(not any(math.hypot(float(b.get('pos').split()[0])-p[0],
                                    float(b.get('pos').split()[1])-p[1]) <= tolerance
                                    for p in slots[next(i+1 for i,c in enumerate(COLORS) if b.get('name').startswith(c))])
                                    for b in initial)
            wait(lambda: '仿真保持运行' in log_path.read_text(), args.timeout)
            (directory / 'observations.yaml').write_text(yaml.safe_dump(samples))
            assert completed[0] == expected_moves, (completed, expected_moves)
            assert Counter(o.color for o in latest) == expected_colors, [(o.object_id, o.color) for o in latest]
            for o in latest:
                p = o.pose.position
                assert any(math.dist((p.x,p.y,p.z), target) <= tolerance for target in slots[o.color]), o
            print(f'PASS one-click MuJoCo: colors={dict(expected_colors)}, verified={completed[0]}; log: {log_path}', flush=True)
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
