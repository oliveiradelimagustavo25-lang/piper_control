#!/usr/bin/env python3
"""合成标签/深度验证多实例 ID、轻微移动、漏检重现、歧义与 TF 故障。"""
import argparse
import copy
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from my_robot_interface.msg import DetectedObjectArray

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-domain-id', type=int, default=188)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    nodes = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True, text=True,
                           check=True, timeout=10)
    if nodes.stdout.strip():
        raise RuntimeError('Test domain occupied')
    rclpy.init()
    node = Node('multi_perception_probe')
    labels = node.create_publisher(Image, '/probe/labels', 10)
    depths = node.create_publisher(Image, '/probe/depth', 10)
    infos = node.create_publisher(CameraInfo, '/probe/info', 10)
    received = []
    node.create_subscription(DetectedObjectArray, '/probe/objects', lambda msg: received.append(msg), 10)
    directory = ROOT / 'log/multi_perception'
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'localizer.log').open('w') as log:
        process = subprocess.Popen([str(ROOT/'install/my_robot_perception/lib/my_robot_perception/depth_localizer'),
            '--ros-args', '-p', 'labels_topic:=/probe/labels', '-p', 'depth_topic:=/probe/depth',
            '-p', 'camera_info_topic:=/probe/info', '-p', 'objects_topic:=/probe/objects',
            '-p', 'objects_frame:=base_link'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def feed(regions, frame='base_link', seconds=0.5):
            data = np.zeros((120,160), dtype=np.uint8)
            for u,color in regions:
                data[58:63, u-2:u+3] = color
            start = len(received)
            deadline = time.monotonic()+seconds
            while time.monotonic()<deadline:
                stamp = node.get_clock().now().to_msg()
                info = CameraInfo()
                info.header.frame_id=frame; info.header.stamp=stamp
                info.width=160; info.height=120; info.k=[200.,0.,80.,0.,200.,60.,0.,0.,1.]
                info.distortion_model='plumb_bob'
                infos.publish(info)
                image = Image()
                image.header=info.header; image.width=160; image.height=120
                image.encoding='mono8'; image.step=160; image.data=data.tobytes()
                depth=copy.deepcopy(image)
                depth.encoding='32FC1'; depth.step=160*4
                depth.data=np.full((120,160),0.5,dtype=np.float32).tobytes()
                labels.publish(image); depths.publish(depth)
                rclpy.spin_once(node, timeout_sec=0.03)
                if process.poll() is not None:
                    raise AssertionError('Localizer exited')
            return received[start:]
        try:
            feed([(60,1),(100,1)], seconds=1.5)
            first = sorted(received[-1].objects,key=lambda o:o.pose.position.x)
            assert len(first)==2 and first[0].object_id!=first[1].object_id
            feed([(63,1),(100,1)])
            moved=sorted(received[-1].objects,key=lambda o:o.pose.position.x)
            assert [o.object_id for o in first]==[o.object_id for o in moved]
            feed([])
            assert not received[-1].objects
            feed([(63,1),(100,1)])
            returned=sorted(received[-1].objects,key=lambda o:o.pose.position.x)
            assert [o.object_id for o in moved]==[o.object_id for o in returned]
            feed([(73,2),(83,2)])
            assert len(received[-1].objects)==2
            feed([(78,2)])
            assert received[-1].objects[0].tracking_uncertain
            feed([(63,1),(100,1)], frame='missing_optical_frame', seconds=0.2)
            # 排空故障切换前在途的回调；随后故障帧不应产生空 objects。
            before=len(received)
            new=feed([(63,1),(100,1)], frame='missing_optical_frame', seconds=0.4)
            assert not new, [len(m.objects) for m in new]
            print('PASS perception: two red IDs, movement, reappearance, ambiguity, TF fault', flush=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL); process.wait()
            node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
