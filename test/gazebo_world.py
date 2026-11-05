#!/usr/bin/env python3
"""Preview the Gazebo world and three cubes without starting the robot."""
import argparse
import os
from pathlib import Path
import signal
import subprocess

from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-gui', action='store_true', help='run without the Gazebo window')
    parser.add_argument('--world', help='Gazebo world file')
    args = parser.parse_args()
    world = Path(get_package_share_directory('my_robot_world'))
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID', '227')
    env['ROS_LOCALHOST_ONLY'] = env.get('ROS_LOCALHOST_ONLY', '1')
    env['GAZEBO_MODEL_PATH'] = os.pathsep.join(filter(None, [
        str(world / 'objects'), '/usr/share/gazebo-11/models', env.get('GAZEBO_MODEL_PATH', '')]))
    env['GAZEBO_MODEL_DATABASE_URI'] = ''
    world_file = args.world or str(world / 'basic/preview.world')
    server = subprocess.Popen(['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
                               f'world:={world_file}', f'gui:={str(not args.no_gui).lower()}'], env=env)
    try:
        # spawn_entity waits for Gazebo's factory service; no second physics process is started.
        result = subprocess.run(['ros2', 'launch', 'my_robot_world', 'spawn_cube.launch.py',
                                 f"ros_domain_id:={env['ROS_DOMAIN_ID']}"], env=env)
        if result.returncode:
            return result.returncode
        return server.wait()
    except KeyboardInterrupt:
        return 0
    finally:
        if server.poll() is None:
            server.send_signal(signal.SIGINT)
            server.wait(timeout=30)


if __name__ == '__main__':
    raise SystemExit(main())
