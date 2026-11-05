#!/usr/bin/env python3
"""Run the robot and ros2_control in Gazebo without spawning sorting objects."""
import os
import sys

if __name__ == '__main__':
    os.execvp('ros2', ['ros2', 'launch', 'my_robot_bringup', 'ros2_control.launch.py',
                       'spawn_objects:=false', *sys.argv[1:]])
