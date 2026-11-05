#!/usr/bin/env python3
"""Preview the robot description in RViz, with joint sliders."""
import os

if __name__ == '__main__':
    os.execvp('ros2', ['ros2', 'launch', 'my_robot_description', 'robot_piper_model.launch.xml'])
