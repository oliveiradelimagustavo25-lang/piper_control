"""Attach the configured wrist camera to the robot's optical frame."""

import math
from pathlib import Path

import yaml


def add_camera(spec, config_path):
    camera = yaml.safe_load(Path(config_path).read_text())
    body = spec.body(camera['frame'])
    if body is None:
        raise ValueError(f"Camera frame not found: {camera['frame']}")

    # ROS optical +Z faces forward, while a MuJoCo camera faces local -Z.
    fovy = math.degrees(2 * math.atan(
        math.tan(camera['horizontal_fov_rad'] / 2) * camera['height'] / camera['width']))
    body.add_camera(name=camera['name'], quat=[0, 1, 0, 0], fovy=fovy,
                    resolution=[camera['width'], camera['height']])
