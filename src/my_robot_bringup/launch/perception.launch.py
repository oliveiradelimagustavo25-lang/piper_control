"""Start one selected 2D detector and the common depth localization node."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


# Empty overrides leave the corresponding value in perception.yaml unchanged.
TOPIC_ARGUMENTS = {
    'image_topic': ('detector', 'RGB image input'),
    'labels_topic': ('both', 'Class label image'),
    'debug_topic': ('detector', 'Detector debug image'),
    'depth_topic': ('depth_localizer', 'Aligned depth image input'),
    'camera_info_topic': ('depth_localizer', 'Camera calibration input'),
    'points_topic': ('depth_localizer', 'Localized point cloud output'),
    'objects_topic': ('depth_localizer', 'Detected objects output'),
}


def launch_setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    config = yaml.safe_load(Path(argument('perception_config')).read_text())
    shared = config.get('/**', {}).get('ros__parameters', {})
    detector = argument('detector') or shared.get('detector', 'cv')
    if detector not in ('cv', 'yolo'):
        raise ValueError('detector must be cv or yolo')
    executable = 'color_detector' if detector == 'cv' else 'yolo_detector'
    if detector == 'yolo':
        binary = Path(get_package_prefix('my_robot_perception')) / 'lib/my_robot_perception/yolo_detector'
        if not binary.is_file():
            raise RuntimeError('YOLO adapter is not built; install tools/yolo SDK and build '
                               'my_robot_perception with -DBUILD_YOLO=ON (see tools/yolo/README.md)')
    detection = {**shared, **config[executable]['ros__parameters']}
    depth = {**shared, **config['depth_localizer']['ros__parameters']}
    for parameters in (detection, depth):
        parameters.update(perception_config=argument('perception_config'),
                          class_config_override=argument('class_config'))
    for name, (target, _) in TOPIC_ARGUMENTS.items():
        value = argument(name)
        if value:
            if target in ('detector', 'both'):
                detection[name] = value
            if target in ('depth_localizer', 'both'):
                depth[name] = value

    for name in ('show_window', 'publish_debug'):
        value = argument(name).lower()
        if value:
            if value not in ('true', 'false'):
                raise ValueError(f'{name} must be true or false')
            detection[name] = value == 'true'
    if detection.get('labels_topic') != depth.get('labels_topic'):
        raise ValueError('Detector and depth labels_topic differ; set labels_topic:=... to override both')
    if detector == 'yolo':
        detection['model_path_override'] = argument('yolo_model')
    use_sim_time = argument('use_sim_time').lower() == 'true'
    detection['use_sim_time'] = depth['use_sim_time'] = use_sim_time
    return [
        Node(package='my_robot_perception', executable=executable,
             parameters=[detection], output='screen'),
        Node(package='my_robot_perception', executable='depth_localizer',
             parameters=[depth], output='screen'),
    ]


def generate_launch_description():
    perception = Path(get_package_share_directory('my_robot_perception'))
    arguments = [
        DeclareLaunchArgument('perception_config', default_value=str(perception / 'config/perception.yaml'),
                              description='Detector and depth node parameters'),
        DeclareLaunchArgument('detector', default_value='',
                              description='cv or yolo; empty uses perception_config'),
        DeclareLaunchArgument('yolo_model', default_value='',
                              description='Explicit ONNX path; empty uses yolo_detector.model_path'),
        DeclareLaunchArgument('class_config', default_value='',
                              description='Shared class catalog; empty uses perception_config'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'],
                              description='Use ROS simulation time'),
        DeclareLaunchArgument('show_window', default_value='',
                              description='Show detector debug window; empty uses perception_config'),
        DeclareLaunchArgument('publish_debug', default_value='',
                              description='Publish color debug image; empty uses perception_config'),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value='',
                     description=description + '; empty uses perception_config')
                     for name, (_, description) in TOPIC_ARGUMENTS.items())
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
