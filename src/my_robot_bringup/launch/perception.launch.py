"""Start the color segmentation and depth localization pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


# Empty overrides leave the corresponding value in perception.yaml unchanged.
TOPIC_ARGUMENTS = {
    'image_topic': ('color_detector', 'RGB image input'),
    'labels_topic': ('both', 'Color label image'),
    'debug_topic': ('color_detector', 'Color debug image'),
    'depth_topic': ('depth_localizer', 'Aligned depth image input'),
    'camera_info_topic': ('depth_localizer', 'Camera calibration input'),
    'points_topic': ('depth_localizer', 'Localized point cloud output'),
    'objects_topic': ('depth_localizer', 'Detected objects output'),
}


def launch_setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    config = yaml.safe_load(Path(argument('perception_config')).read_text())
    color = dict(config['color_detector']['ros__parameters'])
    depth = dict(config['depth_localizer']['ros__parameters'])
    for name, (target, _) in TOPIC_ARGUMENTS.items():
        value = argument(name)
        if value:
            if target in ('color_detector', 'both'):
                color[name] = value
            if target in ('depth_localizer', 'both'):
                depth[name] = value

    for name in ('show_window', 'publish_debug'):
        value = argument(name).lower()
        if value:
            if value not in ('true', 'false'):
                raise ValueError(f'{name} must be true or false')
            color[name] = value == 'true'
    use_sim_time = argument('use_sim_time').lower() == 'true'
    color['use_sim_time'] = depth['use_sim_time'] = use_sim_time
    return [
        Node(package='my_robot_perception', executable='color_detector',
             parameters=[color], output='screen'),
        Node(package='my_robot_perception', executable='depth_localizer',
             parameters=[depth], output='screen'),
    ]


def generate_launch_description():
    perception = Path(get_package_share_directory('my_robot_perception'))
    arguments = [
        DeclareLaunchArgument('perception_config', default_value=str(perception / 'config/perception.yaml'),
                              description='Color and depth node parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='Use ROS simulation time'),
        DeclareLaunchArgument('show_window', default_value='',
                              description='Show color debug window; empty uses perception_config'),
        DeclareLaunchArgument('publish_debug', default_value='',
                              description='Publish color debug image; empty uses perception_config'),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value='',
                     description=description + '; empty uses perception_config')
                     for name, (_, description) in TOPIC_ARGUMENTS.items())
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
