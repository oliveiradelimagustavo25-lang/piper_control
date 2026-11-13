"""Start the depth localizer independently."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('my_robot_perception')) / 'config/perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('perception_config', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        Node(package='my_robot_perception', executable='depth_localizer', output='screen',
             parameters=[LaunchConfiguration('perception_config'),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
