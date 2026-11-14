"""Start only the ROS YOLO adapter; the inference SDK is installed separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('my_robot_perception')) / 'config/perception.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('perception_config', default_value=str(config)),
        DeclareLaunchArgument('class_config', default_value=''),
        DeclareLaunchArgument('yolo_model', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        Node(package='my_robot_perception', executable='yolo_detector', output='screen',
             parameters=[LaunchConfiguration('perception_config'), {
                 'perception_config': LaunchConfiguration('perception_config'),
                 'class_config_override': ParameterValue(LaunchConfiguration('class_config'), value_type=str),
                 'model_path_override': ParameterValue(LaunchConfiguration('yolo_model'), value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
