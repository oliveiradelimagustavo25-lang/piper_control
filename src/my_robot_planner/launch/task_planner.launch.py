"""Start the planner directly, using its configured class catalog and geometry."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    parameters=yaml.safe_load(Path(argument('config')).read_text())['task_planner']['ros__parameters']
    if any(name.startswith('place_positions.') for name in parameters):
        parameters['use_position_rules']=True
    # ROS launch cannot infer empty-array types; keep the node's typed defaults.
    parameters={name:value for name,value in parameters.items() if value!=[]}
    parameters.update(use_sim_time=argument('use_sim_time')=='true',
                      perception_config=argument('perception_config'),
                      class_config_override=argument('class_config'))
    return [Node(package='my_robot_planner',executable='task_planner',output='screen',parameters=[parameters])]


def generate_launch_description():
    planner=Path(get_package_share_directory('my_robot_planner'))
    perception=Path(get_package_share_directory('my_robot_perception'))
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(planner/'config/task_planner.yaml')),
        DeclareLaunchArgument('use_sim_time',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('perception_config',default_value=str(perception/'config/perception.yaml')),
        DeclareLaunchArgument('class_config',default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
