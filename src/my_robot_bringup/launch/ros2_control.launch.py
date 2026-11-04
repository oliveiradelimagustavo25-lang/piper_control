"""Reusable robot state publisher, MuJoCo hardware and trajectory controllers."""

from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def launch_setup(context):
    description_dir = Path(get_package_share_directory('my_robot_description'))
    controllers = LaunchConfiguration('controllers_file').perform(context)
    description = LaunchConfiguration('robot_description').perform(context)
    if not description:
        # Standalone stage test: generate the robot-only description.
        speed = LaunchConfiguration('sim_speed_factor').perform(context)
        if not math.isfinite(float(speed)) or float(speed) <= 0:
            raise ValueError('sim_speed_factor must be a finite positive number')
        description = xacro.process_file(
            str(description_dir / 'urdf/piper_control.urdf.xacro'),
            mappings={
                'headless': LaunchConfiguration('headless').perform(context),
                'sim_speed_factor': speed,
                'model_path': LaunchConfiguration('model_path').perform(context),
            },
        ).toxml()
    plugin_file = LaunchConfiguration('plugins_file').perform(context)
    control_parameters = [controllers]
    if plugin_file:
        control_parameters.append(plugin_file)
    control_parameters.append({'use_sim_time': True})
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(description, value_type=str),
                          'use_sim_time': True}], output='screen'),
        Node(package='mujoco_ros2_control', executable='ros2_control_node',
             parameters=control_parameters, output='screen',
             remappings=[('~/robot_description', '/robot_description')], on_exit=Shutdown()),
    ]
    for name in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller'):
        nodes.append(Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--param-file', controllers],
        ))
    return nodes


def generate_launch_description():
    description = Path(get_package_share_directory('my_robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Run without the MuJoCo window'),
        DeclareLaunchArgument('sim_speed_factor', default_value='1.0',
                              description='Positive simulation speed relative to wall time'),
        DeclareLaunchArgument('model_path', default_value=str(description / 'mjcf/control.xml'),
                              description='Robot-only model, unless overridden by the parent launch'),
        DeclareLaunchArgument('controllers_file',
                              default_value=str(description / 'config/ros2_controllers.yaml'),
                              description='ros2_control controller configuration'),
        DeclareLaunchArgument('plugins_file', default_value='',
                              description='Optional MuJoCo plugin configuration'),
        # The complete-world entry supplies its already expanded control description.
        DeclareLaunchArgument('robot_description', default_value='',
                              description='Internal override supplied by the world launch'),
        OpaqueFunction(function=launch_setup),
    ])
