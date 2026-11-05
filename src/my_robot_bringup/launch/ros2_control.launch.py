"""Start Gazebo Classic, the Piper controllers, and optionally MoveIt."""

import os
from pathlib import Path
import socket
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, SetLaunchConfiguration, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def check_gazebo_master():
    """Fail before starting nodes when the configured Gazebo master is occupied."""
    uri = os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345')
    address = urlparse(uri)
    if not address.hostname or not address.port:
        raise ValueError(f'Invalid GAZEBO_MASTER_URI: {uri}')
    try:
        with socket.create_connection((address.hostname, address.port), timeout=0.5):
            raise RuntimeError(
                f'Gazebo master {uri} is already in use. Stop the existing Gazebo '
                'instance or choose another GAZEBO_MASTER_URI before launching.')
    except ConnectionRefusedError:
        pass
    except OSError as error:
        raise RuntimeError(f'Cannot check Gazebo master {uri}: {error}') from error


def launch_setup(context):
    check_gazebo_master()

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot = xacro.process_file(arg('robot_model'), mappings={
        'controllers_file': arg('controllers_file'), 'camera_config': arg('camera_config'),
    }).toxml()
    robot_params = {'robot_description': ParameterValue(robot, value_type=str), 'use_sim_time': True}
    world = Path(get_package_share_directory('my_robot_world'))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros')) /
                                          'launch/gazebo.launch.py')),
        launch_arguments={'world': arg('world'), 'gui': arg('gui'), 'verbose': 'true'}.items())
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', '/robot_description', '-entity', 'piper', '-timeout', '60'])
    controllers = [Node(package='controller_manager', executable='spawner', output='screen',
                        arguments=[name, '--controller-manager', '/controller_manager'])
                   for name in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')]

    def after_spawn(event, _context):
        if event.returncode:
            return [LogInfo(msg='Piper spawn failed; stopping launch.'), Shutdown()]
        return controllers

    spawn_cubes = GroupAction(condition=IfCondition(LaunchConfiguration('spawn_objects')), actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(world / 'launch/spawn_cube.launch.py')),
            launch_arguments={name: arg(name) for name in
                ('ros_domain_id', 'localhost_only', 'x', 'red_y', 'green_y', 'blue_y', 'z')}.items())])
    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', arg('localhost_only')),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(filter(None, [
            str(world / 'objects'), str(Path(get_package_share_directory('my_robot_description')).parent),
            '/usr/share/gazebo-11/models', os.environ.get('GAZEBO_MODEL_PATH', '')]))),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', os.pathsep.join(filter(None, [
            '/usr/share/gazebo-11', os.environ.get('GAZEBO_RESOURCE_PATH', '')]))),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetLaunchConfiguration('_piper_control_description', robot),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_params], output='screen'),
        gazebo, RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        spawn, spawn_cubes,
    ]
    if (os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp'
            and not os.environ.get('CYCLONEDDS_URI') and arg('localhost_only') == '1'):
        config = Path(get_package_share_directory('my_robot_bringup')) / 'config/cyclonedds_local.xml'
        actions.insert(2, SetEnvironmentVariable('CYCLONEDDS_URI', config.as_uri()))
    if IfCondition(LaunchConfiguration('with_moveit')).evaluate(context):
        pipeline = arg('planning_pipeline')
        moveit = (MoveItConfigsBuilder('piper', package_name='my_robot_moveit_pkg')
                  .trajectory_execution(file_path=arg('moveit_controllers_file'))
                  .planning_pipelines(default_planning_pipeline=pipeline, pipelines=[pipeline])
                  .to_moveit_configs())
        moveit.sensors_3d = {}
        params = [moveit.to_dict(), robot_params]
        actions.extend([
            Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                 parameters=params + [{'allow_trajectory_execution': True,
                    'publish_robot_description': True, 'publish_robot_description_semantic': True,
                    'publish_planning_scene': True, 'publish_geometry_updates': True,
                    'publish_state_updates': True, 'publish_transforms_updates': True}]),
            Node(package='rviz2', executable='rviz2', output='log',
                 condition=IfCondition(LaunchConfiguration('use_rviz')),
                 arguments=['-d', str(Path(get_package_share_directory('my_robot_moveit_pkg')) /
                                      'config/moveit.rviz')], parameters=params),
        ])
    return actions


def generate_launch_description():
    world = Path(get_package_share_directory('my_robot_world'))
    robot = Path(get_package_share_directory('my_robot_description'))
    moveit = Path(get_package_share_directory('my_robot_moveit_pkg'))
    defaults = {
        'ros_domain_id': ('227', 'ROS 2 domain'), 'localhost_only': ('1', 'Restrict ROS discovery to localhost'),
        'gui': ('true', 'Show Gazebo window'), 'use_rviz': ('true', 'Show MoveIt RViz'),
        'world': (str(world / 'basic/preview.world'), 'Gazebo world file'),
        'robot_model': (str(robot / 'urdf/piper_control.urdf.xacro'), 'Robot URDF or xacro'),
        'camera_config': (str(world / 'config/camera.yaml'), 'Wrist camera settings'),
        'controllers_file': (str(robot / 'config/ros2_controllers.yaml'), 'ros2_control settings'),
        'moveit_controllers_file': (str(moveit / 'config/moveit_controllers.yaml'), 'MoveIt controller mapping'),
        'planning_pipeline': ('ompl', 'MoveIt planning pipeline'),
        'with_moveit': ('true', 'Start MoveIt'), 'spawn_objects': ('true', 'Spawn three cubes'),
        'x': ('0.30', 'Cube X coordinate'), 'red_y': ('-0.06', 'Red cube Y coordinate'),
        'green_y': ('0.0', 'Green cube Y coordinate'), 'blue_y': ('0.06', 'Blue cube Y coordinate'),
        'z': ('0.006', 'Cube Z coordinate'),
    }
    return LaunchDescription([*(DeclareLaunchArgument(name, default_value=value, description=description)
                                for name, (value, description) in defaults.items()),
                              OpaqueFunction(function=launch_setup)])
