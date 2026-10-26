import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robot_path = get_package_share_directory('demo_robot_pkg')
    model_path = get_package_share_directory('my_robot_model')
    perception_path = get_package_share_directory('my_robot_perception')
    moveit_config = MoveItConfigsBuilder(
        'piper', package_name='my_robot_moveit_pkg').to_moveit_configs()
    # 暂不启用模板中的 Kinect/OctoMap 配置。
    moveit_config.sensors_3d = {}
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('robot_model')]), value_type=str)
    robot_params = {
        'robot_description': robot_description,
        'use_sim_time': True,
    }
    moveit_params = [moveit_config.to_dict(), robot_params]

    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('robot_model', default_value=os.path.join(
            robot_path, 'urdf', 'gazebo_piper_description.xacro')),
        # 仅作用于本 launch 的子进程，不修改用户 Shell 配置。
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        DeclareLaunchArgument('localhost_only', default_value='1'),
    ]
    environment = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', LaunchConfiguration('localhost_only')),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(filter(None, [
            '/usr/share/gazebo-11/models',
            os.path.join(get_package_prefix('demo_robot_pkg'), 'share'),
            model_path, os.environ.get('GAZEBO_MODEL_PATH', ''),
        ]))),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', os.pathsep.join(filter(None, [
            '/usr/share/gazebo-11', model_path,
            os.environ.get('GAZEBO_RESOURCE_PATH', ''),
        ]))),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(robot_path, 'world', 'room.world'),
            'verbose': 'true', 'gui': LaunchConfiguration('gui'),
        }.items(),
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', '/robot_description', '-entity', 'piper', '-timeout', '60'],
    )
    controllers = [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[name, '--controller-manager', '/controller_manager'])
        for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']
    ]

    def after_spawn(event, context):
        # 生成失败时不继续启动控制器，保留最初的错误信息。
        if event.returncode != 0:
            return [LogInfo(msg='Piper spawn failed; controllers were not started.')]
        return controllers

    controller_handler = RegisterEventHandler(OnProcessExit(
        target_action=spawn, on_exit=after_spawn))
    config_file = os.path.join(perception_path, 'config', 'perception.yaml')
    perception = [
        Node(package='my_robot_perception', executable=name, output='screen',
             parameters=[config_file, {'use_sim_time': True}])
        for name in ['color_detector', 'depth_localizer']
    ]
    return LaunchDescription(args + environment + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_params], output='screen'),
        gazebo,
        controller_handler,
        spawn,
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=moveit_params + [{
                 'allow_trajectory_execution': True,
                 'publish_robot_description': True,
                 'publish_robot_description_semantic': True,
                 'publish_planning_scene': True,
                 'publish_geometry_updates': True,
                 'publish_state_updates': True,
                 'publish_transforms_updates': True,
             }]),
        Node(package='rviz2', executable='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', os.path.join(get_package_share_directory(
                 'my_robot_moveit_pkg'), 'config', 'moveit.rviz')],
             parameters=moveit_params),
        Node(package='my_robot_commander', executable='commander', output='screen',
             parameters=moveit_params),
    ] + perception)
