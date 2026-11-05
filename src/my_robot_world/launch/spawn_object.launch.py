import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(get_package_share_directory('my_robot_world'), 'objects')
    model = LaunchConfiguration('model')

    arguments = [
        DeclareLaunchArgument('ros_domain_id', default_value='227'),
        DeclareLaunchArgument('localhost_only', default_value='1'),
        DeclareLaunchArgument(
            'model', default_value='blue_cube',
            choices=['blue_cube', 'green_cube', 'red_cube', 'aruco_model'],
            description='my_robot_world 下的模型目录名。'),
        DeclareLaunchArgument(
            'entity', default_value=model,
            description='Gazebo 实例名；重复插入同一模型时需使用不同名称。'),
    ]
    # 默认放在地面附近；桌面场景需调整 z，位置单位 m，角度单位 rad。
    for name, default in [
        ('x', '0.4'), ('y', '0.0'), ('z', '0.006'),
        ('roll', '0.0'), ('pitch', '0.0'), ('yaw', '0.0'),
        ('timeout', '30.0'),
    ]:
        arguments.append(DeclareLaunchArgument(name, default_value=default))

    # 此处仅设置子进程环境；已运行的 Gazebo 也需预先配置模型搜索路径。
    model_environment = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.pathsep.join(filter(None, [
            model_path, os.environ.get('GAZEBO_MODEL_PATH', '')])),
    )
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', LaunchConfiguration('entity'),
            '-file', PathJoinSubstitution([model_path, model, 'model.sdf']),
            '-reference_frame', 'world',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw'),
            '-timeout', LaunchConfiguration('timeout'),
        ],
    )
    return LaunchDescription(arguments + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', LaunchConfiguration('localhost_only')),
        model_environment, spawn,
    ])
