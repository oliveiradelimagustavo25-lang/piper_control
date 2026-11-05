from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    spawn_launch = os.path.join(
        get_package_share_directory('my_robot_world'), 'launch', 'spawn_object.launch.py')
    actions = [
        DeclareLaunchArgument('ros_domain_id', default_value='227'),
        DeclareLaunchArgument('localhost_only', default_value='1'),
        DeclareLaunchArgument('x', default_value='0.30'),
        DeclareLaunchArgument(
            'z', default_value='0.006',
            description='物块中心高度；桌面场景需按桌面高度调整。'),
    ]

    # 三个物块沿 y 方向排列，每个 include 使用独立参数作用域。
    for color, default_y in [('red', '-0.06'), ('green', '0.0'), ('blue', '0.06')]:
        y_argument = color + '_y'
        actions.append(DeclareLaunchArgument(y_argument, default_value=default_y))
        actions.append(GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
                launch_arguments={
                    'model': color + '_cube',
                    'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                    'localhost_only': LaunchConfiguration('localhost_only'),
                    'entity': color + '_cube',
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration(y_argument),
                    'z': LaunchConfiguration('z'),
                }.items(),
            ),
        ]))

    return LaunchDescription(actions)
