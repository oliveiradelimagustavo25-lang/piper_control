from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    spawn_launch = os.path.join(
        get_package_share_directory('my_robot_sim'), 'launch', 'spawn_object.launch.py')
    actions = [
        DeclareLaunchArgument('ros_domain_id', default_value='95'),
        DeclareLaunchArgument('x', default_value='0.4'),
        DeclareLaunchArgument(
            'z', default_value='0.006',
            description='物块中心高度；桌面场景需按桌面高度调整。'),
    ]

    # 三个物块沿 y 方向排列，每个 include 使用独立参数作用域。
    for color, default_y in [('red', '-0.1'), ('green', '0.0'), ('blue', '0.1')]:
        y_argument = color + '_y'
        actions.append(DeclareLaunchArgument(y_argument, default_value=default_y))
        actions.append(GroupAction(actions=[   #GroupAction会和 IncludeLaunchDescription一同使用，本质上是清除 launch_arguments 的作用域，避免参数冲突。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
                launch_arguments={  #launch_arguments 是一个字典，包含了传递给被包含的 launch 文件的参数。注意一定要有 items() 方法，因为 launch_arguments 需要一个字典的迭代器，而不是字典本身。
                    'model': color + '_cube',  #其实也可以写成[('model', color + '_cube'), ('entity', color + '_cube')]，这样就没有字典的语义问题了。
                    'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                    'entity': color + '_cube',
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration(y_argument),
                    'z': LaunchConfiguration('z'),
                }.items(),
            ),
        ]))

    return LaunchDescription(actions)
