from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('piper', package_name='my_robot_moveit_pkg').to_moveit_configs()
    )
    launch = LaunchDescription([
        DeclareBooleanLaunchArg('debug', default_value=False),
        DeclareLaunchArgument('rviz_config',
                              default_value=str(moveit_config.package_path / 'config/moveit.rviz')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='Use the MuJoCo simulation clock'),
    ])
    # Send the SRDF as a ROS parameter file; move_group may not publish it as a topic.
    rviz_parameters = [
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
    ]
    add_debuggable_node(
        launch, package='rviz2', executable='rviz2', output='log', respawn=False,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=rviz_parameters,
    )
    return launch
