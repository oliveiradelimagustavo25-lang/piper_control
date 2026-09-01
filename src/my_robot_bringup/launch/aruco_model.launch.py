import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory,get_package_prefix
import os
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    x_arg = DeclareLaunchArgument("x",default_value="0.5")
    y_arg = DeclareLaunchArgument("y",default_value="0.1")
    z_arg = DeclareLaunchArgument("z",default_value="0.25")
    pitch_arg = DeclareLaunchArgument("pitch",default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw",default_value="1.57")
    roll_arg = DeclareLaunchArgument("roll",default_value="0.0")
    label_arg = DeclareLaunchArgument("label",default_value="0")

    x=launch.substitutions.LaunchConfiguration("x")
    y=launch.substitutions.LaunchConfiguration("y")
    z=launch.substitutions.LaunchConfiguration("z")

    pitch=launch.substitutions.LaunchConfiguration("pitch")
    yaw=launch.substitutions.LaunchConfiguration("yaw")
    roll=launch.substitutions.LaunchConfiguration("roll")

    label =launch.substitutions.LaunchConfiguration("label")

    action_spawn_entity = launch_ros.actions.Node(  #使用spawn_entity节点，调用/gazebo的服务端，加载模型进入gazeboservice整体引擎
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",["marker", label],

                "-database","aruco_board",

                "-x", x,
                "-y", y,
                "-z", z,

                "-R",roll,
                "-P",pitch,
                "-Y",yaw,
        ],
        output="screen"
    )
    

    return launch.LaunchDescription([  
        x_arg,
        y_arg,
        z_arg,
        pitch_arg,
        yaw_arg,
        roll_arg,
        label_arg,
        action_spawn_entity,
       
    ])