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
    robot_description_package_name = "demo_robot_pkg"

    robot_description_package_path = get_package_share_directory(robot_description_package_name)
    default_gazebo_world_path = os.path.join(robot_description_package_path,"world","room.world")
    gazebo_launch_path = os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    #moveit_launch_path = os.path.join(get_package_share_directory("my_robot_moveit_pkg"), "launch", "move_group.launch.py")
    moveit_config = MoveItConfigsBuilder("piper", package_name="my_robot_moveit_pkg").to_moveit_configs()

    use_rviz_arg = DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the MoveIt plugin.",
    )
    
    gui_arg = DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the Gazebo client.",
    )

    robot_description_xacro_arg =DeclareLaunchArgument(
        name="robot_model",
        default_value=str(os.path.join(robot_description_package_path, "urdf", "gazebo_piper_description.xacro")),
        description="加载model.xacro路径"
    )

    robot_desctiption = ParameterValue(
        launch.substitutions.Command(
            [
                "xacro ",
                launch.substitutions.LaunchConfiguration("robot_model"),
            ]
        ),
        value_type=str,
    )

    gui = launch.substitutions.LaunchConfiguration("gui")
    use_rviz = launch.substitutions.LaunchConfiguration("use_rviz")

    
    action_robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desctiption, "use_sim_time": True}],
        output="screen"
    )

    environment_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH",os.path.join(get_package_prefix(robot_description_package_name), "share"))
    disable_gazebo_model_database = SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI",value="")

    pkg_share=os.pathsep+os.path.join(get_package_prefix(robot_description_package_name), "share")
    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"]+=pkg_share
    else:
        os.environ["GAZEBO_MODEL_PATH"] ="/usr/share/gazebo-11/models"+pkg_share
    
    
     # 加载Gazebo世界模型
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(  #加载gazeboservice整体引擎进程(不是节点，只是一个run()的while循环)，运行世界模型；初始化使用gazebo_ros_init插件创建/gazebo子节点，使用executor.spin_more()运行/gazebo子节点，是世界模型与ros世界的关键
            PythonLaunchDescriptionSource(
                gazebo_launch_path,
            ),
            launch_arguments=[("world", default_gazebo_world_path), ("verbose", "true"),("gui", gui)]
    )
    
        # Spawn Entity  
    action_spawn_entity = launch_ros.actions.Node(  #使用spawn_entity节点，调用/gazebo的服务端，加载模型进入gazeboservice整体引擎
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "/robot_description", "-entity", "piper"],
            output="screen"
    )
    
    
    action_load_joint_state_controller = launch_ros.actions.Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen"
        )
    
    action_load_arm_controller = launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen"
            )
    action_load_gripper_controller = launch_ros.actions.Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "gripper_controller",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                    output="screen"
                )
     
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=action_spawn_entity,
            on_exit=[
                action_load_joint_state_controller,
                action_load_arm_controller,
                action_load_gripper_controller,

            ],
        )
    )

    action_launch_move_group = launch_ros.actions.Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    action_rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=launch.conditions.IfCondition(use_rviz),
        arguments=["-d", os.path.join(get_package_share_directory("my_robot_moveit_pkg"), "config", "moveit.rviz")],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    action_launch_commander = launch_ros.actions.Node(
        package="my_robot_commander",
        executable="commander",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    action_launch_camera = launch_ros.actions.Node(
        package="my_robot_commander",
        executable="camera_image",
        parameters=[
            {"use_sim_time": True},
        ],
    )


    
    

    return launch.LaunchDescription([
        robot_description_xacro_arg,
        gui_arg,
        use_rviz_arg,
        environment_variable,
        disable_gazebo_model_database,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        start_controllers_after_spawn,
        action_launch_move_group,
        action_rviz,  
        action_launch_commander,
        action_launch_camera,
    ])