"""Launch the complete MuJoCo world, ros2_control, MoveIt and execution APIs."""

import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

from my_robot_world.scenes.scene import export_scene


# Frequent execution options are arguments; geometry and grasp details stay in YAML.
EXECUTION_ARGUMENTS = {
    'action_name': (str, 'PickPlace action name'),
    'move_to_named_action': (str, 'MoveToNamed action name'),
    'arm_group': (str, 'MoveIt arm planning group'),
    'gripper_group': (str, 'MoveIt gripper planning group'),
    'reference_frame': (str, 'Execution target reference frame'),
    'end_effector_link': (str, 'MoveIt tool link'),
    'velocity_scaling': (float, 'Motion velocity scaling'),
    'acceleration_scaling': (float, 'Motion acceleration scaling'),
    'planning_time': (float, 'Motion planning timeout in seconds'),
    'cartesian_step': (float, 'Cartesian interpolation step in metres'),
}


# The detailed color thresholds and localization settings remain in perception_config.
PERCEPTION_ARGUMENTS = {
    'perception_config': 'Color and depth node parameters',
    'image_topic': 'RGB image input',
    'labels_topic': 'Color label image',
    'debug_topic': 'Color debug image',
    'depth_topic': 'Aligned depth image input',
    'camera_info_topic': 'Camera calibration input',
    'points_topic': 'Localized point cloud output',
    'objects_topic': 'Detected objects output',
    'show_window': 'Show color debug window',
    'publish_debug': 'Publish color debug image',
}


def motion_nodes(context, robot_parameters, with_executor):
    """The executor exposes actions and uses the linked MotionController library."""
    from moveit_configs_utils import MoveItConfigsBuilder

    pipeline = LaunchConfiguration('planning_pipeline').perform(context)
    config = (MoveItConfigsBuilder('piper', package_name='my_robot_moveit_pkg')
              .robot_description(file_path='config/piper.urdf.xacro')
              .trajectory_execution(file_path=LaunchConfiguration('moveit_controllers_file').perform(context))
              .planning_pipelines(default_planning_pipeline=pipeline, pipelines=[pipeline])
              .to_moveit_configs())
    parameters = config.to_dict()
    parameters.update(robot_parameters)
    nodes = [Node(package='moveit_ros_move_group', executable='move_group',
                  parameters=[parameters], output='screen', on_exit=Shutdown())]
    if with_executor:
        overrides = {}
        for name, (value_type, _) in EXECUTION_ARGUMENTS.items():
            value = LaunchConfiguration(name).perform(context)
            if value:
                overrides[name] = value_type(value)
        config_file = Path(LaunchConfiguration('manipulation_config').perform(context))
        execution = yaml.safe_load(config_file.read_text())['pick_place_executor']['ros__parameters']
        # Merge before serialization: Humble's named YAML section can otherwise
        # take precedence over wildcard sections generated for argument overrides.
        execution_parameters = {**parameters, **execution, **robot_parameters, **overrides}
        nodes.append(Node(
            package='my_robot_commander', executable='pick_place_executor',
            parameters=[execution_parameters],
            output='screen', on_exit=Shutdown(),
        ))
    return nodes


def launch_setup(context):
    """Compose the world once and give its model to the sole physics owner."""
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    with_moveit = IfCondition(LaunchConfiguration('with_moveit')).evaluate(context)
    with_executor = IfCondition(LaunchConfiguration('with_executor')).evaluate(context)
    if with_executor and not with_moveit:
        raise ValueError('with_executor:=true requires with_moveit:=true')
    speed = float(argument('sim_speed_factor'))
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('sim_speed_factor must be a finite positive number')

    bringup = Path(get_package_share_directory('my_robot_bringup'))
    world = Path(get_package_share_directory('my_robot_world'))
    robot = Path(get_package_share_directory('my_robot_description'))
    runtime = tempfile.TemporaryDirectory(prefix='piper_world_')
    model_path = Path(runtime.name) / 'world.xml'
    try:
        # The same scene loader serves the world preview and the controlled runtime.
        export_scene(model_path, world, robot, robot_model=argument('robot_model'),
                     scene_file=argument('scene_file'), camera_config=argument('camera_config'))
        description = xacro.process_file(
            str(robot / 'urdf/piper_control.urdf.xacro'),
            mappings={'headless': argument('headless'), 'sim_speed_factor': argument('sim_speed_factor'),
                      'model_path': str(model_path)},
        ).toxml()
    except Exception:
        runtime.cleanup()
        raise

    with_camera = IfCondition(LaunchConfiguration('with_camera')).evaluate(context)
    with_perception = IfCondition(LaunchConfiguration('with_perception')).evaluate(context)
    plugins_file = ''
    if with_camera:
        # Use the same camera and input topics as the scene and perception nodes.
        camera = yaml.safe_load(Path(argument('camera_config')).read_text())
        perception = yaml.safe_load(Path(argument('perception_config')).read_text())
        color = perception['color_detector']['ros__parameters']
        depth = perception['depth_localizer']['ros__parameters']
        plugin = {
            'type': 'mujoco_ros2_control_plugins/CameraPlugin',
            'camera_publish_rate': float(camera['update_rate']),
            camera['name']: {
                'policy': 'streaming', 'frame_name': camera['frame'],
                'image_topic': argument('image_topic') or color['image_topic'],
                'depth_topic': argument('depth_topic') or depth['depth_topic'],
                'info_topic': argument('camera_info_topic') or depth['camera_info_topic'],
            },
        }
        plugins_file = str(Path(runtime.name) / 'camera_plugin.yaml')
        Path(plugins_file).write_text(yaml.safe_dump({
            '/**': {'ros__parameters': {'mujoco_plugins': {'camera_publisher': plugin}}}
        }))

    robot_parameters = {
        'robot_description': ParameterValue(description, value_type=str),
        'use_sim_time': True,
    }
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch/ros2_control.launch.py')),
        launch_arguments={
            'headless': argument('headless'), 'sim_speed_factor': argument('sim_speed_factor'),
            'robot_description': description, 'controllers_file': argument('controllers_file'),
            'plugins_file': plugins_file,
        }.items(),
    )
    nodes = motion_nodes(context, robot_parameters, with_executor) if with_moveit else []
    if with_perception:
        perception_arguments = {
            name: argument(name) for name in PERCEPTION_ARGUMENTS
        }
        perception_arguments['use_sim_time'] = 'true'
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/perception.launch.py')),
            launch_arguments=perception_arguments.items(),
        ))
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: runtime.cleanup())),
        LogInfo(msg=f'MuJoCo complete world: {model_path}'),
        control, *nodes,
    ]


def generate_launch_description():
    world = Path(get_package_share_directory('my_robot_world'))
    robot = Path(get_package_share_directory('my_robot_description'))
    moveit = Path(get_package_share_directory('my_robot_moveit_pkg'))
    commander = Path(get_package_share_directory('my_robot_commander'))
    perception = Path(get_package_share_directory('my_robot_perception'))
    arguments = [
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Run without the MuJoCo window'),
        DeclareLaunchArgument('with_perception', default_value='true', choices=['true', 'false'],
                              description='Start color detection and depth localization'),
        DeclareLaunchArgument('with_camera', default_value=LaunchConfiguration('with_perception'),
                              choices=['true', 'false'], description='Publish MuJoCo RGB, depth and camera info'),
        DeclareLaunchArgument('with_moveit', default_value='true', choices=['true', 'false'],
                              description='Start the MoveIt motion planning server'),
        DeclareLaunchArgument('with_executor', default_value=LaunchConfiguration('with_moveit'),
                              choices=['true', 'false'], description='Start pick/place and named motion APIs'),
        DeclareLaunchArgument('sim_speed_factor', default_value='1.0',
                              description='Positive simulation speed relative to wall time'),
        DeclareLaunchArgument('scene_file', default_value=str(world / 'basic/preview.xml'),
                              description='World scene including ground, objects and robot placeholder'),
        DeclareLaunchArgument('robot_model', default_value=str(robot / 'mjcf/control.xml'),
                              description='Robot MJCF including ros2_control actuators'),
        DeclareLaunchArgument('camera_config', default_value=str(world / 'config/camera.yaml'),
                              description='MuJoCo wrist camera configuration'),
        DeclareLaunchArgument('controllers_file', default_value=str(robot / 'config/ros2_controllers.yaml'),
                              description='ros2_control controllers and tolerances'),
        DeclareLaunchArgument('moveit_controllers_file',
                              default_value=str(moveit / 'config/moveit_controllers.yaml'),
                              description='MoveIt trajectory controller mapping'),
        DeclareLaunchArgument('planning_pipeline', default_value='ompl',
                              description='MoveIt planning pipeline'),
        DeclareLaunchArgument('manipulation_config', default_value=str(commander / 'config/manipulation.yaml'),
                              description='Executor and MotionController options'),
    ]
    for name, description in PERCEPTION_ARGUMENTS.items():
        default = str(perception / 'config/perception.yaml') if name == 'perception_config' else ''
        suffix = '' if name == 'perception_config' else '; empty uses perception_config'
        arguments.append(DeclareLaunchArgument(name, default_value=default, description=description + suffix))
    for name, (_, description) in EXECUTION_ARGUMENTS.items():
        arguments.append(DeclareLaunchArgument(
            name, default_value='', description=description + '; empty uses manipulation_config',
        ))
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
