"""Launch the complete Gazebo world, ros2_control, MoveIt and execution APIs."""

from functools import lru_cache
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import yaml


# Frequent execution options are arguments; geometry and grasp details stay in YAML.
EXECUTION_ARGUMENTS = {
    'action_name': (str, 'Legacy alias for pick_place_action'),
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


# File paths in the external configuration are relative to that YAML file.
CONFIG_PATHS = {
    'world', 'robot_model', 'camera_config', 'controllers_file',
    'moveit_controllers_file', 'manipulation_config', 'planner_config', 'perception_config',
}
CONFIG_KEYS = set(EXECUTION_ARGUMENTS) | set(PERCEPTION_ARGUMENTS) | CONFIG_PATHS | {
    'gui', 'use_rviz', 'with_moveit', 'with_motion', 'with_perception', 'use_task_planner',
    'spawn_objects', 'ros_domain_id', 'localhost_only', 'x', 'red_y', 'green_y', 'blue_y', 'z',
    'planning_pipeline', 'pick_place_action', 'sort_action', 'observation_target',
}


@lru_cache(maxsize=8)
def load_launch_config(path):
    """Read a snapshot for this launch; a new run reloads the YAML file."""
    data = yaml.safe_load(Path(path).read_text())
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError('launch_config must contain a mapping of launch argument names to values')
    if not all(isinstance(name, str) for name in data):
        raise ValueError('launch_config argument names must be strings')
    unknown = set(data) - CONFIG_KEYS
    if unknown:
        raise ValueError(f'Unknown launch_config arguments: {sorted(unknown)}')
    for name, value in data.items():
        if value is not None and not isinstance(value, (str, int, float, bool)):
            raise ValueError(f'launch_config value for {name} must be a scalar')
    return data


class ConfigDefault(Substitution):
    """Use external YAML when present, otherwise retain the normal launch default."""

    def __init__(self, name, fallback):
        super().__init__()
        self.name = name
        self.fallback = normalize_to_list_of_substitutions(fallback)

    def describe(self):
        fallback = ''.join(item.describe() for item in self.fallback)
        return f'launch_config[{self.name}] or {fallback}'

    def perform(self, context):
        config_file = LaunchConfiguration('launch_config').perform(context)
        if config_file:
            path = Path(config_file).expanduser().resolve()
            value = load_launch_config(str(path)).get(self.name)
            if value is not None:
                if isinstance(value, bool):
                    return str(value).lower()
                value = str(value)
                if self.name in CONFIG_PATHS and value:
                    return str((path.parent / value).resolve())
                return value
        return perform_substitutions(context, self.fallback)


def config_argument(name, default_value, **options):
    return DeclareLaunchArgument(name, default_value=ConfigDefault(name, default_value), **options)


def include_layer(bringup, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch' / filename)),
        launch_arguments=arguments.items(),
    )


def control_layer(bringup, argument):
    names = ('gui', 'use_rviz', 'controllers_file', 'with_moveit', 'planning_pipeline',
             'moveit_controllers_file', 'world', 'camera_config', 'robot_model', 'spawn_objects',
             'ros_domain_id', 'localhost_only', 'x', 'red_y', 'green_y', 'blue_y', 'z')
    arguments = {name: argument(name) for name in names}
    return include_layer(bringup, 'ros2_control.launch.py', arguments)


def motion_layer(bringup, argument, objects_topic):
    names = ('planning_pipeline', 'moveit_controllers_file', 'manipulation_config',
             'planner_config', 'pick_place_action', 'sort_action', 'observation_target', 'use_task_planner',
             *EXECUTION_ARGUMENTS)
    arguments = {name: argument(name) for name in names}
    arguments.update(robot_description=LaunchConfiguration('_piper_control_description'), objects_topic=objects_topic)
    return include_layer(bringup, 'motion_control.launch.py', arguments)


def perception_layer(bringup, argument):
    arguments = {name: argument(name) for name in PERCEPTION_ARGUMENTS}
    arguments['use_sim_time'] = 'true'
    return include_layer(bringup, 'perception.launch.py', arguments)


def launch_setup(context):
    """Validate and assemble the three launch layers."""
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    config_file = argument('launch_config')
    if config_file:
        load_launch_config(str(Path(config_file).expanduser().resolve()))

    with_moveit = IfCondition(LaunchConfiguration('with_moveit')).evaluate(context)
    with_motion = IfCondition(LaunchConfiguration('with_motion')).evaluate(context)
    with_perception = IfCondition(LaunchConfiguration('with_perception')).evaluate(context)
    if with_motion and not with_moveit:
        raise ValueError('with_motion:=true requires with_moveit:=true')
    if with_perception and not with_motion:
        raise ValueError('with_perception:=true requires with_motion:=true')
    bringup = Path(get_package_share_directory('my_robot_bringup'))

    # Share the perception output when both layers run; otherwise let the
    # standalone planner use its own configured object source.
    objects_topic = argument('objects_topic')
    if with_perception and not objects_topic:
        perception_config = yaml.safe_load(Path(argument('perception_config')).read_text())
        objects_topic = perception_config['depth_localizer']['ros__parameters']['objects_topic']
    layers = [control_layer(bringup, argument)]
    if with_motion:
        layers.append(motion_layer(bringup, argument, objects_topic))
    if with_perception:
        layers.append(perception_layer(bringup, argument))
    return layers


def generate_launch_description():
    world = Path(get_package_share_directory('my_robot_world'))
    robot = Path(get_package_share_directory('my_robot_description'))
    moveit = Path(get_package_share_directory('my_robot_moveit_pkg'))
    commander = Path(get_package_share_directory('my_robot_commander'))
    perception = Path(get_package_share_directory('my_robot_perception'))
    planner = Path(get_package_share_directory('my_robot_planner'))
    arguments = [
        DeclareLaunchArgument('launch_config', default_value='',
                              description='Optional external YAML with launch argument defaults'),
        config_argument('gui', default_value='true', description='Show Gazebo window'),
        config_argument('use_rviz', default_value='true', description='Show MoveIt RViz'),
        config_argument('with_moveit', default_value='true', description='Start MoveIt'),
        config_argument('with_motion', default_value=LaunchConfiguration('with_moveit'), description='Start executor and planner'),
        config_argument('with_perception', default_value=LaunchConfiguration('with_motion'), description='Start perception nodes'),
        config_argument('use_task_planner', default_value='true', description='Start the sorting planner'),
        config_argument('spawn_objects', default_value='true', description='Spawn three cubes'),
        config_argument('ros_domain_id', default_value='227', description='ROS 2 domain'),
        config_argument('localhost_only', default_value='1', description='Restrict ROS discovery to localhost'),
        config_argument('world', default_value=str(world / 'basic/preview.world'), description='Gazebo world'),
        config_argument('robot_model', default_value=str(robot / 'urdf/piper_control.urdf.xacro'), description='Robot URDF or xacro'),
        config_argument('camera_config', default_value=str(world / 'config/camera.yaml'), description='Wrist camera settings'),
        config_argument('controllers_file', default_value=str(robot / 'config/ros2_controllers.yaml'), description='ros2_control settings'),
        config_argument('moveit_controllers_file', default_value=str(moveit / 'config/moveit_controllers.yaml'), description='MoveIt controller mapping'),
        *(config_argument(name, default_value=value, description='Cube position in metres')
          for name, value in [('x', '0.30'), ('red_y', '-0.06'), ('green_y', '0.0'), ('blue_y', '0.06'), ('z', '0.006')]),
        config_argument('planning_pipeline', default_value='ompl',
                              description='MoveIt planning pipeline'),
        config_argument('planner_config', default_value=str(planner / 'config/task_planner.yaml'),
                              description='Sorting task planner configuration'),
        config_argument('pick_place_action', default_value='', description='Pick/place action; empty uses YAML'),
        config_argument('sort_action', default_value='', description='Sorting action; empty uses YAML'),
        config_argument('observation_target', default_value='', description='Observation pose; empty uses YAML'),
        config_argument('manipulation_config', default_value=str(commander / 'config/manipulation.yaml'),
                              description='Executor and MotionController options'),
    ]
    for name, description in PERCEPTION_ARGUMENTS.items():
        default = str(perception / 'config/perception.yaml') if name == 'perception_config' else ''
        suffix = '' if name == 'perception_config' else '; empty uses perception_config'
        arguments.append(config_argument(name, default_value=default, description=description + suffix))
    for name, (_, description) in EXECUTION_ARGUMENTS.items():
        arguments.append(config_argument(
            name, default_value='', description=description + '; empty uses manipulation_config',
        ))
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
