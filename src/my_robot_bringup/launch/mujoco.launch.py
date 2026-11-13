"""Launch the complete MuJoCo world, ros2_control, MoveIt and execution APIs."""

from functools import lru_cache
from pathlib import Path
import importlib.util
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
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


# Class properties live in class_config; image/depth processing stays in perception_config.
PERCEPTION_ARGUMENTS = {
    'perception_config': 'Color and depth node parameters',
    'class_config': 'Shared class catalog for color detection and depth localization',
    'image_topic': 'RGB image input',
    'labels_topic': 'Class label image',
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
    'scene_file', 'robot_model', 'camera_config', 'controllers_file',
    'moveit_controllers_file', 'manipulation_config', 'planner_config', 'perception_config', 'class_config',
}
CONFIG_KEYS = set(EXECUTION_ARGUMENTS) | set(PERCEPTION_ARGUMENTS) | CONFIG_PATHS | {
    'headless', 'with_moveit', 'with_rviz', 'with_executor', 'with_motion', 'with_perception', 'with_camera',
    'sim_speed_factor', 'planning_pipeline', 'pick_place_action', 'sort_action', 'observation_target', 'scenes', 'place_position_modes', 'place_positions', 'dry_run',
}


@lru_cache(maxsize=1)
def placement_config():
    path = Path(get_package_share_directory('my_robot_bringup')) / 'build/placement_config.py'
    spec = importlib.util.spec_from_file_location('piper_placement_config', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(maxsize=1)
def scene_config():
    path = Path(get_package_share_directory('my_robot_bringup')) / 'build/scene_config.py'
    spec = importlib.util.spec_from_file_location('piper_scene_config', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    # Placement values are validated after CLI/YAML precedence is resolved.
    for name, value in data.items():
        if name in ('place_position_modes', 'place_positions'):
            continue
        if name == 'scenes':
            if value is not None and not isinstance(value, dict):
                raise ValueError('scenes must map class IDs to lists of [x, y, z]')
            continue
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
            config = load_launch_config(str(path))
            value = config.get(self.name)
            if value is not None:
                if self.name in ('place_position_modes', 'place_positions'):
                    return yaml.safe_dump(value, default_flow_style=True, width=100000).strip()
                if isinstance(value, bool):
                    return str(value).lower()
                value = str(value)
                if self.name in CONFIG_PATHS and value:
                    return str((path.parent / value).resolve())
                return value
        if self.name == 'scene_file':
            # Only a defaulted scene may be generated. Resolve the catalog after
            # all arguments exist, so an explicit CLI class_config wins as well.
            context.launch_configurations['_piper_generate_scene'] = 'true'
        return perform_substitutions(context, self.fallback)


def config_argument(name, default_value, **options):
    return DeclareLaunchArgument(name, default_value=ConfigDefault(name, default_value), **options)


def include_layer(bringup, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch' / filename)),
        launch_arguments=arguments.items(),
    )


def control_layer(bringup, argument):
    names = ('headless', 'sim_speed_factor', 'controllers_file', 'with_moveit', 'with_camera',
             'planning_pipeline', 'moveit_controllers_file', 'scene_file', 'camera_config',
             'perception_config', 'image_topic', 'depth_topic', 'camera_info_topic')
    arguments = {name: argument(name) for name in names}
    arguments['model_path'] = argument('robot_model')
    return include_layer(bringup, 'ros2_control.launch.py', arguments)


def motion_layer(bringup, argument, objects_topic):
    names = ('planning_pipeline', 'moveit_controllers_file', 'manipulation_config',
             'planner_config', 'perception_config', 'class_config', 'pick_place_action', 'sort_action', 'observation_target',
             'place_position_modes', 'place_positions',
             *EXECUTION_ARGUMENTS)
    arguments = {name: argument(name) for name in names}
    arguments.update(robot_description=LaunchConfiguration('_piper_control_description'), objects_topic=objects_topic)
    return include_layer(bringup, 'motion_control.launch.py', arguments)


def perception_layer(bringup, argument):
    arguments = {name: argument(name) for name in PERCEPTION_ARGUMENTS}
    arguments['use_sim_time'] = 'true'
    return include_layer(bringup, 'perception.launch.py', arguments)


def rviz_layer():
    """Reuse the MoveIt RViz launch and the simulation clock."""
    moveit = Path(get_package_share_directory('my_robot_moveit_pkg'))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit / 'launch' / 'moveit_rviz.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )


def launch_setup(context):
    """Validate and assemble the three launch layers."""
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    config_file = argument('launch_config')
    if config_file:
        load_launch_config(str(Path(config_file).expanduser().resolve()))

    with_moveit = IfCondition(LaunchConfiguration('with_moveit')).evaluate(context)
    with_rviz = IfCondition(LaunchConfiguration('with_rviz')).evaluate(context)
    with_motion = IfCondition(LaunchConfiguration('with_motion')).evaluate(context)
    with_perception = IfCondition(LaunchConfiguration('with_perception')).evaluate(context)
    if with_motion and not with_moveit:
        raise ValueError('with_motion:=true requires with_moveit:=true')
    if with_rviz and not with_moveit:
        raise ValueError('with_rviz:=true requires with_moveit:=true')
    if with_perception and not with_motion:
        raise ValueError('with_perception:=true requires with_motion:=true')
    bringup = Path(get_package_share_directory('my_robot_bringup'))

    catalog_path = scene_config().catalog_file(argument('perception_config'), argument('class_config'))
    context.launch_configurations['class_config'] = str(catalog_path)
    if context.launch_configurations.get('_piper_generate_scene') == 'true':
        directory = tempfile.mkdtemp(prefix='piper_scene_')
        try:
            config = load_launch_config(str(Path(config_file).expanduser().resolve())) if config_file else {}
            generated = scene_config().generate(config.get('scenes'), directory,
                                                get_package_share_directory('my_robot_world'), catalog_path)
        except Exception:
            shutil.rmtree(directory, ignore_errors=True)
            raise
        context.launch_configurations['_piper_scene_runtime'] = directory
        context.launch_configurations['scene_file'] = str(generated)

    # Share the perception output when both layers run; otherwise let the
    # standalone planner use its own configured object source.
    objects_topic = argument('objects_topic')
    if with_perception and not objects_topic:
        perception_config = yaml.safe_load(Path(argument('perception_config')).read_text())
        objects_topic = perception_config['depth_localizer']['ros__parameters']['objects_topic']
    layers = []
    runtime = context.launch_configurations.get('_piper_scene_runtime')
    if runtime:
        layers.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: shutil.rmtree(runtime, ignore_errors=True))))
    if IfCondition(LaunchConfiguration('dry_run')).evaluate(context):
        planner = yaml.safe_load(Path(argument('planner_config')).read_text())['task_planner']['ros__parameters']
        planner = placement_config().resolve(planner, argument('place_position_modes'), argument('place_positions'))
        execution = yaml.safe_load(Path(argument('manipulation_config')).read_text())['pick_place_executor']['ros__parameters']
        frame = argument('reference_frame') or execution.get('reference_frame', 'base_link')
        return layers + [LogInfo(msg=line) for line in placement_config().describe(
            planner, frame, scene_config().load_catalog(catalog_path))]
    layers.append(control_layer(bringup, argument))
    if with_motion:
        layers.append(motion_layer(bringup, argument, objects_topic))
    if with_perception:
        layers.append(perception_layer(bringup, argument))
    if with_rviz:
        layers.append(rviz_layer())
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
        config_argument('dry_run', default_value='false', choices=['true', 'false'], description='Validate and describe placement without starting nodes'),
        config_argument('place_position_modes', default_value='', description='X/Y/Z relative or absolute modes; empty uses planner YAML'),
        config_argument('place_positions', default_value='', description='Mapping of class IDs to center coordinates or offsets'),
        config_argument('headless', default_value='false', choices=['true', 'false'],
                              description='Run without the MuJoCo window'),
        config_argument('with_moveit', default_value='true', choices=['true', 'false'],
                              description='Start the MoveIt motion planning server'),
        config_argument('with_rviz', default_value='false', choices=['true', 'false'],
                              description='Start the MoveIt RViz window'),
        config_argument('with_executor', default_value=LaunchConfiguration('with_moveit'),
                              choices=['true', 'false'], description='Legacy default for with_motion'),
        config_argument('with_motion', default_value=LaunchConfiguration('with_executor'),
                              choices=['true', 'false'], description='Start executor and task planner'),
        config_argument('with_perception', default_value=LaunchConfiguration('with_motion'),
                              choices=['true', 'false'], description='Start color detection and depth localization'),
        config_argument('with_camera', default_value=LaunchConfiguration('with_perception'),
                              choices=['true', 'false'], description='Publish MuJoCo RGB, depth and camera info'),
        config_argument('sim_speed_factor', default_value='1.0',
                              description='Positive simulation speed relative to wall time'),
        config_argument('scene_file', default_value=str(world / 'basic/preview.xml'),
                              description='World scene including ground, objects and robot placeholder'),
        config_argument('robot_model', default_value=str(robot / 'mjcf/control.xml'),
                              description='Robot MJCF including ros2_control actuators'),
        config_argument('camera_config', default_value=str(world / 'config/camera.yaml'),
                              description='MuJoCo wrist camera configuration'),
        config_argument('controllers_file', default_value=str(robot / 'config/ros2_controllers.yaml'),
                              description='ros2_control controllers and tolerances'),
        config_argument('moveit_controllers_file',
                              default_value=str(moveit / 'config/moveit_controllers.yaml'),
                              description='MoveIt trajectory controller mapping'),
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
