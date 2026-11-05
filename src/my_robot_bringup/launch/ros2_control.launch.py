"""Start the MuJoCo world, robot controllers and optionally MoveIt."""

import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.actions import SetLaunchConfiguration, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

from my_robot_world.scenes.scene import export_scene


def camera_plugin_file(runtime, argument):
    """Match the camera publisher topics to the perception configuration."""
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
    path = Path(runtime.name) / 'camera_plugin.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'mujoco_plugins': {'camera_publisher': plugin}}}}))
    return str(path)


def prepare_model(argument, robot, world):
    """Compose the complete world or the robot-only stage test model."""
    speed = float(argument('sim_speed_factor'))
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('sim_speed_factor must be a finite positive number')
    robot_only = argument('robot_only') == 'true'
    camera = argument('with_camera')
    with_camera = camera == 'true' or (camera == 'auto' and not robot_only)
    if robot_only and with_camera:
        raise ValueError('robot_only:=true requires with_camera:=false or auto')

    runtime = tempfile.TemporaryDirectory(prefix='piper_world_') if not robot_only else None
    try:
        model_path = Path(argument('model_path'))
        plugins_file = argument('plugins_file')
        if runtime:
            model_path = Path(runtime.name) / 'world.xml'
            export_scene(model_path, world, robot, robot_model=argument('model_path'),
                         scene_file=argument('scene_file'), camera_config=argument('camera_config'))
            if with_camera and not plugins_file:
                plugins_file = camera_plugin_file(runtime, argument)
        description = xacro.process_file(
            str(robot / 'urdf/piper_control.urdf.xacro'),
            mappings={'headless': argument('headless'), 'sim_speed_factor': argument('sim_speed_factor'),
                      'model_path': str(model_path)},
        ).toxml()
        return description, plugins_file, runtime, model_path
    except Exception:
        if runtime:
            runtime.cleanup()
        raise


def control_nodes(description, controllers, plugins_file):
    parameters = [controllers]
    if plugins_file:
        parameters.append(plugins_file)
    parameters.append({'use_sim_time': True})
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(description, value_type=str),
                          'use_sim_time': True}], output='screen'),
        Node(package='mujoco_ros2_control', executable='ros2_control_node',
             parameters=parameters, output='screen',
             remappings=[('~/robot_description', '/robot_description')], on_exit=Shutdown()),
    ]
    for name in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller'):
        nodes.append(Node(package='controller_manager', executable='spawner', output='screen',
                          arguments=[name, '--controller-manager', '/controller_manager',
                                     '--param-file', controllers]))
    return nodes


def moveit_node(context, description):
    """MoveIt shares the same expanded robot description as the controllers."""
    from moveit_configs_utils import MoveItConfigsBuilder

    pipeline = LaunchConfiguration('planning_pipeline').perform(context)
    config = (MoveItConfigsBuilder('piper', package_name='my_robot_moveit_pkg')
              .robot_description(file_path='config/piper.urdf.xacro')
              .trajectory_execution(file_path=LaunchConfiguration('moveit_controllers_file').perform(context))
              .planning_pipelines(default_planning_pipeline=pipeline, pipelines=[pipeline])
              .to_moveit_configs().to_dict())
    config.update(robot_description=ParameterValue(description, value_type=str), use_sim_time=True)
    return Node(package='moveit_ros_move_group', executable='move_group',
                parameters=[config], output='screen', on_exit=Shutdown())


def launch_setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    robot = Path(get_package_share_directory('my_robot_description'))
    world = Path(get_package_share_directory('my_robot_world'))
    description = argument('robot_description')
    plugins_file = argument('plugins_file')
    runtime = None
    model_path = None
    if not description:
        description, plugins_file, runtime, model_path = prepare_model(argument, robot, world)

    # The parent motion layer receives this exact expanded description.
    actions = [SetLaunchConfiguration('_piper_control_description', description)]
    if runtime:
        actions.extend([
            RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: runtime.cleanup())),
            LogInfo(msg=f'MuJoCo complete world: {model_path}'),
        ])
    actions.extend(control_nodes(description, argument('controllers_file'), plugins_file))
    if IfCondition(LaunchConfiguration('with_moveit')).evaluate(context):
        actions.append(moveit_node(context, description))
    return actions


def generate_launch_description():
    robot = Path(get_package_share_directory('my_robot_description'))
    world = Path(get_package_share_directory('my_robot_world'))
    moveit = Path(get_package_share_directory('my_robot_moveit_pkg'))
    perception = Path(get_package_share_directory('my_robot_perception'))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Run without the MuJoCo window'),
        DeclareLaunchArgument('robot_only', default_value='false', choices=['true', 'false'],
                              description='Load only the robot for the independent control test'),
        DeclareLaunchArgument('with_camera', default_value='auto', choices=['auto', 'true', 'false'],
                              description='Publish world camera data; auto disables it in robot-only mode'),
        DeclareLaunchArgument('sim_speed_factor', default_value='1.0',
                              description='Positive simulation speed relative to wall time'),
        DeclareLaunchArgument('model_path', default_value=str(robot / 'mjcf/control.xml'),
                              description='Robot MJCF with ros2_control actuators'),
        DeclareLaunchArgument('scene_file', default_value=str(world / 'basic/preview.xml'),
                              description='Complete world scene'),
        DeclareLaunchArgument('camera_config', default_value=str(world / 'config/camera.yaml'),
                              description='MuJoCo wrist camera configuration'),
        DeclareLaunchArgument('perception_config', default_value=str(perception / 'config/perception.yaml'),
                              description='Default ROS camera topic names'),
        DeclareLaunchArgument('image_topic', default_value='', description='RGB topic; empty uses perception_config'),
        DeclareLaunchArgument('depth_topic', default_value='', description='Depth topic; empty uses perception_config'),
        DeclareLaunchArgument('camera_info_topic', default_value='', description='CameraInfo topic; empty uses perception_config'),
        DeclareLaunchArgument('controllers_file', default_value=str(robot / 'config/ros2_controllers.yaml'),
                              description='ros2_control controller configuration'),
        DeclareLaunchArgument('with_moveit', default_value='true', choices=['true', 'false'],
                              description='Start MoveIt with the control system'),
        DeclareLaunchArgument('planning_pipeline', default_value='ompl', description='MoveIt planning pipeline'),
        DeclareLaunchArgument('moveit_controllers_file', default_value=str(moveit / 'config/moveit_controllers.yaml'),
                              description='MoveIt trajectory controller mapping'),
        DeclareLaunchArgument('plugins_file', default_value='', description='Optional native plugin configuration'),
        DeclareLaunchArgument('robot_description', default_value='',
                              description='Optional pre-expanded control description'),
        OpaqueFunction(function=launch_setup),
    ])
