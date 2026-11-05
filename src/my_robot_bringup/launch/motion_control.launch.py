"""Start the pick/place executor and sorting task planner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


EXECUTION_ARGUMENTS = {
    'arm_group': (str, 'MoveIt arm group'),
    'gripper_group': (str, 'MoveIt gripper group'),
    'reference_frame': (str, 'Execution reference frame'),
    'end_effector_link': (str, 'MoveIt tool link'),
    'velocity_scaling': (float, 'Motion velocity scaling'),
    'acceleration_scaling': (float, 'Motion acceleration scaling'),
    'planning_time': (float, 'Planning timeout in seconds'),
    'cartesian_step': (float, 'Cartesian interpolation step in metres'),
}


def launch_setup(context):
    from moveit_configs_utils import MoveItConfigsBuilder

    def argument(name):
        return LaunchConfiguration(name).perform(context)

    pipeline = argument('planning_pipeline')
    moveit = (MoveItConfigsBuilder('piper', package_name='my_robot_moveit_pkg')
              .robot_description(file_path='config/piper.urdf.xacro')
              .trajectory_execution(file_path=argument('moveit_controllers_file'))
              .planning_pipelines(default_planning_pipeline=pipeline, pipelines=[pipeline])
              .to_moveit_configs().to_dict())
    description = argument('robot_description')
    if description:
        moveit['robot_description'] = ParameterValue(description, value_type=str)
    moveit['use_sim_time'] = True

    execution = yaml.safe_load(Path(argument('manipulation_config')).read_text())['pick_place_executor']['ros__parameters']
    planner = yaml.safe_load(Path(argument('planner_config')).read_text())['task_planner']['ros__parameters']
    execution = dict(execution)
    planner = dict(planner)
    for name, (value_type, _) in EXECUTION_ARGUMENTS.items():
        if argument(name):
            execution[name] = value_type(argument(name))

    # Resolve shared interfaces once so both nodes always use the same names.
    pick = argument('pick_place_action') or argument('action_name') or execution.get('action_name', '/manipulation/pick_place')
    if argument('pick_place_action') and argument('action_name') and argument('pick_place_action') != argument('action_name'):
        raise ValueError('pick_place_action and legacy action_name disagree')
    named = argument('move_to_named_action') or execution.get('move_to_named_action', '/manipulation/move_to_named')
    execution.update(action_name=pick, move_to_named_action=named)
    planner.update(pick_place_action=pick, move_to_named_action=named, use_sim_time=True)
    for source, target in (('sort_action', 'action_name'), ('objects_topic', 'objects_topic'),
                           ('observation_target', 'observation_target')):
        if argument(source):
            planner[target] = argument(source)
    # The planner and executor must refer to the same motion geometry.
    planner['reference_frame'] = execution.get('reference_frame', 'base_link')
    planner['end_effector_frame'] = execution.get('end_effector_link', 'link6')
    planner['tool_to_tcp'] = execution.get('tool_to_tcp', [0.0, 0.0, 0.132])
    return [
        Node(package='my_robot_commander', executable='pick_place_executor',
             parameters=[{**moveit, **execution}], output='screen', on_exit=Shutdown()),
        Node(package='my_robot_planner', executable='task_planner',
             parameters=[planner], output='screen', on_exit=Shutdown()),
    ]


def generate_launch_description():
    moveit = Path(get_package_share_directory('my_robot_moveit_pkg'))
    commander = Path(get_package_share_directory('my_robot_commander'))
    planner = Path(get_package_share_directory('my_robot_planner'))
    arguments = [
        DeclareLaunchArgument('robot_description', default_value='', description='Expanded robot description'),
        DeclareLaunchArgument('planning_pipeline', default_value='ompl', description='MoveIt planning pipeline'),
        DeclareLaunchArgument('moveit_controllers_file', default_value=str(moveit / 'config/moveit_controllers.yaml'),
                              description='MoveIt trajectory controller mapping'),
        DeclareLaunchArgument('manipulation_config', default_value=str(commander / 'config/manipulation.yaml'),
                              description='Pick/place executor configuration'),
        DeclareLaunchArgument('planner_config', default_value=str(planner / 'config/task_planner.yaml'),
                              description='Sorting task planner configuration'),
    ]
    for name, description in {
        'pick_place_action': 'Pick/place action name',
        'action_name': 'Legacy alias for pick_place_action',
        'move_to_named_action': 'Named motion action name',
        'sort_action': 'Sorting action name',
        'objects_topic': 'Detected objects input topic',
        'observation_target': 'Observation named pose',
    }.items():
        arguments.append(DeclareLaunchArgument(name, default_value='', description=description + '; empty uses YAML'))
    arguments.extend(DeclareLaunchArgument(name, default_value='', description=description + '; empty uses YAML')
                     for name, (_, description) in EXECUTION_ARGUMENTS.items())
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
