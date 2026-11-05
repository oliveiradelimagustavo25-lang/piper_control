"""生成测试专用配置和模型，始终写入本次 log 目录，不覆盖 src。"""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

from test.support.runtime import ROOT


def prepare_executor(args, out):
    """保留原执行器测试的物理配置与诊断参数。"""
    world = ET.parse(ROOT / 'src/my_robot_world/basic/preview.world')
    # 抓取测试使用近景，方便 GUI 中观察指尖和物块。
    world.getroot().find('world/gui/camera/pose').text = '0.60 -0.40 0.24 0 0.33 2.275'
    plugin = ET.SubElement(world.getroot().find('world'), 'plugin',
                           name='validation_state', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/validation'
    ET.SubElement(plugin, 'update_rate').text = '30.0'
    if args.contact_layer is not None:
        physics = world.getroot().find('world/physics')
        ode = physics.find('ode')
        if ode is None:
            ode = ET.SubElement(physics, 'ode')
        constraints = ode.find('constraints')
        if constraints is None:
            constraints = ET.SubElement(ode, 'constraints')
        ET.SubElement(constraints, 'contact_surface_layer').text = str(args.contact_layer)
    world.write(out / 'world.world', encoding='unicode')
    config = yaml.safe_load((ROOT / 'src/my_robot_commander/config/manipulation.yaml').read_text())
    config['pick_place_executor']['ros__parameters']['objects_topic'] = '/validation/objects'
    if args.profile == 'baseline':
        config['pick_place_executor']['ros__parameters'].update(
            tool_to_tcp=[0.0, 0.0, 0.18], grasp_rpy=[math.pi, 0.0, 0.0],
            approach_distance=0.06, lift_distance=0.10, transport_height=0.20, retreat_distance=0.06)
    if args.profile == 'ground':
        config['pick_place_executor']['ros__parameters'].update(
            tool_to_tcp=[0.0, 0.0, 0.132], grasp_rpy=[0.0, math.pi, 0.0],
            approach_distance=0.04, lift_distance=0.06, transport_height=0.08, retreat_distance=0.04)
    if args.tcp_z is not None:
        config['pick_place_executor']['ros__parameters']['tool_to_tcp'][2] = args.tcp_z
    if args.compression is not None:
        config['pick_place_executor']['ros__parameters']['gripper_compression'] = args.compression
    (out / 'manipulation.yaml').write_text(yaml.safe_dump(config))

    # 摩擦参数只应用于展开后的测试模型，保持源 URDF 不变。
    model = ROOT / 'src/my_robot_description/urdf/piper_control.urdf.xacro'
    robot_xml = subprocess.check_output(['xacro', str(model)], text=True, timeout=30)
    robot = ET.fromstring(robot_xml)
    controller_params = robot.find("gazebo/plugin[@name='gazebo_ros2_control']/parameters")
    controller_config = yaml.safe_load(Path(controller_params.text).read_text())
    (out / 'controllers.yaml').write_text(yaml.safe_dump(controller_config))
    controller_params.text = str(out / 'controllers.yaml')
    simulation_control = {
        joint.get('name'): joint.find('command_interface').get('name')
        for joint in robot.findall('ros2_control/joint')}
    for link in ['link7', 'link8']:
        gazebo = robot.find(f"gazebo[@reference='{link}']")
        if gazebo is None:
            gazebo = ET.SubElement(robot, 'gazebo', reference=link)
        for name in ['mu1', 'mu2']:
            coefficient = gazebo.find(name)
            if coefficient is None:
                coefficient = ET.SubElement(gazebo, name)
            coefficient.text = str(args.friction)
    robot_xml = ET.tostring(robot, encoding='unicode')
    (out / 'robot.urdf').write_text(robot_xml)

    return robot_xml, simulation_control


def prepare_sorting(output):
    """三色回归使用间隔更大的放置区，与手动 launch 默认场景区分。"""
    config = yaml.safe_load((ROOT / 'src/my_robot_planner/config/task_planner.yaml').read_text())
    for color, y in [('red', -0.06), ('green', 0.0), ('blue', 0.06)]:
        config['task_planner']['ros__parameters'][color + '.place_position'] = [0.26, y, 0.005]
    (output / 'task_planner.yaml').write_text(yaml.safe_dump(config))
    world = ET.parse(ROOT / 'src/my_robot_world/basic/preview.world')
    world.getroot().find('world/gui/camera/pose').text = '0.60 -0.40 0.30 0 0.40 2.275'
    plugin = ET.SubElement(world.getroot().find('world'), 'plugin',
                           name='sorting_state', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/sorting_test'
    world.write(output / 'world.world', encoding='unicode')
