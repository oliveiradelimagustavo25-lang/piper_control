"""空载流程、物理抓放和带载取消恢复；每个阶段可单独准备物块。"""

import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import SpawnEntity
from my_robot_interface.action import PickPlace
from std_srvs.srv import Trigger

from test.support.runtime import ROOT


def positions(profile):
    return (0.35, 0.08, 0.10) if profile == 'baseline' else (0.26, 0.04, 0.025)


def check_air(session):
    x, _, air_z = positions(session.args.profile)
    passed = session.run_action('free_space_sequence',
        session.goal('empty_space', [x, 0.0, air_z], [x, -0.03, air_z]))
    session.remove_objects('empty_space')
    return passed


def spawn_cube(session):
    x, y, _ = positions(session.args.profile)
    assert session.until(session.state_client.service_is_ready, 10), 'Gazebo state plugin unavailable'
    assert session.until(session.spawn_client.service_is_ready, 10), 'Spawn service unavailable'
    request = SpawnEntity.Request()
    request.name = 'validation_cube'
    cube = ET.parse(ROOT / 'src/my_robot_model/blue_cube/model.sdf')
    size = session.args.cube_size
    mass = session.args.cube_mass if session.args.cube_mass is not None else 0.001 * (size / 0.01)**3
    for box in cube.getroot().findall('.//box/size'):
        box.text = f'{size} {size} {size}'
    cube.getroot().find('.//inertial/mass').text = str(mass)
    for axis in ['ixx', 'iyy', 'izz']:
        cube.getroot().find('.//inertia/' + axis).text = str(mass * size * size / 6)
    collision = cube.getroot().find('.//collision')
    ode = ET.SubElement(ET.SubElement(ET.SubElement(collision, 'surface'), 'friction'), 'ode')
    ET.SubElement(ode, 'mu').text = str(session.args.friction)
    ET.SubElement(ode, 'mu2').text = str(session.args.friction)
    sensor = ET.SubElement(cube.getroot().find('.//link'), 'sensor',
                           name='cube_contact', type='contact')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '50'
    ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = collision.get('name')
    bumper = ET.SubElement(sensor, 'plugin', name='cube_bumper', filename='libgazebo_ros_bumper.so')
    ros = ET.SubElement(bumper, 'ros')
    ET.SubElement(ros, 'namespace').text = '/validation'
    ET.SubElement(ros, 'remapping').text = 'bumper_states:=cube_contacts'
    request.xml = ET.tostring(cube.getroot(), encoding='unicode')
    (session.output / 'cube.sdf').write_text(request.xml)
    session.report['cube_mass_kg'] = mass
    position = request.initial_pose.position
    position.x, position.y, position.z = x, y, size / 2.0 + 0.001
    request.initial_pose.orientation.w = 1.0
    assert session.wait(session.spawn_client.call_async(request)).success, 'Cube spawn failed'
    session.waiter.settle(0.5)
    return request.name


def check_physical(session, name):
    x, y, _ = positions(session.args.profile)
    pick = session.state(name)
    passed = session.run_action('physical_pick_place',
        session.goal(name, pick, [x, -y, session.args.cube_size / 2.0]), track=True)
    return (passed and session.report['physical']['passed']
            and session.report['scene'].get('attached_during_transport', False)
            and session.report['scene'].get('released_into_world', False))


def check_recovery(session, name):
    x, y, _ = positions(session.args.profile)
    pick = session.state(name)
    # 在搬运阶段取消，检查负载保留、新抓取拒绝和显式释放。
    transfer_seen = []
    target = session.goal(name, pick, [x, y, session.args.cube_size / 2.0])
    handle = session.wait(session.client.send_goal_async(target,
        feedback_callback=lambda msg: transfer_seen.append(msg.feedback.stage)))
    assert handle.accepted
    assert session.until(lambda: PickPlace.Feedback.TRANSFERRING in transfer_seen, 45)
    session.until(lambda: False, 0.2)
    assert session.wait(handle.cancel_goal_async()).goals_canceling
    result = session.wait(handle.get_result_async(), 5)
    scene = session.planning_scene()
    recovery = {'canceled': result.status == GoalStatus.STATUS_CANCELED,
        'load_retained': any(o.object.id == name for o in scene.robot_state.attached_collision_objects)}
    competing = session.goal('must_reject', pick, pick)
    recovery['new_pick_rejected'] = not session.wait(session.client.send_goal_async(competing)).accepted
    recovery['explicit_release'] = session.wait(session.release_held.call_async(Trigger.Request()), 20).success
    scene = session.planning_scene()
    recovery['detached_after_release'] = not any(
        o.object.id == name for o in scene.robot_state.attached_collision_objects)
    session.report['recovery'] = recovery
    return all(recovery.values())
