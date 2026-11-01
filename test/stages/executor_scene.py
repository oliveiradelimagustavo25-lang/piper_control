"""场景同步、运动中取消与障碍阻挡检查；会驱动仿真机械臂。"""

import time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from my_robot_interface.action import MoveToNamed
from my_robot_interface.msg import DetectedObject, DetectedObjectArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


def check_scene(session):
    # 同步其他观测物体，并在机械臂实际运动后验证取消。
    assert session.until(session.named_client.server_is_ready, 20)
    assert session.until(session.get_scene.service_is_ready, 10)
    obj = DetectedObject(object_id='observed_obstacle', color=1)
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = 0.8, 0.8, 0.025
    obj.pose.orientation.w = 1.0
    obj.dimensions.x = obj.dimensions.y = obj.dimensions.z = 0.05
    observation = DetectedObjectArray(objects=[obj])
    observation.header.frame_id = 'base_link'
    assert session.until(lambda: session.objects_pub.get_subscription_count() > 0, 5)
    session.objects_pub.publish(observation)
    session.until(lambda: False, 0.3)
    before = dict(session.joint_positions)
    handle = session.wait(session.named_client.send_goal_async(MoveToNamed.Goal(target='pose_1')))
    assert handle.accepted
    assert session.until(lambda: any(abs(session.joint_positions.get(j, v) - v) > 0.03
        for j, v in before.items() if j in ['joint1', 'joint2', 'joint3']), 15), 'Arm did not start moving'
    start_cancel = time.monotonic()
    assert session.wait(handle.cancel_goal_async()).goals_canceling
    result = session.wait(handle.get_result_async(), 5)
    cancel_latency = time.monotonic() - start_cancel
    scene = session.planning_scene()
    names = {o.id for o in scene.world.collision_objects}
    session.report['scene'] = {'ground_present': 'environment/ground' in names,
                      'observations_imported': obj.object_id in names}
    session.until(lambda: False, 0.3)
    stopped = dict(session.joint_positions)
    session.until(lambda: False, 0.3)
    session.report['cancel'] = {'latency_s': cancel_latency,
        'canceled': result.status == GoalStatus.STATUS_CANCELED,
        'settled': all(abs(session.joint_positions.get(j, v) - v) < 0.02 for j, v in stopped.items())}
    canceled = session.report['cancel']
    canceled['passed'] = canceled['canceled'] and canceled['settled'] and canceled['latency_s'] < 2.0
    # 覆盖预抓取位置的碰撞盒必须阻止执行。
    blocker = CollisionObject(id='validation_blocker', operation=CollisionObject.ADD)
    blocker.header.frame_id = 'base_link'
    blocker.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.12, 0.12, 0.12])]
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x, pose.position.y, pose.position.z = 0.26, 0.04, 0.19
    blocker.primitive_poses = [pose]
    session.scene_objects([blocker])
    target = session.goal('collision_probe', [0.26, 0.04, 0.015], [0.26, -0.04, 0.015])
    handle = session.wait(session.client.send_goal_async(target))
    assert handle.accepted
    result = session.wait(handle.get_result_async(), 30)
    session.report['scene']['obstacle_blocks_pick'] = result.status == GoalStatus.STATUS_ABORTED
    session.remove_objects('validation_blocker', 'collision_probe')
    session.remove_objects('observed_obstacle')
    return all(session.report['scene'].values()) and session.report['cancel']['passed']
