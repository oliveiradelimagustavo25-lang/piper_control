"""目标合法性、时间戳、忙状态与取消协议检查。"""

import json
from action_msgs.msg import GoalStatus


def check_contract(session):
    # 使用无效目标、未来时间戳和缺失 TF 检查协议，不要求实际运动。
    contract = {}
    invalid = session.goal('invalid', [0.26, 0.04, 0.005], [0.26, -0.04, 0.005])
    invalid.object_dimensions.x = -1.0
    contract['invalid_goal_rejected'] = not session.wait(session.client.send_goal_async(invalid)).accepted
    future = session.goal('future', [0.26, 0.04, 0.005], [0.26, -0.04, 0.005])
    future.object_pose.header.stamp.sec = 360000
    handle = session.wait(session.client.send_goal_async(future))
    assert handle.accepted
    result = session.wait(handle.get_result_async())
    contract['future_goal_aborted'] = result.status == GoalStatus.STATUS_ABORTED and not result.result.success
    blocked = session.goal('blocked', [0.26, 0.04, 0.005], [0.26, -0.04, 0.005])
    blocked.object_pose.header.frame_id = 'validation_missing_frame'
    handle = session.wait(session.client.send_goal_async(blocked))
    assert handle.accepted
    competing = session.goal('competing', [0.26, 0.04, 0.005], [0.26, -0.04, 0.005])
    contract['busy_goal_rejected'] = not session.wait(session.client.send_goal_async(competing)).accepted
    canceled = session.wait(handle.cancel_goal_async())
    result = session.wait(handle.get_result_async())
    contract['cancel_completed'] = bool(canceled.goals_canceling) and result.status == GoalStatus.STATUS_CANCELED
    session.report['contract'] = contract
    print('Action contract: ' + json.dumps(contract), flush=True)
    return all(session.report['contract'].values())
