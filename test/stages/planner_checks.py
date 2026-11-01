"""任务策略、观测时效、失败传播与取消检查。"""

from action_msgs.msg import GoalStatus
from my_robot_interface.action import SortObjects

from test.support.planner_environment import object_msg
from test.support.planner_fixture import PlannerFixture


class PlannerChecks(PlannerFixture):
    """模拟执行端的接口回归；每个用例重置观测和 Action 行为。"""

    def test_color_order_and_defaults(self):
        result = self.wait(self.send(colors=[3, 1, 2]))
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.result.success)
        self.assertEqual(self.colors_called(), [3, 1, 2])
        self.assertEqual(self.feedback[-1].completed_count, 3)
        self.assertEqual(self.environment.calls[1].place_pose.pose.position.x, 0.60)
        self.environment.calls.clear()
        result = self.wait(self.send(strategy='', colors=[]))
        self.assertTrue(result.result.success)
        self.assertEqual(self.colors_called(), [1, 2, 3])

    def test_nearest_rechecks_current_tcp(self):
        result = self.wait(self.send('nearest'))
        self.assertTrue(result.result.success)
        # 初始距离为红、蓝、绿；红完成后 TCP 到 x=0.60，下一目标应变为绿。
        self.assertEqual(self.colors_called(), [1, 2, 3])
        self.assertEqual(self.environment.observation_calls, ['pose_1'] * 3)

    def test_observation_failure_stops_before_pick(self):
        self.environment.observation_fail = True
        result = self.wait(self.send())
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.environment.calls)
        self.assertIn('Observation motion failed', result.result.message)

    def test_observation_cancel_forwarding(self):
        self.environment.observation_hold = True
        future = self.send()
        self.spin_until(lambda: len(self.environment.observation_calls) == 1)
        self.wait(self.handle.cancel_goal_async())
        self.assertEqual(self.wait(future).status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(self.environment.calls)

    def test_old_images_after_observation_are_not_used(self):
        self.environment.observation_hold = True
        future = self.send()
        self.spin_until(lambda: len(self.environment.observation_calls) == 1)
        self.environment.publish_enabled = False
        self.environment.release.set()
        self.assertEqual(self.wait(future).status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.environment.calls)

    def test_refreshes_next_object_pose(self):
        self.environment.hold = True
        future = self.send(colors=[1, 2])
        self.spin_until(lambda: len(self.environment.calls) == 1)
        self.environment.scene = [object_msg(1, 0.21), object_msg(2, 0.45), object_msg(3, 0.30)]
        self.settle()
        self.environment.hold = False
        self.environment.release.set()
        self.assertTrue(self.wait(future).result.success)
        self.assertAlmostEqual(self.environment.calls[1].object_pose.pose.position.x, 0.45)

    def test_busy_rejection_and_cancel_forwarding(self):
        self.environment.hold = True
        future = self.send()
        self.spin_until(lambda: len(self.environment.calls) == 1)
        other = self.wait(self.client.send_goal_async(SortObjects.Goal()))
        self.assertFalse(other.accepted)
        self.assertTrue(self.wait(self.handle.cancel_goal_async()).goals_canceling)
        self.assertEqual(self.wait(future).status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(self.environment.cancel_seen.is_set())
        self.assertEqual(len(self.environment.calls), 1)

    def test_failure_stops_remaining_objects(self):
        self.environment.fail_ids = {'red_cube'}
        result = self.wait(self.send(colors=[3, 1, 2]))
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)
        self.assertEqual(self.colors_called(), [3, 1])
        self.assertEqual(self.feedback[-1].completed_count, 1)

    def test_invalid_policy_rejected(self):
        for strategy, colors in [('unknown', [1]), ('color_order', [1, 1]),
                                 ('nearest', [0]), ('nearest', [4])]:
            goal = SortObjects.Goal(strategy=strategy, color_order=colors)
            self.assertFalse(self.wait(self.client.send_goal_async(goal)).accepted)
        self.assertFalse(self.environment.calls)

    def test_missing_color_times_out(self):
        self.environment.scene = [object_msg(1, 0.21)]
        self.settle()
        self.assertEqual(self.wait(self.send()).status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.environment.calls)

    def test_stale_observation_rejected(self):
        self.environment.stamp_offset = -10.0
        self.settle()
        self.assertEqual(self.wait(self.send()).status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.environment.calls)

    def test_lost_next_color_does_not_skip_order(self):
        self.environment.hold = True
        future = self.send()
        self.spin_until(lambda: len(self.environment.calls) == 1)
        self.environment.scene = [object_msg(3, 0.30)]
        self.settle()
        self.environment.hold = False
        self.environment.release.set()
        self.assertEqual(self.wait(future).status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(self.colors_called(), [1])

    def test_object_frame_transform(self):
        self.environment.frame = 'check_offset'
        self.settle()
        self.assertTrue(self.wait(self.send(colors=[1])).result.success)
        target = self.environment.calls[0].object_pose
        self.assertEqual(target.header.frame_id, 'base_link')
        self.assertAlmostEqual(target.pose.position.x, 0.31)
        self.assertAlmostEqual(target.pose.position.y, 0.20)
        self.assertAlmostEqual(target.pose.position.z, 0.305)

    def test_duplicate_ids_abort(self):
        self.environment.scene[1].object_id = 'red_cube'
        self.settle()
        self.assertEqual(self.wait(self.send()).status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.environment.calls)

    def test_pick_rejection_aborts(self):
        self.environment.reject_ids = {'red_cube'}
        result = self.wait(self.send())
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertIn('rejected', result.result.message)
        self.assertFalse(self.environment.calls)

