#!/usr/bin/env python3
"""隔离域中的调度逻辑测试；使用合成观测和模拟执行器，不证明物理抓取。"""
import argparse
import copy
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from my_robot_interface.action import MoveToNamed, PickPlace, SortObjects
from my_robot_interface.msg import DetectedObject, DetectedObjectArray
from std_srvs.srv import Trigger
import yaml

ROOT = Path(__file__).resolve().parents[2]


def obj(name, color, x, y, z=0.005):
    o = DetectedObject()
    o.object_id, o.color = name, color
    o.pose.position.x, o.pose.position.y, o.pose.position.z = x, y, z
    o.pose.orientation.w = 1.0
    o.dimensions.x = o.dimensions.y = o.dimensions.z = 0.01
    return o


class FakeCell(Node):
    def __init__(self):
        super().__init__('adaptive_planner_probe')
        self.lock = threading.RLock()
        self.scene, self.calls = [], []
        self.behavior = lambda goal, attempt: 'success'
        self.recoveries = 0
        self.recovery_fails = False
        self.frame_count = 0
        self.group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(DetectedObjectArray, '/verification/objects', 10)
        self.create_timer(0.03, self.publish, callback_group=self.group)
        self.named = ActionServer(self, MoveToNamed, '/manipulation/move_to_named', self.move_named,
                                  callback_group=self.group, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.pick = ActionServer(self, PickPlace, '/manipulation/pick_place', self.execute,
                                 callback_group=self.group, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_service(Trigger, '/manipulation/release_held_object', self.recover, callback_group=self.group)
        self.client = ActionClient(self, SortObjects, '/sorting/sort_objects', callback_group=self.group)
        self.pending = None

    def publish(self):
        msg = DetectedObjectArray()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            self.frame_count += 1
            msg.objects = copy.deepcopy(self.scene)
        self.publisher.publish(msg)

    def move_named(self, handle):
        handle.succeed()
        return MoveToNamed.Result(success=True)

    def move_object(self, goal):
        for o in self.scene:
            if o.object_id == goal.object_id:
                o.pose = copy.deepcopy(goal.place_pose.pose)

    def execute(self, handle):
        goal = handle.request
        with self.lock:
            self.calls.append(copy.deepcopy(goal))
            attempt = sum(g.object_id == goal.object_id for g in self.calls)
            outcome = self.behavior(goal, attempt)
            if outcome == 'holding':
                self.pending = copy.deepcopy(goal)
            elif outcome in ('success', 'success_new_id'):
                self.move_object(goal)
                if outcome == 'success_new_id':
                    next(o for o in self.scene if o.object_id == goal.object_id).object_id += '_placed'
        time.sleep(0.03)
        result = PickPlace.Result()
        result.success = outcome in ('success', 'success_new_id', 'empty')
        result.released = result.success
        result.holding = outcome == 'holding'
        result.ready_for_next_pick = not result.holding
        result.unreachable_placement = outcome == 'unreachable'
        result.message = outcome
        if result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def recover(self, _, response):
        with self.lock:
            self.recoveries += 1
            if self.recovery_fails:
                response.success = False
                response.message = 'simulated recovery blocked'
                return response
            if self.pending:
                self.move_object(self.pending)
                self.pending = None
        response.success = True
        return response


def wait(condition, timeout, process=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        if process is not None and process.poll() is not None:
            raise AssertionError(f'planner exited: {process.returncode}')
        time.sleep(0.02)
    raise AssertionError('Probe timed out')


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def run_case(cell, name, scene, behavior=None, overrides=None, expected=None, cancel_when=None, colors=None):
    params = dict(objects_topic='/verification/objects', observation_target='',
                  end_effector_frame='base_link', observation_settle_time=0.02,
                  idle_timeout=0.35, verification_timeout=0.25, slot_tolerance=0.01,
                  max_observation_age=1.0)
    params.update({'red.place_slots': [0.26, -0.06, 0.005, 0.34, -0.06, 0.005],
                   'green.place_slots': [0.26, 0.0, 0.005, 0.34, 0.0, 0.005],
                   'blue.place_slots': [0.26, 0.06, 0.005, 0.34, 0.06, 0.005]})
    for key, value in (overrides or {}).items():
        if value is None:
            params.pop(key, None)
        else:
            params[key] = value
    logs = ROOT / 'log/adaptive_planner'
    logs.mkdir(parents=True, exist_ok=True)
    config, log = logs / f'{name}.yaml', logs / f'{name}.log'
    config.write_text(yaml.safe_dump({'task_planner': {'ros__parameters': params}}))
    with cell.lock:
        cell.scene, cell.calls = copy.deepcopy(scene), []
        cell.behavior = behavior or (lambda goal, attempt: 'success')
        cell.pending, cell.recoveries, cell.frame_count = None, 0, 0
    with log.open('w') as output:
        process = subprocess.Popen([str(ROOT / 'install/my_robot_planner/lib/my_robot_planner/task_planner'),
                                    '--ros-args', '--params-file', str(config)],
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait(cell.client.server_is_ready, 10, process)
            goal = SortObjects.Goal()
            goal.color_order = colors or []
            sent = cell.client.send_goal_async(goal)
            wait(sent.done, 5, process)
            handle = sent.result()
            assert handle.accepted, name
            result = handle.get_result_async()
            if cancel_when:
                wait(lambda: cancel_when(cell), 12, process)
                canceled = handle.cancel_goal_async()
                wait(canceled.done, 5, process)
            wait(result.done, 15, process)
            outcome = result.result()
            if cancel_when:
                assert outcome.status == GoalStatus.STATUS_CANCELED, outcome
            elif expected is not None:
                assert outcome.result.success == expected, (name, outcome.result)
            calls = copy.deepcopy(cell.calls)
            print(f'PASS {name}: {[g.object_id for g in calls]} | {outcome.result.message}', flush=True)
            return calls, outcome.result
        finally:
            stop(process)
            time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-domain-id', type=int, default=186)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    existing = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True, text=True,
                              timeout=10, check=True)
    if existing.stdout.strip():
        raise RuntimeError('Test domain already occupied; select another domain')
    rclpy.init()
    cell = FakeCell()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(cell)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        red = obj('red_1', 1, 0.32, -0.12)
        green = obj('green_1', 2, 0.32, 0.04)
        blue = obj('blue_1', 3, 0.32, 0.12)
        calls, _ = run_case(cell, 'multi', [red, obj('red_2', 1, 0.32, -0.04), green, blue], expected=True)
        assert len(calls) == 4 and [g.object_id for g in calls[:2]] == ['red_2', 'red_1']
        assert calls[0].place_pose.pose.position != calls[1].place_pose.pose.position
        calls, _ = run_case(cell, 'missing_blue', [red, green], expected=True)
        assert len(calls) == 2
        calls, _ = run_case(cell, 'retry', [red, green],
                            lambda g, n: 'failure' if g.object_id == 'red_1' and n < 3 else 'success', expected=True)
        assert [g.object_id for g in calls] == ['red_1', 'red_1', 'red_1', 'green_1']
        calls, _ = run_case(cell, 'exhausted', [red, green],
                            lambda g, n: 'failure' if g.object_id == 'red_1' else 'success', expected=False)
        assert len(calls) == 4 and calls[-1].object_id == 'green_1'
        calls, _ = run_case(cell, 'unreachable_slot', [red],
                            lambda g,n: 'unreachable' if g.place_pose.pose.position.x == 0.26 else 'success', expected=True)
        assert len(calls)==2 and calls[1].place_pose.pose.position.x==0.34
        calls, _ = run_case(cell, 'empty_grasp', [red], lambda g, n: 'empty' if n == 1 else 'success', expected=True)
        assert len(calls) == 2

        def moved(goal, attempt):
            if attempt == 1:
                cell.scene[0].pose.position.x += 0.008
                return 'failure'
            return 'success'
        calls, _ = run_case(cell, 'moved_retry', [red], moved, expected=True)
        assert len(calls) == 2 and calls[1].object_pose.pose.position.x > calls[0].object_pose.pose.position.x
        calls, _ = run_case(cell, 'recovery', [red], lambda g, n: 'holding', expected=True)
        assert len(calls) == 1 and cell.recoveries == 1
        calls, _ = run_case(cell, 'overlapping_regions', [obj('already_sorted', 1, 0.26, -0.06), red], expected=True)
        assert len(calls) == 1 and calls[0].place_pose.pose.position.x == 0.34
        calls, _ = run_case(cell, 'swapped_colors', [obj('red_1', 1, 0.26, 0.0),
                            obj('green_1', 2, 0.26, -0.06)], expected=True)
        assert len(calls) == 2 and calls[0].place_pose.pose.position.x == 0.34
        calls, _ = run_case(cell, 'roi', [red, blue], overrides={
            'workspace_min': [0.30, -0.15, 0.0], 'workspace_max': [0.40, 0.0, 0.10]}, expected=True)
        assert [g.object_id for g in calls] == ['red_1']

        def reappear(goal, attempt):
            if goal.object_id == 'red_1' and attempt == 1:
                cell.scene = [o for o in cell.scene if o.object_id != 'red_1']
                return 'failure'
            if goal.object_id == 'green_1':
                cell.scene += [copy.deepcopy(red), copy.deepcopy(blue)]
            return 'success'
        calls, _ = run_case(cell, 'reappear_priority', [red, green], reappear, expected=True)
        assert [g.object_id for g in calls] == ['red_1', 'green_1', 'red_1', 'blue_1']
        calls, _ = run_case(cell, 'watch_cancel', [], overrides={'finish_mode': 'watch'},
                            cancel_when=lambda _: cell.frame_count >= 30)
        assert not calls
        calls, _ = run_case(cell, 'full_wait', [red, obj('blocker', 2, 0.26, -0.06)], colors=[1], overrides={
            'red.place_slots': [0.26, -0.06, 0.005]},
            cancel_when=lambda _: cell.frame_count >= 30 and not cell.calls)
        assert not calls
        for policy, expected_order in [('priority', ['red_2', 'red_1', 'red_2']),
                                       ('immediate', ['red_2', 'red_2', 'red_1'])]:
            def shift_failed(goal, attempt):
                if goal.object_id == 'red_2' and attempt == 1:
                    next(o for o in cell.scene if o.object_id == 'red_2').pose.position.x = 0.40
                    return 'failure'
                return 'success'
            calls, _ = run_case(cell, f'retry_{policy}', [red, obj('red_2', 1, 0.32, -0.04)],
                                shift_failed, overrides={'retry_policy': policy}, expected=True)
            assert [g.object_id for g in calls] == expected_order
        cell.recovery_fails = True
        run_case(cell, 'cancel_failed_recovery', [red], lambda g, n: 'holding',
                 cancel_when=lambda _: cell.recoveries >= 1)
        cell.recovery_fails = False
        planner = yaml.safe_load((ROOT/'src/my_robot_planner/config/task_planner.yaml').read_text())['task_planner']['ros__parameters']
        region = {k:v for k,v in planner.items() if k.endswith(('.place_region','.place_position')) or k=='placement_step'}
        region.update({c+'.place_slots':None for c in ['red','green','blue']})
        for counts in [(0,4,0),(3,1,1),(4,0,0),(0,0,4)]:
            objects=[]
            for color,n in enumerate(counts,1):
                for i in range(n):
                    index=len(objects)
                    objects.append(obj(f'c{color}_{i}',color,0.32+0.06*(index//4),[-0.04,0.04,-0.12,0.12][index%4]))
            calls,_=run_case(cell,'region_'+'_'.join(map(str,counts)),objects,overrides=region,expected=True)
            assert len(calls)==sum(counts), (counts,len(calls))
        relative={'place_position_modes':['relative']*3,
                  **{'place_positions.'+c:[-0.05,-0.01,0.0005] for c in ['red','green','blue']}}
        calls,_=run_case(cell,'relative_xyz',[red],overrides=relative,expected=True)
        p=calls[0].place_pose.pose.position
        assert len(calls)==1 and abs(p.x-0.27)<1e-9 and abs(p.y+0.13)<1e-9 and abs(p.z-0.0055)<1e-9
        mixed={**relative,'place_position_modes':['relative','absolute','absolute'],
               'place_positions.red':[-0.05,-0.06,-0.12]}
        calls,_=run_case(cell,'mixed_xyz',[red],overrides=mixed,expected=True)
        p=calls[0].place_pose.pose.position
        assert abs(p.x-0.27)<1e-9 and p.y==-0.06 and p.z==-0.12
        absolute={**mixed,'place_position_modes':['absolute']*3,'place_positions.red':[0.26,-0.06,-0.12]}
        calls,_=run_case(cell,'absolute_xyz',[red],overrides=absolute,expected=True)
        p=calls[0].place_pose.pose.position
        assert p.x==0.26 and p.y==-0.06 and p.z==-0.12
        calls,_=run_case(cell,'relative_frozen_retry',[red],moved,overrides=relative,expected=True)
        assert len(calls)==2 and calls[0].place_pose.pose==calls[1].place_pose.pose
        assert calls[1].object_pose.pose.position.x>calls[0].object_pose.pose.position.x
        calls,_=run_case(cell,'relative_empty_grasp',[red],lambda g,n:'empty' if n==1 else 'success',
                         overrides=relative,expected=True)
        assert len(calls)==2 and calls[0].place_pose.pose==calls[1].place_pose.pose
        calls,_=run_case(cell,'relative_changed_id',[red],lambda g,n:'success_new_id',overrides=relative,expected=True)
        assert len(calls)==1
        calls,_=run_case(cell,'relative_multi',[red,obj('red_2',1,0.32,-0.04),green],overrides=relative,expected=True)
        assert len(calls)==3 and len({(g.place_pose.pose.position.x,g.place_pose.pose.position.y) for g in calls})==3
        zero={**relative,**{'place_positions.'+c:[0.0,0.0,0.0] for c in ['red','green','blue']}}
        calls,_=run_case(cell,'relative_zero',[red],overrides=zero,expected=True)
        assert not calls
        calls,_=run_case(cell,'relative_blocked',[red,obj('blocker',2,0.27,-0.13)],overrides=relative,colors=[1],
                         cancel_when=lambda _:cell.frame_count>=30 and not cell.calls)
        assert not calls
        print('All adaptive planner logic probes passed (synthetic observations).', flush=True)
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(3)
        cell.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
