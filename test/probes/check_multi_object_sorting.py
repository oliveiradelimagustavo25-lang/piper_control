#!/usr/bin/env python3
"""原单脚本入口的类别抓放回归；预期直接取自输入，不从生成后的 XML 反推。"""
import argparse
from collections import Counter
import importlib.util
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from my_robot_interface.action import SortObjects
from my_robot_interface.msg import DetectedObjectArray
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/mujoco_launch.yaml')
    parser.add_argument('--count', action='append', default=[], metavar='ID=N', help='生成临时测试场景；可重复')
    parser.add_argument('--ros-domain-id', type=int, default=187)
    parser.add_argument('--timeout', type=float, default=420)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1')
    if os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp' and not os.environ.get('CYCLONEDDS_URI'):
        os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex>'
                                     '<MaxAutoParticipantIndex>63</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    directory=ROOT/'log/multi_object_sorting'/str(time.time_ns())
    directory.mkdir(parents=True)
    config=yaml.safe_load(args.config.read_text()) or {}
    for name in ('camera_config','controllers_file','moveit_controllers_file','manipulation_config',
                 'planner_config','perception_config','class_config','scene_file','robot_model'):
        if config.get(name):
            config[name]=str((args.config.resolve().parent/config[name]).resolve())
    spec=importlib.util.spec_from_file_location('scene_config',ROOT/'script/build/scene_config.py')
    helper=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    perception=config.get('perception_config',ROOT/'src/my_robot_perception/config/perception.yaml')
    catalog_path=helper.catalog_file(perception,config.get('class_config',''))
    catalog=helper.load_catalog(catalog_path)
    if args.count:
        counts={}
        for value in args.count:
            cid,count=map(int,value.split('='))
            if not 1<=cid<=255 or count<0 or cid in counts:
                parser.error('counts require unique IDs and nonnegative quantities')
            counts[cid]=count
        total=sum(counts.values())
        columns=max(1,math.ceil(total/5))
        rows=max(1,math.ceil(total/columns))
        scene={}
        index=0
        for cid,count in counts.items():
            scene[cid]=[]
            for _ in range(count):
                x=0.32+(0.04*(index//rows)/(columns-1) if columns>1 else 0)
                y=-0.12+0.24*(index%rows)/(rows-1) if rows>1 else 0
                scene[cid].append([x,y,0.03])
                index+=1
        config.pop('scene_file',None)
        config['scenes']=scene
    requested=config.get('scenes')
    if requested is None:
        requested={cid:[c['simulation']['default_position']] for cid,c in catalog.items()
                   if 'default_position' in c.get('simulation',{})}
    initial=[]
    for key,positions in requested.items():
        cid=int(key)
        assert cid in catalog, f'Undefined requested class {cid}; normal physical probe expects no skips'
        assert catalog[cid]['geometry']['type'] in ('box', 'cylinder'), 'Physical probe covers upright boxes/cylinders'
        for position in positions:
            source=list(position)
            source[2]=catalog[cid]['geometry']['dimensions'][2]/2
            initial.append((cid,source))
    expected=Counter(cid for cid,_ in initial)
    launch_config=directory/'mujoco_launch.yaml'
    launch_config.write_text(yaml.safe_dump(config))
    log_path=directory/'entry.log'
    process,node=None,None
    try:
        with log_path.open('w') as output:
            process=subprocess.Popen([str(ROOT/'script/pick_three_colors.sh'),'--config',str(launch_config),
                '--ros-domain-id',str(args.ros_domain_id),'--headless','--no-debug-window'],
                cwd=ROOT,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+30
            while '等待机械臂控制器' not in log_path.read_text():
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Entry startup failed: '+log_path.read_text()[-2500:])
                time.sleep(0.1)
            rclpy.init()
            node=Node('multi_object_sorting_probe')
            latest,samples=[],[]
            completed=[0]
            def observe(msg):
                latest[:]=msg.objects
                samples.append([(o.object_id,o.class_id,o.color,o.pose.position.x,o.pose.position.y,o.pose.position.z) for o in msg.objects])
            def feedback(msg):
                if msg.feedback.completed_count!=completed[0]:
                    completed[0]=msg.feedback.completed_count
                    print(f'Verified {completed[0]} objects',flush=True)
            node.create_subscription(DetectedObjectArray,'/perception/objects',observe,10)
            node.create_subscription(SortObjects.Impl.FeedbackMessage,'/sorting/sort_objects/_action/feedback',feedback,10)
            def wait(condition,seconds):
                until=time.monotonic()+seconds
                while not condition():
                    if time.monotonic()>until or process.poll() is not None:
                        raise RuntimeError('Entry did not complete: '+log_path.read_text()[-2500:])
                    rclpy.spin_once(node,timeout_sec=0.1)
            client=node.create_client(GetParameters,'/task_planner/get_parameters')
            wait(client.service_is_ready,60)
            ids=list(catalog)
            request=client.call_async(GetParameters.Request(names=['slot_tolerance','place_position_modes','use_position_rules','class_config']))
            wait(request.done,10)
            values=request.result().values
            assert len(values)==4, values
            tolerance=values[0].double_value
            modes=values[1].string_array_value
            assert values[3].string_value==str(catalog_path)
            use_rules=values[2].bool_value
            names=[('place_positions.'+str(cid)) if use_rules else ('classes.'+str(cid)+'.place_slots') for cid in ids]
            request=client.call_async(GetParameters.Request(names=names))
            wait(request.done,10)
            coordinates=request.result().values
            assert len(coordinates)==len(ids), coordinates
            rules={cid:coordinates[i].double_array_value for i,cid in enumerate(ids)}
            targets={cid:list(zip(*[iter(coordinates[i].double_array_value)]*3)) for i,cid in enumerate(ids)}
            expected_moves=0
            if use_rules:
                targets={cid:[] for cid in ids}
                for cid,source in initial:
                    assert len(rules[cid])==3, f'Class {cid} has no placement rule'
                    target=tuple(v+source[i] if modes[i]=='relative' else v for i,v in enumerate(rules[cid]))
                    targets[cid].append(target)
                    expected_moves+=math.dist(source,target)>tolerance
            else:
                expected_moves=sum(not any(math.dist(source,p)<=tolerance for p in targets[cid]) for cid,source in initial)
            wait(lambda:'仿真保持运行' in log_path.read_text(),args.timeout)
            text=log_path.read_text()
            assert '本批任务未全部完成' not in text, text[-2000:]
            assert 'skipped_targets=0' in text
            if use_rules:
                # 相对规则基准是首次派发观测，不是 XML 初始坐标。独立检查
                # 源位姿+规则=派发目标，再用原容差检查最终视觉位置。
                dispatched={}
                measured_targets={cid:[] for cid in ids}
                for name,cid,source,target in re.findall(
                        r'Pick (\S+) class=(\d+) \(([^)]+)\) -> slot \d+ \(([^)]+)\)',text):
                    cid=int(cid)
                    source=tuple(map(float,source.split(',')))
                    target=tuple(map(float,target.split(',')))
                    if name not in dispatched:
                        predicted=tuple(v+source[i] if modes[i]=='relative' else v for i,v in enumerate(rules[cid]))
                        assert math.dist(predicted,target)<=0.0002,(name,predicted,target)
                        dispatched[name]=target
                        measured_targets[cid].append(predicted)
                    else:
                        assert math.dist(dispatched[name],target)<=0.0002,'Retry changed frozen placement'
                for cid in ids:
                    if measured_targets[cid]:
                        targets[cid]=measured_targets[cid]
            assert completed[0]==expected_moves,(completed,expected_moves)
            assert Counter(o.class_id for o in latest)==expected,[(o.object_id,o.class_id) for o in latest]
            for obj in latest:
                p=obj.pose.position
                assert any(math.dist((p.x,p.y,p.z),target)<=tolerance for target in targets[obj.class_id]),obj
                assert [obj.dimensions.x,obj.dimensions.y,obj.dimensions.z]==catalog[obj.class_id]['geometry']['dimensions']
            (directory/'observations.yaml').write_text(yaml.safe_dump(samples))
            print(f'PASS one-click MuJoCo: classes={dict(expected)}, verified={completed[0]}; log: {log_path}',flush=True)
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()
