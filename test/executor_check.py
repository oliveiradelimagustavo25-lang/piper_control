#!/usr/bin/env python3
"""分阶段验证 Gazebo 执行器；默认运行完整回归。"""

import argparse
from pathlib import Path
import math
import sys


STAGES = ['ready', 'motion', 'contract', 'scene', 'air', 'physical', 'recovery']


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['all'] + STAGES, default='all')
    parser.add_argument('--mode', choices=['gui', 'headless'], default='headless')
    parser.add_argument('--skip-motion', action='store_true', help='完整回归时跳过运动探针')
    parser.add_argument('--skip-air', action='store_true', help='完整回归时跳过空载抓放')
    parser.add_argument('--profile', choices=['current', 'baseline', 'ground'], default='current')
    parser.add_argument('--cube-size', type=float, default=0.03, help='物块边长，m')
    parser.add_argument('--cube-mass', type=float, help='物块质量，kg；默认按体积缩放')
    parser.add_argument('--friction', type=float, default=1.0)
    parser.add_argument('--tcp-z', type=float)
    parser.add_argument('--contact-layer', type=float)
    parser.add_argument('--compression', type=float)
    parser.add_argument('--tag', help='本次输出目录名，不能与已有目录重名')
    args = parser.parse_args(argv)
    values = [args.cube_size, args.friction, args.cube_mass,
              args.tcp_z, args.contact_layer, args.compression]
    if any(value is not None and not math.isfinite(value) for value in values):
        parser.error('数值参数必须有限')
    if not 0 < args.cube_size < 0.07 or args.friction < 0:
        parser.error('cube-size 必须在 0～0.07 m 之间，friction 不能为负')
    if args.cube_mass is not None and args.cube_mass <= 0:
        parser.error('cube-mass 必须大于零')
    if any(value is not None and value < 0 for value in [args.contact_layer, args.compression]):
        parser.error('contact-layer 和 compression 不能为负')
    if args.stage != 'all' and (args.skip_motion or args.skip_air):
        parser.error('单阶段直接使用 --stage，skip 参数只用于完整回归')
    return args


def run_stages(session, processes, robot_xml):
    from test.stages.executor_contract import check_contract
    from test.stages.executor_motion import check_motion
    from test.stages.executor_physical import check_air, check_physical, check_recovery, spawn_cube
    from test.stages.executor_scene import check_scene

    selected = STAGES if session.args.stage == 'all' else [session.args.stage]
    operations = {
        'ready': lambda: True,
        'motion': lambda: check_motion(session, processes, robot_xml),
        'contract': lambda: check_contract(session),
        'scene': lambda: check_scene(session),
        'air': lambda: check_air(session),
    }
    cube = None
    for name in selected:
        if (name == 'motion' and session.args.skip_motion) or (name == 'air' and session.args.skip_air):
            session.report['stages'][name] = {'skipped': True}
            continue
        # physical/recovery 共用本次物块；单独执行 recovery 时自行创建。
        session.report['stages'][name] = {'passed': False}
        print(f'开始阶段：{name}', flush=True)
        if name in {'physical', 'recovery'}:
            cube = cube or spawn_cube(session)
            passed = check_physical(session, cube) if name == 'physical' else check_recovery(session, cube)
        else:
            passed = operations[name]()
        session.report['stages'][name]['passed'] = passed
        if not passed:
            raise AssertionError(f'{name} 阶段未通过')


def main(argv=None):
    args = arguments(argv)
    from test.support.runtime import (Processes, configure_ros, display_command,
                                      interrupt_on_sigterm, output_directory, write_report)

    output = output_directory('executor_validation', args.tag)
    configure_ros(output, domain=93, gazebo_port=11493, middleware='rmw_fastrtps_cpp')
    interrupt_on_sigterm()
    report = {'mode': args.mode, 'stage': args.stage, 'passed': False, 'stages': {},
              'motion_skipped': args.skip_motion, 'air_skipped': args.skip_air,
              'motion': [], 'actions': {}, 'physical': {}, 'profile': args.profile,
              'cube_size_m': args.cube_size, 'friction_mu': args.friction,
              'tcp_z_override': args.tcp_z, 'contact_layer_override': args.contact_layer,
              'compression_override': args.compression}
    print('输出目录：' + str(output), flush=True)
    try:
        from test.support.executor_session import ExecutorSession
        from test.support.scenarios import prepare_executor

        robot_xml, control = prepare_executor(args, output)
        report['simulation_control'] = control
        gui = str(args.mode == 'gui').lower()
        with Processes(output) as processes:
            process = processes.start(display_command(args.mode) + [
                'ros2', 'launch', 'my_robot_bringup', 'gazebo.launch.py',
                'ros_domain_id:=93', 'gui:=' + gui, 'use_rviz:=' + gui,
                'use_task_planner:=false', 'with_perception:=false', 'spawn_objects:=false', 'world:=' + str(output / 'world.world'),
                'manipulation_config:=' + str(output / 'manipulation.yaml'),
                'robot_model:=' + str(output / 'robot.urdf')], 'bringup')
            with ExecutorSession(args, output, report, process) as session:
                session.ready()
                run_stages(session, processes, robot_xml)
                report['passed'] = True
    except KeyboardInterrupt:
        report['error'] = '测试被中断'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        write_report(output, report)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
