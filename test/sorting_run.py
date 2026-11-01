#!/usr/bin/env python3
"""完整三色分拣的 Python 入口，Shell 仅负责加载工作区环境。"""

import argparse
import logging
import os
from pathlib import Path
import sys


def color_order(value):
    try:
        colors = [int(color) for color in value.split(',')]
    except ValueError as error:
        raise argparse.ArgumentTypeError('颜色使用 1/2/3，以逗号分隔') from error
    if not colors or len(set(colors)) != len(colors) or any(color not in {1, 2, 3} for color in colors):
        raise argparse.ArgumentTypeError('颜色必须是互不重复的 1/2/3')
    return colors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', nargs='?', choices=['gui', 'headless'], default='gui')
    parser.add_argument('strategy', nargs='?', choices=['color_order', 'nearest'], default='color_order')
    parser.add_argument('order', nargs='?', type=color_order, default='1,2,3')
    parser.add_argument('--tag', help='本次输出目录名，不能与已有目录重名')
    parser.add_argument('--ros-domain-id', type=int, default=os.environ.get('SORTING_ROS_DOMAIN_ID', '95'))
    args = parser.parse_args(argv)

    from test.support.runtime import (Processes, configure_ros, display_command,
                                      interrupt_on_sigterm, output_directory, write_report)

    output = output_directory('sorting', args.tag)
    configure_ros(output, domain=args.ros_domain_id, gazebo_port=11495)
    interrupt_on_sigterm()
    logging.basicConfig(level=logging.INFO, format='%(message)s', handlers=[
        logging.StreamHandler(), logging.FileHandler(output / 'task.log')])
    report = {'strategy': args.strategy, 'color_order': args.order, 'passed': False}
    print('输出目录：' + str(output), flush=True)
    print(f"其他终端查询：source '{output / 'ros_env.bash'}'", flush=True)
    try:
        import rclpy
        from test.stages.sorting import SortingCheck
        from test.support.scenarios import prepare_sorting

        prepare_sorting(output)
        gui = str(args.mode == 'gui').lower()
        with Processes(output) as processes:
            process = processes.start(display_command(args.mode) + [
                'ros2', 'launch', 'my_robot_bringup', 'gazebo_robot_model.launch.py',
                f'ros_domain_id:={args.ros_domain_id}', 'gui:=' + gui, 'use_rviz:=' + gui,
                'world:=' + str(output / 'world.world'),
                'task_planner_config:=' + str(output / 'task_planner.yaml')], 'bringup')
            rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
            node = rclpy.create_node('sorting_script_client')
            check = None
            try:
                check = SortingCheck(node, output, process)
                check.ready()
                check.spawn_objects(processes, args.ros_domain_id)
                check.execute(args.strategy, args.order, report)
            finally:
                if check:
                    report['vision'] = check.vision
                    check.close()
                node.destroy_node()
                rclpy.try_shutdown()
                # GUI 运行会等待用户关闭，在等待之前先保存任务报告。
                if args.mode == 'gui':
                    write_report(output, report)
            if args.mode == 'gui':
                print('任务结束，窗口保留；按 Ctrl+C 关闭本次仿真。', flush=True)
                process.wait()
    except KeyboardInterrupt:
        if not report['passed']:
            report['error'] = '分拣被中断'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        report['passed'] = False
    finally:
        write_report(output, report)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(main())
