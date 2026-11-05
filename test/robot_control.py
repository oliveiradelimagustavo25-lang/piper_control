#!/usr/bin/env python3
"""显示机器人本体并提供 ros2_control 接口，供交互测试使用。"""

import argparse
import os
from pathlib import Path
import signal
import subprocess


def stop_launch(process):
    """停止本测试创建的启动进程组。"""
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='不打开窗口，保留控制接口')
    parser.add_argument('--sim-speed-factor', type=float, default=1.0, help='目标仿真倍率')
    parser.add_argument('--controllers-file', type=Path, help='替换控制器配置')
    parser.add_argument('--ros-domain-id', type=int, help='覆盖当前 ROS 域；默认继承终端环境')
    args = parser.parse_args()
    env = os.environ.copy()
    if args.ros_domain_id is not None:
        env['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    command = [
        'ros2', 'launch', 'my_robot_bringup', 'ros2_control.launch.py',
        'with_moveit:=false', 'robot_only:=true',
        f'headless:={str(args.headless).lower()}',
        f'sim_speed_factor:={args.sim_speed_factor}',
    ]
    if args.controllers_file:
        command.append(f'controllers_file:={args.controllers_file.resolve()}')
    print(f"机器人控制测试，ROS 域 {env.get('ROS_DOMAIN_ID', '0')}；Ctrl+C 退出。", flush=True)
    print('轨迹接口：/arm_controller/follow_joint_trajectory、'
          '/gripper_controller/follow_joint_trajectory', flush=True)
    process = subprocess.Popen(command, env=env, start_new_session=True)
    try:
        # 保持窗口与接口在线；不自动发送动作或在验证后关闭。
        return process.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        stop_launch(process)


if __name__ == '__main__':
    raise SystemExit(main())
