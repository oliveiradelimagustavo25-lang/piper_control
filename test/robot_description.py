#!/usr/bin/env python3
"""在 MuJoCo 中查看机器人本体，可选同时打开 RViz。"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

import mujoco

from support.viewer import open_viewer


DESCRIPTION = Path(__file__).resolve().parents[1] / 'src' / 'my_robot_description'


def stop_processes(processes):
    """只停止本次显示入口创建的进程组。"""
    for process in reversed(processes):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


def show_rviz(directory, urdf, domain):
    """使用独立 ROS 域显示 URDF；关节 GUI 只改变显示用关节状态。"""
    import yaml
    from ament_index_python.packages import get_package_share_directory

    installed = Path(get_package_share_directory('my_robot_description'))
    if installed.resolve() != directory.resolve():
        raise RuntimeError('RViz 使用 package:// 网格，请先构建并 source 目标描述包，再用 --description-dir 指向其安装目录')
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    processes = []
    temporary = tempfile.TemporaryDirectory(prefix='piper_preview_')
    params = Path(temporary.name) / 'robot.yaml'
    params.write_text(yaml.safe_dump({'robot_state_publisher': {
        'ros__parameters': {'robot_description': urdf.read_text()}}}))
    commands = [
        ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
         '--ros-args', '--params-file', str(params)],
        ['ros2', 'run', 'joint_state_publisher_gui', 'joint_state_publisher_gui'],
        ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(directory / 'rviz' / 'preview.rviz')],
    ]
    try:
        for command in commands:
            processes.append(subprocess.Popen(command, env=env, start_new_session=True))
        print(f'RViz 模型预览已启动，ROS 域 {domain}；关闭 MuJoCo 查看器或按 Ctrl+C 退出。', flush=True)
        return processes, temporary
    except Exception:
        stop_processes(processes)
        temporary.cleanup()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description-dir', type=Path, help='指定源码或安装后的机器人描述包目录')
    parser.add_argument('--rviz', action='store_true', help='同时打开 RViz 和关节滑块')
    parser.add_argument('--ros-domain-id', type=int, default=96, help='RViz 使用的 ROS 域')
    parser.add_argument('--check', action='store_true', help='只加载模型并退出，不打开窗口')
    args = parser.parse_args()
    if args.check and args.rviz:
        parser.error('--check 与 --rviz 不能同时使用')

    processes = []
    temporary = None
    try:
        directory = args.description_dir
        if directory is None and args.rviz:
            from ament_index_python.packages import get_package_share_directory
            directory = Path(get_package_share_directory('my_robot_description'))
        directory = (directory or DESCRIPTION).resolve()
        model_path = directory / 'mjcf' / 'piper.xml'
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        print(f'MuJoCo {mujoco.__version__}：已加载 {model_path}（{model.njnt} 个关节）', flush=True)
        if args.check:
            return 0
        if args.rviz:
            urdf = directory / 'urdf' / 'piper_description.urdf'
            processes, temporary = show_rviz(directory, urdf, args.ros_domain_id)
        # 只显示静态模型；不调用 mj_step，也不启动控制器。
        with open_viewer(model, data) as (viewer, interrupted):
            while viewer.is_running() and not interrupted.is_set():
                viewer.sync()
                interrupted.wait(0.03)
        return 130 if interrupted.is_set() else 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(f'机器人模型加载失败：{error}', file=sys.stderr)
        return 1
    finally:
        stop_processes(processes)
        if temporary is not None:
            temporary.cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
