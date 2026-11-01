"""测试的进程、输出目录和通信环境；导入模块不会启动任何程序。"""

from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
from zoneinfo import ZoneInfo


ROOT = Path(__file__).resolve().parents[2]


def output_directory(group, tag=None):
    """每次运行独立保存配置和日志，避免覆盖上一次验收证据。"""
    if tag is not None and (Path(tag).name != tag or tag in {'.', '..'}):
        raise ValueError('tag 必须是单个目录名')
    name = tag or datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'log' / group / name
    output.mkdir(parents=True, exist_ok=False)
    return output


def configure_ros(output, domain, gazebo_port=None, middleware=None):
    """在导入 rclpy/消息类型之前设置通信环境；只影响本进程及其子进程。"""
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(output / 'ros'))
    if middleware:
        os.environ['RMW_IMPLEMENTATION'] = middleware
    if gazebo_port:
        os.environ['GAZEBO_MASTER_URI'] = f'http://127.0.0.1:{gazebo_port}'
    if (os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp'
            and not os.environ.get('CYCLONEDDS_URI')):
        config = ROOT / 'src/my_robot_bringup/config/cyclonedds_local.xml'
        os.environ['CYCLONEDDS_URI'] = config.as_uri()

    # 保存可 source 的环境片段，方便其他终端使用相同的发现设置。
    keys = ['ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
            'CYCLONEDDS_URI', 'GAZEBO_MASTER_URI']
    lines = [f'export {key}={shlex.quote(os.environ[key])}' if key in os.environ
             else f'unset {key}' for key in keys]
    (output / 'ros_env.bash').write_text('\n'.join(lines) + '\n')


def write_report(output, report):
    """先落盘再输出，失败时也能从同一个位置找到原因。"""
    text = json.dumps(report, ensure_ascii=False, indent=2)
    (output / 'report.json').write_text(text + '\n')
    print(text, flush=True)


class Processes:
    """只管理自己创建的进程组，退出时连同 launch 的子进程一起回收。"""

    def __init__(self, output):
        self.output = output
        self.entries = []

    def __enter__(self):
        return self

    def start(self, command, name):
        log = (self.output / f'{name}.log').open('w')
        try:
            process = subprocess.Popen(command, cwd=ROOT, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        except Exception:
            log.close()
            raise
        self.entries.append((process, log, name))
        return process

    @staticmethod
    def stop(process):
        # 即使 launch 已退出，也尝试清理同组内仍存活的子进程。
        for signum, timeout in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            # 父进程结束不保证整个进程组已经结束，继续清理剩余成员。

    def close(self):
        for process, log, _ in reversed(self.entries):
            try:
                self.stop(process)
            finally:
                log.close()
        self.entries.clear()

    def __exit__(self, *_):
        self.close()


def display_command(mode):
    """headless 仍需渲染 RGB-D，相机无 DISPLAY 时通过虚拟显示运行。"""
    return ['xvfb-run', '-a', '-s', '-screen 0 1280x800x24'] if (
        mode == 'headless' and not os.environ.get('DISPLAY')) else []


def interrupt_on_sigterm():
    """让 SIGTERM 走与 Ctrl+C 相同的 finally 清理路径。"""
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
