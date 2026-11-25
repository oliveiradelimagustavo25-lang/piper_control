"""按工作区和 ROS 域清理孤立启动链；不终止正常 launch 管理的任务。"""
import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import shlex
import signal
import time

ROOT = Path(__file__).resolve().parents[2]
NODE_PATHS = (
    '/lib/robot_state_publisher/robot_state_publisher',
    '/lib/mujoco_ros2_control/ros2_control_node',
    '/lib/controller_manager/spawner',
    '/lib/moveit_ros_move_group/move_group',
    '/lib/my_robot_commander/pick_place_executor',
    '/lib/my_robot_planner/task_planner',
    '/lib/my_robot_perception/color_detector',
    '/lib/my_robot_perception/depth_localizer',
    '/lib/my_robot_perception/yolo_detector',
    '/lib/my_robot_perception/aruco_detector',
    '/lib/my_robot_perception/grasp_candidate_publisher',
)
LAUNCH_FILES = {'mujoco.launch.py', 'ros2_control.launch.py', 'motion_control.launch.py', 'perception.launch.py'}
PERCEPTION_LAUNCH_FILES = {
    'aruco_detector.launch.py', 'color_detector.launch.py',
    'depth_localizer.launch.py', 'yolo_detector.launch.py',
}


@dataclass
class Process:
    pid: int
    parent: int
    started: str
    kind: str
    domain: int
    cwd: Path
    executable: str
    orphan: bool
    workspace_owned: bool = False


def classify(args, cwd, root):
    """识别实际节点、ROS CLI 包装器和 GraspNet 在线推理链。"""
    if not args:
        return None, False
    executable = Path(args[0])
    shell = executable.name in {'sh', 'bash', 'dash'}
    if shell and len(args) == 3 and args[1] == '-c':
        args = shlex.split(args[2])
    if args and Path(args[0]).name.startswith('python'):
        args = args[1:]
    if not args:
        return None, False
    executable = Path(args[0])
    path = Path(os.path.abspath(cwd / executable))
    if (path == root / 'tools/graspnet/scripts/predict.py' and
            all(flag in args[1:] for flag in ('--input', '--out', '--proposals'))):
        return ('inference_shell' if shell else 'inference'), True
    if shell:
        return None, False
    if executable.name == 'ros2' and len(args) >= 4:
        operation, package, target = args[1:4]
        if operation == 'launch' and (
                (package == 'my_robot_bringup' and target in LAUNCH_FILES)
                or (package == 'my_robot_perception' and target in PERCEPTION_LAUNCH_FILES)
                or (package == 'my_robot_planner' and target == 'task_planner.launch.py')
                or (package == 'my_robot_moveit_pkg' and target == 'moveit_rviz.launch.py')):
            return 'launch', False
        if (operation == 'run' and package.startswith('my_robot_') and
                f'/lib/{package}/{target}' in NODE_PATHS):
            return 'runner', False
        return None, False
    if str(path).endswith(NODE_PATHS):
        return 'node', path.is_relative_to(root)
    if str(path).endswith('/lib/rviz2/rviz2'):
        configs = (Path(os.path.abspath(cwd / name))
                   for option, name in zip(args, args[1:]) if option == '-d')
        return 'rviz', any(config.is_relative_to(root) and config.name == 'moveit.rviz'
                           for config in configs)
    return None, False


def read_process(pid):
    proc = Path('/proc') / str(pid)
    try:
        if proc.stat().st_uid != os.getuid():
            return None
        # comm 可以包含空格，不能直接把整条 stat 按空格取第 22 列。
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        args = [os.fsdecode(x) for x in (proc / 'cmdline').read_bytes().split(b'\0') if x]
        cwd = Path(os.readlink(proc / 'cwd'))
        kind, owned = classify(args, cwd, ROOT)
        if kind is None:
            return None
        domain = next((x.split(b'=', 1)[1] for x in (proc / 'environ').read_bytes().split(b'\0')
                       if x.startswith(b'ROS_DOMAIN_ID=')), b'0')
        parent = int(fields[1])
        orphan = parent == 1 or (Path('/proc') / str(parent) / 'comm').read_text().strip() == 'systemd'
        return Process(int(pid), parent, fields[19], kind, int(domain),
                       cwd, args[0], orphan, owned)
    except (OSError, ValueError, IndexError):
        return None


def in_scope(process, root, domain):
    return process.domain == domain and (process.workspace_owned or process.cwd.is_relative_to(root))


def select_targets(processes, root, domain):
    scoped = {p.pid: p for p in processes if in_scope(p, root, domain)}
    roots = {pid for pid, p in scoped.items()
             if p.orphan and (p.kind != 'rviz' or p.workspace_owned)}

    def managed_by_orphan(pid):
        visited = set()
        while pid in scoped and pid not in visited:
            if pid in roots:
                return True
            visited.add(pid)
            pid = scoped[pid].parent
        return False

    targets = [p for p in scoped.values() if managed_by_orphan(p.pid)]
    leaders = [scoped[pid] for pid in sorted(roots)]
    return sorted(targets, key=lambda p: (p.kind != 'launch', p.pid)), leaders


def alive(original):
    current = read_process(original.pid)
    return current is not None and (current.started, current.domain, current.cwd, current.executable) == (
        original.started, original.domain, original.cwd, original.executable)


def send(processes, sig):
    for p in processes:
        if alive(p):
            try:
                os.kill(p.pid, sig)
            except ProcessLookupError:
                pass


def wait_exit(processes, seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if not any(alive(p) for p in processes):
            return True
        time.sleep(0.2)
    return not any(alive(p) for p in processes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='只显示候选和原因，不发送信号')
    parser.add_argument('--ros-domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    args = parser.parse_args()
    processes = [p for entry in Path('/proc').iterdir() if entry.name.isdigit()
                 if (p := read_process(entry.name)) is not None]
    targets, leaders = select_targets(processes, ROOT, args.ros_domain_id)
    print(f'工作区: {ROOT}；ROS 域: {args.ros_domain_id}', flush=True)
    if not targets:
        active = [p for p in processes if in_scope(p, ROOT, args.ros_domain_id)]
        print('没有发现符合条件的残留启动链。')
        if active:
            print(f'另有 {len(active)} 个相关进程仍由活动启动进程管理，未清理；请在对应终端正常退出。')
        print('若 node list 仍有记录，可用 ros2 node list --no-daemon 区分发现缓存与其他来源的节点。')
        return 0
    for p in targets:
        reason = '已被 init/systemd 接管' if p.orphan else '残留启动链的子进程'
        print(f'PID={p.pid} PPID={p.parent} [{p.kind}]：{reason}；{p.executable}', flush=True)
    if args.dry_run:
        print('仅显示结果，没有发送信号。')
        return 0
    # 先让 launch 转发一次 SIGINT，避免对子进程重复发送；后续仅处理同一身份的存活 PID。
    send(leaders, signal.SIGINT)
    if not wait_exit(targets, 10):
        send(targets, signal.SIGTERM)
        if not wait_exit(targets, 3):
            send(targets, signal.SIGKILL)
            wait_exit(targets, 2)
    remaining = [p.pid for p in targets if alive(p)]
    if remaining:
        print(f'仍存活的 PID：{remaining}')
        return 1
    print('残留启动链已清理；所选进程均已退出。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
