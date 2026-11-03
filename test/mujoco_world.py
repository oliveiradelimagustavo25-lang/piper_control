#!/usr/bin/env python3
"""检查机器人、物块、标记板和腕部相机组成的 MuJoCo 场景。"""

import argparse
import threading
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

from support.viewer import open_viewer


WORKSPACE = Path(__file__).resolve().parents[1]
# 从工作区执行时直接使用场景包的源码；模型资源可另行指定为安装目录。
sys.path.insert(0, str(WORKSPACE / 'src'))
from my_robot_world.scenes.scene import load_scene


def check_scene(model):
    """检查相机坐标与三个自由物块能否在地面上落稳。"""
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'wrist_camera')
    optical = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'camera_optical_link')
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ground')
    board = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'aruco_board')
    if min(camera, optical, ground, board) < 0 or model.cam_bodyid[camera] != optical:
        raise ValueError('场景缺少地面、标记板或安装到光学坐标系的相机')
    bodies = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
              for name in ('red_cube', 'green_cube', 'blue_cube')]
    if min(bodies) < 0 or not np.allclose(model.body_mass[bodies], 0.001):
        raise ValueError('物块缺失或质量与原模型不一致')

    data = mujoco.MjData(model)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    heights = data.xpos[bodies, 2]
    speeds = np.linalg.norm(data.cvel[bodies, 3:], axis=1)
    if (not np.isfinite(data.qpos).all() or
            not np.all((heights > 0.004) & (heights < 0.006)) or np.any(speeds > 0.01)):
        raise ValueError(f'物块未落稳：高度 {heights}，速度 {speeds}')
    print(f'场景检查通过：3 个物块已落地，相机垂直视场 {model.cam_fovy[camera]:.2f}°。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world-dir', type=Path, default=WORKSPACE / 'src/my_robot_world')
    parser.add_argument('--description-dir', type=Path, default=WORKSPACE / 'src/my_robot_description')
    parser.add_argument('--check', action='store_true', help='检查场景并退出，不打开窗口')
    parser.add_argument('--render-camera', type=Path, help='保存腕部相机的 RGB 图像')
    args = parser.parse_args()
    try:
        model = load_scene(args.world_dir, args.description_dir)
        check_scene(model)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        if args.render_camera:
            import cv2
            with mujoco.Renderer(model, height=600, width=800) as renderer:
                renderer.update_scene(data, camera='wrist_camera')
                rgb = renderer.render()
            if not cv2.imwrite(str(args.render_camera), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise OSError(f'无法保存图像：{args.render_camera}')
            print(f'相机图像已保存：{args.render_camera}')
        if not args.check:
            paused = threading.Event()
            single_step = threading.Event()

            def on_key(key):
                if key == 32:  # Space：暂停/继续物理；侧栏仍可操作。
                    if paused.is_set():
                        paused.clear()
                    else:
                        paused.set()
                elif key == 262 and paused.is_set():  # Right：暂停时推进一步。
                    single_step.set()

            print('空格暂停/继续，右方向键单步；Tab / Shift+Tab 切换侧栏，Ctrl+C 退出。', flush=True)
            with open_viewer(model, data, key_callback=on_key) as (viewer, interrupted):
                while viewer.is_running() and not interrupted.is_set():
                    started = time.monotonic()
                    with viewer.lock():
                        if not paused.is_set() or single_step.is_set():
                            mujoco.mj_step(model, data)
                            single_step.clear()
                    viewer.sync()
                    elapsed = time.monotonic() - started
                    interrupted.wait(max(0, model.opt.timestep - elapsed))
            if interrupted.is_set():
                return 130
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError) as error:
        print(f'场景检查失败：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
