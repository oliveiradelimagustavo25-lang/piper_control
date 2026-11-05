# 模型与世界测试

## 机器人本体

```bash
python3 test/robot_description.py            # 打开 MuJoCo 静态查看器
python3 test/robot_description.py --check    # 仅加载模型，不打开窗口
python3 test/robot_description.py --rviz     # 同时打开 RViz 和关节滑块
```

默认直接加载 `my_robot_description/mjcf/piper.xml`，无需 URDF；`--rviz` 才使用 URDF 和 ROS。RViz 的滑块只改变 RViz 中的关节姿态，当前不驱动 MuJoCo。关闭 MuJoCo 查看器或按 Ctrl+C 会清理本次启动的 RViz 节点。

此处是静态预览。MoveIt 规划 RViz 使用独立配置；先启动正式 `mujoco.launch.py`，再在同一 ROS 域运行 `ros2 launch my_robot_moveit_pkg moveit_rviz.launch.py use_sim_time:=true`。

## 完整世界

```bash
python3 test/mujoco_world.py                 # 检查并打开世界，推进物理
python3 test/mujoco_world.py --check         # 检查并退出，不打开窗口
python3 test/mujoco_world.py --check --render-camera /tmp/wrist.png
```

世界加载器组合 `my_robot_world/basic/preview.xml` 和机器人，添加光学相机和棋盘格地面。检查让物块在重力下运行 2 秒并判断落地高度，同时检查相机及主要资源是否存在。查看器运行时推进物理；当前没有控制器，机械臂会受重力运动。渲染相机需要 OpenGL 和 OpenCV；修正相机与支架网格后，默认姿态可看到标记板，夹爪仍占据部分视场。

两个查看器默认显示原生左右侧栏，可用 Tab / Shift+Tab 切换。在世界窗口按空格暂停/继续，暂停时按右方向键推进一个物理步。当前采用 passive 查看器，原生 Simulation 的 Pause/Run 按钮禁用，由这些快捷键控制脚本推进；本体查看始终保持静态。

关闭窗口或 Ctrl+C 后，`support/viewer.py` 等待本次查看器线程结束，再允许 Python 清理 GLFW。此辅助文件仅管理测试查看器生命周期；正式 ros2_control 入口独立运行。Ctrl+C 正常返回 130，窗口关闭正常返回 0。

## ROS 安装与路径

RViz 模式需要已构建并加载描述包：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
python3 test/robot_description.py --rviz
```

RViz 默认使用 ROS 域 96，可用 `--ros-domain-id` 修改。脚本默认从自身位置找到工作区源码；`--description-dir` 和世界脚本的 `--world-dir` 可指定安装后的资源目录。RViz 使用 `package://` 路径，因此指定目录需与当前 source 的描述包一致。

新增或重命名包后，即使构建成功，也要重新加载 `install/setup.bash`；否则当前终端可能仍只认识旧包。这里只使用默认 `build/` 和 `install/`，不要混用此前报告中的独立构建目录。

## 机器人控制：交互阶段

```bash
python3 test/robot_control.py                    # 默认打开机器人本体窗口并保持控制接口在线
python3 test/robot_control.py --headless         # 不显示窗口，仍保持接口在线
python3 test/robot_control.py --ros-domain-id 184 # 可选独立 ROS 域
```

先构建并加载工作区。此脚本复用 `ros2_control.launch.py`，显式设置 `robot_only:=true` 并关闭 MoveIt，只加载机器人本体和八个关节的控制接口，不自动发送动作；保持运行直到关闭 MuJoCo 窗口或按 Ctrl+C。默认继承终端的 ROS 域，便于另一个相同环境的终端发送测试指令。

主要接口是 `/joint_states`、`/clock`、`/arm_controller/follow_joint_trajectory` 和 `/gripper_controller/follow_joint_trajectory`。例如在另一终端测试夹爪：

```bash
source install/setup.bash
# 如测试时指定了 --ros-domain-id，这个终端也需设置相同 ROS_DOMAIN_ID。
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [joint7, joint8], points: [{positions: [0.01, -0.01], time_from_start: {sec: 3}}]}}'
```

`--sim-speed-factor` 调整目标仿真倍率，`--controllers-file` 替换控制器配置；默认配置位于 `my_robot_description/config/ros2_controllers.yaml`。机器人本体不包含世界地面和物块；完整世界与 MoveIt、抓放执行接口使用正式 `mujoco.launch.py`。

## 自动控制探针

```bash
python3 test/probes/check_robot_control.py
python3 test/probes/check_robot_control.py --viewer
```

原自动轨迹测试迁移到此处：在默认 ROS 域 184 启动机器人本体，检查状态、时钟、三个控制器，并发送小幅机械臂和夹爪轨迹，完成后退出。可通过 `--ros-domain-id` 换域。当前验收容差为机械臂 0.08 rad、夹爪 0.004 m，不代表实机精度。

每个阶段独立启动自己的仿真，测试结束后再运行其他阶段；避免相同 ROS 域的控制仿真互相干扰。`support/viewer.py` 只服务于前两个 Python 查看器，机器人控制窗口由原生插件负责。
