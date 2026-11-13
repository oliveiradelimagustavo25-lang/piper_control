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

## MoveIt 规划场景与碰撞

```bash
python3 test/probes/check_planning_scene.py
python3 test/probes/check_planning_scene.py --grasp
python3 test/probes/check_planning_scene.py --recovery
```

探针默认使用独立 ROS 域 185，启动无窗口 MuJoCo 与 MoveIt。基础检查向感知话题发布合成物块，验证地面、标记板与物块碰撞盒、障碍加入和移除后的状态检测；`--grasp` 额外检查抓放期间附着及释放后的规划盒位置；`--recovery` 检查附着后取消、带载拒绝新抓放及显式释放。可用 `--ros-domain-id` 调整测试域、`--objects-topic` 测试自定义物体话题；若该域已发现其他节点，探针会退出。启动日志位于 `log/planning_scene/`。

这些检查使用合成视觉观测，验证的是 MoveIt 规划场景，不单独证明 MuJoCo 物理抓取成功；完整相机和物理效果需要另行观察。

## 自适应分拣

`python3 test/probes/check_class_catalog.py` 运行真实 CV 和 depth，验证默认三色四实例，以及显式覆盖为非连续类别编号后的颜色映射、各类标称尺寸、中心高度和原图时间戳；同时检查类别表文件相对路径及总入口优先级。这是正常数据流验证，不包含新的故障注入。

`probes/check_multi_object_perception.py` 使用合成标签/深度检查多实例、移动、重现、歧义和 TF 异常；`probes/check_adaptive_planner.py` 用模拟执行器检查缺色、重试、抓空核验、ROI、重现优先级及取消，不证明物理抓取。

`check_scene_config.py` 离线检查总配置场景生成与默认/优先级语义。`xvfb-run -a python3 test/probes/check_multi_object_sorting.py` 直接运行唯一 `.sh` 入口；可用 `--count 2=4`、`--count 1=3 --count 2=1 --count 3=1` 生成临时测试输入，也可直接测试 `--config` 指定的总配置。测试不传 XML，验收数量直接从请求输入推导，并按 class_id 核验，不从生成 XML 或颜色前缀反推。每次使用空闲独立 ROS 域，可用 `--ros-domain-id` 覆盖。详细配置与局限见 [planner 说明](../src/my_robot_planner/README.md)。

`python3 test/probes/check_clear_processes.py` 离线检查清理选择边界，不创建或终止任何节点；覆盖孤立 launch 连带节点、正常 launch 保留、跨域/跨目录排除与 RViz 范围。

`check_placement_config.py` 验证放置规则校验、实际 launch 参数优先级和脚本 dry-run。调度探针增加逐轴混合、负绝对 Z、冻结目标重试、抓空、目标 ID 改变、零偏移和占用等待等用例；完整入口探针同时支持区域和相对/绝对规则，从运行参数推导目标。

类别升级回归增加未定义 ID、缺失放置规则、同色不同类核验、不同尺寸预留及不完整感知汇总；场景生成检查请求/生成/跳过报告，并检查 CLI 类别表覆盖和圆柱几何尺寸（只验证生成，不证明圆柱识别抓取）。
