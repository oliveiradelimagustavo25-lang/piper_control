# Piper MuJoCo 工作区

本分支面向 Piper 机械臂的 MuJoCo 仿真。正式入口已组合机器人、地面、物块、标记板和相机，并提供 ros2_control、MoveIt 与执行接口；相机数据发布、规划场景物体同步、任务规划和完整分拣尚未接入。

## 当前结构

```text
src/
  my_robot_description/       # URDF、机器人 MJCF、网格和 RViz 配置
  my_robot_world/            # MuJoCo 世界、物块、标记板和相机参数
    objects/
    basic/preview.xml         # 基础世界组合入口
    scenes/scene.py           # 场景和跨包机器人资源加载
    sensors/camera.py         # 腕部相机
    assets/
    config/camera.yaml
  my_robot_bringup/           # 仿真、控制与执行启动入口
    launch/mujoco.launch.py
    launch/ros2_control.launch.py # 可复用的仿真控制启动层
  my_robot_interface/
  my_robot_perception/
  my_robot_planner/
  my_robot_commander/
  my_robot_moveit_pkg/

test/
  robot_description.py        # 机器人本体查看，可选 RViz
  mujoco_world.py             # 完整世界加载、物理检查和查看
  robot_control.py            # 机器人本体窗口与 ros2_control 交互测试
  probes/check_robot_control.py # 自动状态、时钟与轨迹检查
  support/viewer.py            # 两个 Python 查看器的退出管理
  README.md

docs/                         # 讨论与开发报告
```

旧 `demo_robot_pkg`、`my_robot_model` 和 `my_robot_sim` 在当前工作区已删除；MoveIt 描述引用已迁移，其他旧包仍应按后续集成情况检查。新模型和原 SDF 的物块尺寸、质量及颜色对应，初始位置属于测试配置。

| 入口 | 模型与用途 |
| --- | --- |
| `test/robot_description.py` | 静态机器人本体，可选 RViz |
| `test/mujoco_world.py` | 世界与传感器预览，由 Python 推进物理 |
| `test/robot_control.py` | 机器人本体窗口与 ros2_control 接口，供交互测试 |
| `my_robot_bringup/mujoco.launch.py` | 完整世界、ros2_control、MoveIt 与执行接口 |

## 运行与检查

```bash
# 打开 MuJoCo 机器人本体查看器
python3 test/robot_description.py

# 检查加载，不打开窗口
python3 test/robot_description.py --check

# 加载并显示完整世界（会推进物理）
python3 test/mujoco_world.py

# 检查世界，不打开窗口
python3 test/mujoco_world.py --check
```

机器人本体查看不推进物理，也不使用 URDF。要同时打开 RViz：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
python3 test/robot_description.py --rviz
```

构建完成后需要重新执行 `source install/setup.bash`，使当前终端发现新增或重命名的包；`colcon build` 不会自动刷新终端环境。新终端运行 ROS 命令前也要加载该工作区。若终端曾加载已删除的安装路径，建议新开终端，先加载 `/opt/ros/humble/setup.bash` 再构建，避免旧前缀警告。详细原因见[工作区环境诊断](docs/2026-11-02_workspace_environment_fix.md)。

系统或 ROS 库更新后，如果运行时提示找不到旧版本共享库，需要重新配置并编译受影响的包。2026-11-03 的具体检查、四个 C/C++ 包重建命令和剩余问题见[系统库检查报告](docs/2026-11-03_system_library_audit.md)。

机器人控制链使用相同的环境：

```bash
# 完整世界、ros2_control、MoveIt 和抓放执行接口
ros2 launch my_robot_bringup mujoco.launch.py

# 只打开机器人本体窗口，保持 ros2_control 接口在线
python3 test/robot_control.py

# 自动检查状态、时钟、控制器和小幅轨迹
python3 test/probes/check_robot_control.py
```

`mujoco.launch.py headless:=true` 可无窗口运行；`with_moveit:=false` 保留完整世界与控制器；`with_executor:=false` 只关闭抓放执行端；`sim_speed_factor:=0.5` 设置半速仿真。场景、控制器、执行配置和常用运动参数均提供 arguments，使用 `--show-args` 查看，定义见 [bringup/README.md](src/my_robot_bringup/README.md)。交互测试继承终端 ROS 域，自动探针默认使用域 184。每个阶段独立运行，避免同域控制仿真冲突。正式入口不启动检测器和任务策略。

RViz 使用 URDF 和关节滑块；当前滑块不驱动 MuJoCo。两个可视化窗口需要图形环境。其他选项、相机渲染和安装路径说明见 [test/README.md](test/README.md)。

## 分工与下一步

`my_robot_description` 保存机械臂和相机外形、安装坐标；`my_robot_world` 使用它组合世界，并在相机光学坐标系上添加 MuJoCo 相机。场景文件通过包内加载器解析机器人引用，避免复制机器人模型。当前相机只支持 MuJoCo 渲染，尚未发布 ROS 图像、深度或 CameraInfo；画面遮挡和标记板可见性还需调整与校验。

相机与支架网格已修正额外 90° 上轴旋转，两个测试查看器恢复侧栏，世界加入棋盘格地面。查看器退出时等待其线程清理完成；世界窗口支持空格暂停/继续、右方向键单步。修改与验证结果见[相机、查看器与启动参数修正报告](docs/2026-11-02_camera_viewer_launch_fix.md)。

当前完整世界由原生控制插件统一推进，测试脚本不作为正式运行库。`motion_controller.cpp` 作为共享库由 `pick_place_executor` 使用，执行参数由 YAML 配置并允许 launch 覆盖。职责与接口修正见[完整世界启动报告](docs/2026-11-02_world_control_roles.md)。

后续继续校准控制精度、同步 MuJoCo 物体到 MoveIt 规划场景，并接入相机发布、感知和任务规划。当前环境的部分 C++ 节点在退出时仍有析构异常，详见本轮报告。

正式测试和必要资源建议纳入 Git；临时测试放在 `test/local/`，构建产物和缓存已忽略。临时节点移除及当前执行入口见[开发报告](docs/2026-11-02_remove_temporary_commander.md)，MoveIt 和执行链整合见[整合报告](docs/2026-11-02_mujoco_moveit_execution.md)，此前控制链构建见[控制报告](docs/2026-11-02_mujoco_control.md)，此前世界构建过程见[构建报告](docs/2026-11-02_mujoco_world_construction.md)。
