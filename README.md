# Piper 三色物块分拣

基于 **Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 / MoveIt 2**，完成红、绿、蓝物块的视觉定位与抓放。当前分支面向 Gazebo 仿真；MuJoCo 实现在独立分支。

流程：RGB-D 相机 → 颜色检测与深度定位 → 任务规划 → 抓放执行器 → MoveIt → ros2_control。

## 项目结构与架构

| 目录 | 职责 |
| --- | --- |
| `src/my_robot_description` | Piper URDF、网格、Gazebo 相机插件和 ros2_control 描述 |
| `src/my_robot_world` | Gazebo 世界、物块模型、相机设置和物体生成 launch |
| `src/my_robot_bringup` | 仿真控制、运动、感知分层启动及总入口 |
| `src/my_robot_perception` | 颜色标签、深度定位及 ArUco 检测 |
| `src/my_robot_interface` | 共用消息和 Action |
| `src/my_robot_planner` | 观察位、批次目标选择和分拣策略 |
| `src/my_robot_commander` | 可复用运动/碰撞场景库与抓放执行器 |
| `src/my_robot_moveit_pkg` | MoveIt 规划、运动学与控制器配置 |
| [`script/`](script/README.md) | 简短的 Shell 运行入口及运行说明 |
| [`test/`](test/README.md) | Python 分阶段测试、共用工具及 C++ 运动探针 |
| `docs/` | 本地沟通与开发详细报告，不代替目录使用说明 |

感知通过 `/perception/objects` 提供物体中心；任务节点以 Action 调用抓放执行器。执行器复用运动与场景库，经 MoveIt 和 ros2_control 驱动仿真。场景、采样和验收代码位于 `test/`，业务节点位于 `src/`。

## 安装与构建

先配置 [ROS 2 Humble 软件源](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)，再安装依赖：

```bash
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-yaml python3-opencv libopencv-dev libopencv-contrib-dev \
  ros-humble-desktop ros-humble-moveit ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-message-filters \
  ros-humble-xacro ros-humble-tf2-geometry-msgs ros-humble-example-interfaces \
  ros-humble-rmw-fastrtps-cpp

cd ~/piper_control
source /opt/ros/humble/setup.bash
# rosdep 首次使用前执行 sudo rosdep init，再执行 rosdep update。
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

每个新终端均需加载 ROS 和工作区环境。脚本默认加载工作区的 `install/setup.bash`，与 MuJoCo 分支一致。修改 C++、包清单或安装规则后重新构建；使用 `--symlink-install` 时，已有 launch、模型和 YAML 文件的修改在下次启动时读取。新增文件仍建议重新构建。
`requirement.txt` 仅约束 NumPy/PyYAML；优先使用系统包，保留 NumPy 1.x，不用 pip OpenCV 替换系统 OpenCV。

## 启动与分层

```text
gazebo.launch.py                    总入口：参数、顺序和依赖
├── ros2_control.launch.py          Gazebo、世界、机器人、控制器、MoveIt
│   └── my_robot_world/launch/spawn_cube.launch.py
│       └── spawn_object.launch.py  复用单个物体生成
├── motion_control.launch.py        抓放执行器与任务规划
└── perception.launch.py            颜色检测与深度定位
```

总入口默认启动完整世界和三个物块，等待任务，不会自行抓取：

```bash
ros2 launch my_robot_bringup gazebo.launch.py
# 无 GUI 的示例；无显示服务时相机渲染仍需要 xvfb-run
xvfb-run -a ros2 launch my_robot_bringup gazebo.launch.py \
  gui:=false use_rviz:=false show_window:=false
```

也可以在三个终端顺序启动 `ros2_control.launch.py`、`motion_control.launch.py`、`perception.launch.py`。它们各自有默认参数；第二、三层分别依赖前面已启动的服务与话题。终端都需 source 同一工作区，并设置相同的 ROS 域和通信配置。例如 `export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1`。本机使用 Cyclone DDS 且未自行配置时，另在各终端设置：

```bash
export CYCLONEDDS_URI="file://$PWD/src/my_robot_bringup/config/cyclonedds_local.xml"
```

`ros2_control.launch.py` 单独启动时也默认有完整世界、物块和 MoveIt。可以用 `spawn_objects:=false` 仅测机器人；用 `with_moveit:=false` 关闭 MoveIt。总入口中运动依赖 MoveIt，感知依赖运动，相应地使用 `with_motion:=false with_perception:=false` 关闭后续层。各入口可用 `--show-args` 查看参数。

一键脚本启动上述总入口，等待控制器和运动 Action 就绪后先移动到 `pose_1`，再按规划器配置发送分拣任务；规划器每次抓取前仍会更新观测：

```bash
./script/pick_three_colors.sh
./script/pick_three_colors.sh gui:=false use_rviz:=false show_window:=false
```

无显示服务时，后一条需在 `xvfb-run -a` 下运行。脚本任务结束后保留仿真，按 Ctrl+C 退出。脚本和旧的物理验收入口见 [script/README.md](script/README.md)。

单独启动 `gazebo.launch.py` 时，可在另一个终端发送分拣任务。该终端需要加载相同工作区，并使用与仿真一致的 ROS 域和 DDS 设置。例如，仿真使用本机 Cyclone DDS 时：

```bash
cd ~/piper_control
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/my_robot_bringup/config/cyclonedds_local.xml"

# 使用 task_planner.yaml 中的默认策略和顺序
ros2 action send_goal /sorting/sort_objects my_robot_interface/action/SortObjects \
  '{}' --feedback

# 仅为本次任务指定红 → 绿 → 蓝
ros2 action send_goal /sorting/sort_objects my_robot_interface/action/SortObjects \
  '{strategy: color_order, color_order: [1, 2, 3]}' --feedback
```

`{}` 使用 [task_planner.yaml](src/my_robot_planner/config/task_planner.yaml) 的 `default_strategy` 和 `default_color_order`。颜色编号为 1=红、2=绿、3=蓝；当前 `[3, 1, 2]` 表示蓝 → 红 → 绿。手动指定 `color_order` 只影响本次任务，不会修改 YAML。使用 `pick_three_colors.sh` 时脚本已经自动发送任务，无需再次手动发送。

Gazebo 启动提示端口占用时，可先执行 `./script/clear.sh --dry-run` 检查本工作区遗留进程；确认候选后执行 `./script/clear.sh`。脚本只处理符合范围的孤儿进程，不会停止仍由启动器管理的实例。更多说明见 [script/README.md](script/README.md)。

## 外部启动配置与覆盖顺序

[config/gazebo_launch.yaml](config/gazebo_launch.yaml) 提供一键脚本使用的启动默认值；编辑后下次运行生效，无需 `colcon build`。也可直接传给总入口：

```bash
ros2 launch my_robot_bringup gazebo.launch.py launch_config:=$PWD/config/gazebo_launch.yaml gui:=false
```

优先顺序为 **命令行 launch 参数 > 外部启动 YAML > launch 内默认值**；传入的节点参数覆盖对应节点 YAML，空值则沿用节点 YAML。路径在外部启动 YAML 中按该 YAML 所在目录解析。控制器、相机、感知、执行器和规划器各自的 YAML 仍用于各自的详细参数；外部 YAML 只有写出相应启动参数时才会改变它们。已经运行的节点不会自动刷新配置。

脚本会显式传入当前 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY`；Action 名称从节点实际参数读取，只有 launch 参数明确指定时才覆盖配置。只运行单独分层 launch 时，它们使用自身默认值，不读取总入口的 `launch_config`。

## 修改配置与接口

| 内容 | 文件 |
| --- | --- |
| 机器人描述、控制插件 | [my_robot_description](src/my_robot_description) |
| 世界、物块、相机设置 | [my_robot_world](src/my_robot_world) |
| HSV、深度范围、物块尺寸、显示窗口 | [perception.yaml](src/my_robot_perception/config/perception.yaml) |
| 策略、观察位、各颜色放置位置 | [task_planner.yaml](src/my_robot_planner/config/task_planner.yaml) |
| TCP、抓取距离、夹爪、碰撞盒 | [manipulation.yaml](src/my_robot_commander/config/manipulation.yaml) |
| 规划组、运动学、限位 | [my_robot_moveit_pkg/config](src/my_robot_moveit_pkg/config) |
| 机械臂与夹爪控制器 | [ros2_controllers.yaml](src/my_robot_description/config/ros2_controllers.yaml) |
| 消息与 Action 定义 | [my_robot_interface](src/my_robot_interface) |

主要接口：`/perception/objects` 发布物体中心；`/sorting/sort_objects` 执行整批分拣；`/manipulation/pick_place` 执行单次抓放；`/manipulation/move_to_named` 移动到命名姿态。

默认物块为 **1 cm**、坐标系为 `base_link`、TCP 偏移为 `[0, 0, 0.132] m`。尺寸或支撑面改变时，同步调整感知尺寸、放置高度、碰撞盒和抓放参数。位置单位 m，角度单位 rad。

碰撞场景默认包含地面，并在运动前同步检测到的物块；夹持时附着负载，释放后回到世界场景。桌面等固定障碍需在 `scene_boxes` 中配置，**不会自动导入 Gazebo 世界**。

取消会请求停止当前轨迹并保留夹爪状态。有附着物时拒绝新的抓取；确认处于安全位置后，可显式打开夹爪并解除附着：

```bash
ros2 service call /manipulation/release_held_object std_srvs/srv/Trigger '{}'
```

## C++ 复用

调用包在 `package.xml` 声明 `<depend>my_robot_commander</depend>`，并链接需要的库：

```cmake
find_package(my_robot_commander REQUIRED)
target_link_libraries(your_node
  my_robot_commander::motion_controller
  my_robot_commander::pick_place_scene)
```

公共头文件为 `my_robot_commander/motion_controller.hpp` 和 `my_robot_commander/pick_place_scene.hpp`，类位于 `my_robot_commander` 命名空间。同步调用期间需有 executor 处理节点回调；场景方法顺序调用，TF buffer 保持有效。运动库调用 `stop()` 后，新任务需先 `clearStop()`。

## 验证与限制

从已加载环境的工作区终端运行：

```bash
python3 test/planner_check.py                         # 任务与感知接口，域 94
python3 test/executor_check.py --mode headless        # 运动、碰撞、取消与物理抓放，域 93
python3 test/executor_check.py --stage physical       # 独立验证物理抓放
```

执行器测试使用 Gazebo 端口 11493，输出位于 `log/executor_validation/`；无显示器时自动使用 `xvfb-run`，需已安装 `xvfb`。物理验收要求抬升超过 **4 cm**、最终放置误差小于 **2 cm**。完整阶段列表、前提与复用方法见 [`test/README.md`](test/README.md)。

本次重构后固定场景三色闭环通过，放置误差约 **0.43～0.55 mm**；任务/感知接口 **16 项通过**。这些结果只代表固定仿真场景，不代表实机精度。

- 视觉当前每种颜色保留一个物块，要求已知尺寸、平放、顶部可见；RGB 与深度需对齐。
- Action 成功表示运动流程完成，尚未加入抓后视觉复核或未知障碍物自动建图。
- 软件取消不能替代硬件急停；实机需重新标定 TCP、相机外参和夹持参数。
- 正式 `test/` 与 `script/` 不再被整体忽略，待审查后可纳入 Git；`test/local/`、`log/` 和构建产物继续忽略，本地 `docs/` 保留现有忽略设置。
