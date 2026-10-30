# Piper 三色物块分拣

基于 **Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 / MoveIt 2**，完成红、绿、蓝物块的视觉定位与抓放。目前已通过固定场景仿真；MuJoCo 后端和实机适配尚未完成。

流程：RGB-D 相机 → 颜色检测与深度定位 → 任务规划 → 抓放执行器 → MoveIt → ros2_control。

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
colcon build
source install/setup.bash
```

每个新终端均需加载 ROS 和工作区环境。修改 C++、launch 或 YAML 后，重新构建对应包。
`requirement.txt` 仅约束 NumPy/PyYAML；优先使用系统包，保留 NumPy 1.x，不用 pip OpenCV 替换系统 OpenCV。

## 一键分拣

构建后，从工作区根目录运行，脚本会启动仿真、生成物块并发送任务：

```bash
./test/run_sorting.sh gui color_order 3,1,2  # 蓝 → 红 → 绿
./test/run_sorting.sh headless nearest     # 每次选择离 TCP 最近的目标
```

默认 `gui / color_order / 1,2,3`，颜色编号为红=1、绿=2、蓝=3。GUI 模式任务结束后保留窗口，按 Ctrl+C 退出。
脚本使用 ROS 域 **95**、Gazebo 端口 **11495**；不要同时运行使用相同域和端口的测试。无显示器时需安装 `xvfb`。
日志、视觉截图和物理验收结果位于 `log/sorting/<时间>/`。

在其他终端查询时，加载脚本打印的 `ros_env.sh`，再执行 `ros2 node list --no-daemon`。

## 手动启动

以下两条命令分别在已加载环境的两个终端运行：

```bash
ros2 launch my_robot_bringup gazebo_robot_model.launch.py
ros2 launch my_robot_sim spawn_sorting.launch.py
```

启动后等待分拣 Action，不会自行抓取。在第三个已加载环境的终端发送任务：

```bash
export ROS_DOMAIN_ID=95
export ROS_LOCALHOST_ONLY=1
ros2 action send_goal /sorting/sort_objects my_robot_interface/action/SortObjects \
  '{strategy: color_order, color_order: [1, 2, 3]}' --feedback
```

主仿真、物块生成和任务 launch 默认域为 **95**，可通过 `ros_domain_id:=编号` 覆盖；所有相关节点应使用相同的域、RMW 和通信设置。独立模型预览默认域为 **96**：

```bash
ros2 launch demo_robot_pkg robot_piper_model.launch.xml
```

主仿真支持 `gui:=false use_rviz:=false`。颜色检测窗口单独由 `show_window` 控制，无显示器环境还需处理相机渲染和显示环境。一键脚本已处理无 `DISPLAY` 时的虚拟显示。

## 修改配置与接口

| 内容 | 文件 |
| --- | --- |
| 机器人、相机与 Gazebo 世界 | [demo_robot_pkg](src/demo_robot_pkg) |
| HSV、深度范围、物块尺寸、显示窗口 | [perception.yaml](src/my_robot_perception/config/perception.yaml) |
| 策略、观察位、各颜色放置位置 | [task_planner.yaml](src/my_robot_planner/config/task_planner.yaml) |
| TCP、抓取距离、夹爪、碰撞盒 | [manipulation.yaml](src/my_robot_commander/config/manipulation.yaml) |
| 规划组、运动学、限位 | [my_robot_moveit_pkg/config](src/my_robot_moveit_pkg/config) |
| 机械臂与夹爪控制器 | [ros2_controllers.yaml](src/demo_robot_pkg/config/ros2_controllers.yaml) |
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
```

执行器测试使用 Gazebo 端口 11493，输出位于 `log/executor_validation/`；无显示器时可用 `xvfb-run -a` 包装命令。物理验收要求抬升超过 **4 cm**、最终放置误差小于 **2 cm**。

最近一次三色闭环通过，放置误差约 **0.46～0.51 mm**；任务/感知接口 **16 项通过**。这些结果只代表固定仿真场景，不代表实机精度。

- 视觉当前每种颜色保留一个物块，要求已知尺寸、平放、顶部可见；RGB 与深度需对齐。
- Action 成功表示运动流程完成，尚未加入抓后视觉复核或未知障碍物自动建图。
- 软件取消不能替代硬件急停；实机需重新标定 TCP、相机外参和夹持参数。
- `test/` 和 `log/` 当前被忽略，其他机器上的仓库副本可能没有本地脚本或报告。
