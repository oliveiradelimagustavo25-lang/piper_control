# Piper 机械臂三色物块分拣工作区

目标是在 Gazebo 中完成红、绿、蓝物块的视觉定位与抓放，再将感知和任务逻辑迁移到 Piper_x 实机。

当前已验证机器人生成、控制器激活、三个物块生成和视觉数据话题；单物块抓放 Action 执行节点已实现，完整物理抓放尚待验证。定位话题有数据不代表定位精度已经验收。

## 1. 环境基线

以下版本来自当前开发机，不是完整的版本锁定文件：

| 组件 | 当前环境 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble，安装于 `/opt/ros/humble` |
| Python | 系统 Python 3.10 |
| 仿真 | Gazebo Classic 11 |
| 运动规划 | MoveIt 2，当前安装版本 2.5.9 |
| 视觉 | 系统 OpenCV 4.5.4，包含 contrib / ArUco |
| ROS 通信 | 当前仿真默认 Fast DDS、本机通信 |
| Python 数值与 YAML | Ubuntu 包提供 NumPy 1.21.5、PyYAML 5.4.1 |

本项目使用 Gazebo Classic 的 `gazebo_ros` 与 `gazebo_ros2_control`，并非新版 Gazebo 的 `ros_gz` 接口。

ROS 软件源配置请按照 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)进行；MoveIt 可参考[官方二进制安装说明](https://moveit.ai/install-moveit2/binary/)。

## 2. 安装依赖

### 系统与 ROS 依赖

以下命令假设已经配置好 Ubuntu 和 ROS 2 Humble 软件源。在新机器上执行，已有环境不必重复安装。

```bash
sudo apt update
sudo apt install \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-yaml python3-opencv \
  libopencv-dev libopencv-contrib-dev \
  ros-humble-desktop ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-message-filters ros-humble-xacro \
  ros-humble-tf2-geometry-msgs ros-humble-rmw-fastrtps-cpp \
  ros-humble-example-interfaces
```

初始化 rosdep，仅首次使用需要运行 `sudo rosdep init`；已经初始化时跳过该命令。

```bash
sudo rosdep init
rosdep update

cd /home/ltt/piper_control
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

当前部分包的 `package.xml` 仍未完整反映所有 CMake 依赖，因此保留上面的显式安装清单作为补充。

### requirement.txt 的作用

`requirement.txt` 只描述 Python 层的 NumPy / PyYAML 兼容范围，不能安装 ROS、MoveIt、Gazebo 或 C++ OpenCV。它们应通过 apt / rosdep 安装。

优先沿用上述系统 Python 包；已经满足版本范围时，无需再使用 pip。不要通过安装 pip 版 OpenCV 来替代 `libopencv-dev` 或 `cv_bridge`。

需要独立 Python 工具环境时，可以建立继承系统包的虚拟环境：

```bash
sudo apt install python3-venv
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirement.txt
```

不要使用 `sudo pip`。虚拟环境名称为 `.venv`，运行 colcon 时仍建议使用系统 Python 环境。

## 3. 当前文件结构

```text
src/
├── demo_robot_pkg/          # URDF、相机插件、网格、room.world
├── my_robot_model/         # red_cube / green_cube / blue_cube / aruco_model
├── my_robot_sim/
│   └── launch/
│       ├── spawn_object.launch.py
│       └── spawn_sorting.launch.py
├── my_robot_perception/
│   ├── src/
│   │   ├── aruco_detector.cpp
│   │   ├── color_detector.cpp
│   │   └── depth_localizer.cpp
│   └── config/perception.yaml
├── my_robot_interface/
│   ├── msg/Robotinterface.msg
│   └── action/PickPlace.action
├── my_robot_commander/
│   └── src/
│       ├── commander.cpp          # 原话题兼容入口
│       ├── motion_controller.cpp  # 可复用的 MoveIt 运动库
│       └── pick_place_executor.cpp  # 单物块抓放 Action 服务端
├── my_robot_moveit_pkg/     # SRDF、运动学、限位、控制器配置
└── my_robot_bringup/
    └── launch/gazebo_robot_model.launch.py
```

`build/`、`install/`、`log/` 是构建和运行产物，源码位于 `src/`。

## 4. 构建

在未加载其他项目 overlay 的新终端执行：

```bash
cd /home/ltt/piper_control
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

每个新终端均需加载 `install/setup.bash`。修改 C++ 后重新构建对应包；修改已安装的 launch 或 YAML 后，也应重新构建对应包以同步到 `install/`。

## 5. 启动仿真和物块

终端一：

```bash
cd /home/ltt/piper_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup gazebo_robot_model.launch.py
```

无界面启动：

```bash
ros2 launch my_robot_bringup gazebo_robot_model.launch.py gui:=false use_rviz:=false
```

主 launch 启动机器人、Gazebo、MoveIt、控制器、pick_place_executor，以及颜色和深度定位节点。使用 `use_pick_place:=false` 可以切回 commander 调试入口，两者不会同时启动。它不自动生成工作物块，也不启动 ArUco 检测。

终端二：

```bash
cd /home/ltt/piper_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_sim spawn_sorting.launch.py
```

单个物块或 ArUco：

```bash
ros2 launch my_robot_sim spawn_object.launch.py \
  model:=blue_cube entity:=blue_cube_1 x:=0.4 y:=0.1 z:=0.006

ros2 launch my_robot_sim spawn_object.launch.py \
  model:=aruco_model entity:=marker0 x:=0.5 y:=0.1 z:=0.25 yaw:=1.57
```

物块边长当前为 1 cm，质量暂设为 1 g。默认 `z=0.006` 对应地面附近；桌面场景需设置为桌面高度加物块半高及少量间隙。位置单位 m，角度单位 rad。同一 Gazebo 世界内实例名称不能重复；重复运行三色生成前应删除旧物块或重启仿真。

### 单独运行视觉节点

主 launch 已启动颜色与深度节点，不要再重复启动相同节点。独立调试时，从工作区根目录运行：

```bash
ros2 run my_robot_perception color_detector --ros-args \
  --params-file src/my_robot_perception/config/perception.yaml \
  -p use_sim_time:=true

ros2 run my_robot_perception depth_localizer --ros-args \
  --params-file src/my_robot_perception/config/perception.yaml \
  -p use_sim_time:=true -p target_frame:=base_link
```

上述两个节点需要在不同终端运行。ArUco 检测按需启动：

```bash
ros2 run my_robot_perception aruco_detector --ros-args \
  --params-file src/my_robot_perception/config/perception.yaml \
  -p use_sim_time:=true
```

独立运行节点或 ROS CLI 检查工具前，使用与仿真一致的通信环境：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
```

这三个设置影响当前终端。主 launch 与单物体 spawn 已默认给子进程设置 Fast DDS 和本机通信，ROS 域编号继承启动终端。多机实机通信时需要重新配置网络和 localhost 设置。

## 6. 视觉输入输出

```text
RGB → color_detector → 颜色标签图
                            ↓
对齐深度 + CameraInfo → depth_localizer → 带颜色标签的三维点
```

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/camera_sensor/image_raw` | `sensor_msgs/msg/Image` | 彩色图像 |
| `/camera_sensor/depth/image_raw` | `sensor_msgs/msg/Image` | 当前仿真深度图 |
| `/camera_sensor/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色相机内参 |
| `/perception/color_labels` | `sensor_msgs/msg/Image`，mono8 | 0 背景，1 红，2 绿，3 蓝 |
| `/perception/color_debug` | `sensor_msgs/msg/Image`，bgr8 | 检测框与中心标注 |
| `/perception/object_points` | `sensor_msgs/msg/PointCloud2` | 字段 x/y/z/label，每个有效连通区域一个代表点 |

定位要求深度与 RGB 对齐、尺寸及光学坐标系一致。深度支持 `32FC1` 米制和 `16UC1` 毫米制。标签图保留 RGB 的采集时间，定位节点按时间匹配深度。

输出是可见表面的估计点，不是完整表面点云、物块几何中心或夹爪目标。当前不输出物体朝向或稳定跟踪编号；同色相邻物体也可能合并为一个连通区域。输出为空或数据过期时，后续任务节点必须停止沿用旧目标。

`target_frame` 默认空字符串，即相机光学坐标系；设置为 `base_link` 时，需要对应采集时间的 TF。HSV、面积和深度阈值在 `perception.yaml` 中设置，目前仅在启动时读取。

## 7. 抓放 Action 与后续开发

`PickPlace.action` 对应服务端为 `/manipulation/pick_place`：

| 部分 | 字段 |
| --- | --- |
| Goal | object_id、object_pose、object_dimensions、place_pose |
| Result | success、message |
| Feedback | stage、message |

反馈阶段为 PREPARING、PICKING、TRANSFERRING、PLACING。输入位姿描述物块几何中心，尺寸使用物块本体坐标轴，执行节点负责计算夹爪 TCP 位姿。

```bash
ros2 interface show my_robot_interface/action/PickPlace
```

下一步依次实现：

1. 验证视觉坐标与 Gazebo 真值的误差，确定表面点到物体中心的换算方式。
2. 已新增 `my_robot_commander/src/pick_place_executor.cpp`；接下来验证实际抓放效果。
3. 在 `my_robot_commander/config/manipulation.yaml` 中标定 TCP、抓取朝向、接近距离和速度。
4. 先用手动目标完成单物块抓放，并验证接触、摩擦和夹持效果。
5. 增加桌面与物块碰撞盒，以及抓取后的附着/解除附着管理。
6. 新增任务节点，接收视觉结果，按颜色选择放置区并执行三物块任务。
7. 完成 Piper_x 驱动适配、相机外参和 TCP 标定，再进行实机验证。

旧的 `camera_move_arm.cpp` 位姿桥接节点已移除，后续由任务节点发送 PickPlace Action。当前 Action 服务端已实现，视觉任务调度节点尚未实现。

## 8. 已知配置和排查入口

- `kinematics.yaml` 仅为 arm 配置 KDL，夹爪通过关节目标控制。
- `sensors_3d.yaml` 已停用旧 Kinect / OctoMap 配置，深度定位仍独立运行；当前未实现未知障碍物自动建图。
- 若出现等待 `/robot_description` 或 `/spawn_entity`，先确认 Gazebo 仍在运行，以及各进程的 RMW、ROS 域和本机通信配置一致。
- Gazebo 中看到物体不代表 MoveIt 自动拥有其碰撞模型；需要通过规划场景接口显式添加。
- 相机是否看到物块取决于视场与观察姿态。仅收到空定位点云不能算定位成功。
- 旧源码移除后，增量安装可能留下旧可执行文件；以当前源码和 CMake 安装目标为准。

通信环境一致的终端中可检查：

```bash
ros2 control list_controllers
ros2 topic info /robot_description
ros2 service list
ros2 topic hz /perception/color_labels
ros2 topic echo /perception/object_points --once
```

2026-10-26 的无界面测试验证了 Piper 和三物块生成、三个控制器 active、commander 就绪及各视觉话题收到数据。没有验证抓取动作、定位精度或 GUI 显示。若本地保留日志，可查看 `log/sim_smoke_20261026/report.json`；`log/` 被 Git 忽略，不保证克隆仓库后存在。

## 9. 复用 MotionController

公共头文件位于 `my_robot_commander/motion_controller.hpp`。这是普通 C++ 类，不是独立节点；实现位于已导出的共享库，其他包需要声明依赖并链接，不能只复制头文件。

在调用包的 `package.xml` 添加：

```xml
<depend>my_robot_commander</depend>
```

在其 `CMakeLists.txt` 添加（将 `your_node` 替换为实际目标）：

```cmake
find_package(my_robot_commander REQUIRED)
target_link_libraries(your_node my_robot_commander::motion_controller)
```

代码示例（使用一个已传入 robot_description / SRDF / kinematics 参数且由 executor 持续处理回调的 node）：

```cpp
#include <my_robot_commander/motion_controller.hpp>

my_robot_commander::MotionController::Options options;
options.reference_frame = "base_link";
options.velocity_scaling = 0.2;
my_robot_commander::MotionController motion(node, options);

bool success = motion.moveToNamedTarget("home");
// 也可调用 moveToJoints、moveToPose、openGripper、closeGripper。
```

这些方法同步等待规划与执行，并返回 bool。执行期间必须有 executor 处理 ROS 回调；不要在唯一的 executor 线程中阻塞所有 MoveIt 回调。当前 commander 使用后台多线程 executor，并保留 name_target、joint_cmd、pose_command、open_gripper 四个话题。

默认组名为 arm / gripper，开合命名目标为 open / close；可通过 Options 适配其他具有 MoveIt 配置的机器人。更换机器人仍需相应 URDF、SRDF、运动学和驱动配置。直线 moveToPose 要求输入已转换到 Options.reference_frame，且仅在完整路径计算成功后执行。

true 表示轨迹执行成功，不证明已夹住物块。此库暂未提供立即中断或急停接口；Action 执行器目前只在当前动作结束后响应取消。

## 10. 单物块抓放执行器

默认仿真 launch 已加载 `config/manipulation.yaml`，Action 名称为 `/manipulation/pick_place`。它只在收到目标后开始运动，不自动订阅物体定位点或执行三色调度。

关键参数：

| 参数 | 含义 |
| --- | --- |
| tool_to_tcp | link6 到夹持中心的平移，在 link6 坐标系表达；当前 0.132 m 为网格推算初值，物理夹持仍待调试 |
| grasp_rpy | 固定抓取朝向，单位 rad，当前向下抓取 |
| grasp_offset | 物块中心到夹持中心的额外平移，在 reference_frame 中表达 |
| approach_distance | 预抓取点高于抓取点的距离 |
| lift_distance / transport_height | 最小抬升距离 / TCP 搬运高度下限 |
| release_clearance / retreat_distance | 放置时的抬高余量 / 松开后的撤离距离 |
| gripper_open_width / gripper_compression | 张开宽度 / 相对物块投影宽度的闭合余量 |
| gripper_joint_signs | gripper 组变量顺序对应的方向；当前 joint7=+q、joint8=-q |
| velocity_scaling / acceleration_scaling | 运动速度与加速度缩放 |
| max_target_age | 非零时间戳观测的最大年龄；零时间戳可用于手动静态目标 |

位置与距离单位为 m。Piper 的两个手指暂按开口宽度=2*q 计算，实际接触宽度和 TCP 需要通过模型及实物标定确认。夹紧指令执行完成不等于已验证抓住物体。

执行顺序为：张开 → 预抓取 → 直线下降 → 夹紧 → 直线抬升 → 搬运 → 直线下降 → 松开 → 撤离。任何一步失败都会终止后续步骤。取消请求在当前动作结束后生效，保持当时夹爪状态；不是立即停止接口。

本版固定工具朝向，要求物体抓取前与放置后的朝向一致。放置位置由 Action 提供，颜色与放置区映射应放在后续任务节点配置中。

当前执行器使用 MoveIt 已有规划场景，不自动添加桌面、其他物块或搬运负载。碰撞盒和附着管理仍需后续补齐；成功结果仅表示运动序列完成，实际放置应由视觉验证。

在仅仿真环境中，校准 TCP 并确认目标位置后，可用下面的手动目标测试（命令会触发运动）：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
ros2 action send_goal /manipulation/pick_place my_robot_interface/action/PickPlace '
object_id: blue_cube
object_pose:
  header: {frame_id: base_link}
  pose:
    position: {x: 0.26, y: 0.04, z: 0.005}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
object_dimensions: {x: 0.01, y: 0.01, z: 0.01}
place_pose:
  header: {frame_id: base_link}
  pose:
    position: {x: 0.26, y: -0.04, z: 0.005}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
' --feedback
```

该示例要求相应蓝色物块实际位于指定抓取位置，放置区域无其他物块。不要将深度表面点直接作为 object_pose 的中心位置。

无 GUI 的运动接口及真实物块抓放检测方法见 [validation/README.md](validation/README.md)。测试分别判断运动序列与物理抓放结果。

2026-10-27 执行器检测结果见 [validation/RESULTS.md](validation/RESULTS.md)：运动接口和 Action 流程通过，实际物块夹取未通过，需继续调试物理接触。
