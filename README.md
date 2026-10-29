# Piper 机械臂三色物块分拣工作区

目标是在 Gazebo 中完成红、绿、蓝物块的视觉定位与抓放，再将感知和任务逻辑迁移到 Piper_x 实机。

当前已验证单物块仿真抓放，并在固定场景完成了一次三色视觉分拣闭环：观察位、颜色分割、深度中心定位、任务排序和实际抓放。任务逻辑还通过模拟执行端接口测试；多场景成功率与实机精度仍需验收。

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
| ROS 通信 | RMW 继承终端配置，当前终端为 Cyclone DDS；仿真默认本机通信 |
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
│   ├── msg/                    # Robotinterface / DetectedObject / DetectedObjectArray
│   └── action/                 # PickPlace / MoveToNamed / SortObjects
├── my_robot_planner/
│   ├── src/task_planner.cpp
│   ├── config/task_planner.yaml
│   └── launch/task_planner.launch.py
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

主 launch 启动机器人、Gazebo、MoveIt、控制器、pick_place_executor、task_planner，以及颜色和深度定位节点。规划器收到分拣 Action 后才开始工作。使用 `use_task_planner:=false` 可停用任务节点；`use_pick_place:=false` 切回 commander 调试入口，并默认停用任务节点。它不自动生成工作物块，也不启动 ArUco 检测。

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
# RMW_IMPLEMENTATION 沿用终端配置，项目不再强制切换。
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
```

主 launch 与单物体 spawn 继承终端的 RMW 和 ROS 域编号，不再强制使用 Fast DDS；默认给子进程设置本机通信。多机实机通信时需要重新配置网络和 localhost 设置。

## 6. 视觉输入输出

```text
RGB → color_detector → 颜色标签图
                            ↓
对齐深度 + CameraInfo → depth_localizer → 表面调试点 / 物体中心列表
                                                 ↓
                                            task_planner
                                                 ↓ PickPlace.action
                                         pick_place_executor
```

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/camera_sensor/image_raw` | `sensor_msgs/msg/Image` | 彩色图像 |
| `/camera_sensor/depth/image_raw` | `sensor_msgs/msg/Image` | 当前仿真深度图 |
| `/camera_sensor/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色相机内参 |
| `/perception/color_labels` | `sensor_msgs/msg/Image`，mono8 | 0 背景，1 红，2 绿，3 蓝 |
| `/perception/color_debug` | `sensor_msgs/msg/Image`，bgr8 | 检测框与中心标注 |
| `/perception/object_points` | `sensor_msgs/msg/PointCloud2` | 字段 x/y/z/label，每个有效连通区域一个代表点 |
| `/perception/objects` | `my_robot_interface/msg/DetectedObjectArray` | 共用采集时间、坐标系的物体中心、尺寸、颜色、ID 列表 |

定位要求深度与 RGB 对齐、尺寸及光学坐标系一致。深度支持 `32FC1` 米制和 `16UC1` 毫米制。标签图保留 RGB 的采集时间，定位节点按时间匹配深度。

`object_points` 保留可见表面代表点，用于调试。`objects` 新增物体中心：将区域深度像素变换到 Z 轴向上的 `objects_frame`，选取上部表面点，以 XY 范围中点和“上表面高度减半高”估计中心。尺寸来自 `object_dimensions`，当前默认 1 cm。此方法要求物块平放、轴向对齐且顶部可见，不估计任意姿态。

第一版每种颜色保留最大的有效连通区域，ID 固定为 red_cube / green_cube / blue_cube，适用于三色各一个物块。同色多物块的稳定跟踪需要后续扩展。无有效目标时发布空列表；对象坐标转换不可用时也发布空列表，原调试点云仍可输出。

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

单物块执行器、物体消息与任务调度已实现。接下来应扩大场景验证视觉定位误差与观察姿态，增加桌面/物块碰撞盒及附着管理，再适配 Piper_x 驱动、相机外参和实机 TCP。分拣 Action 的用法见第 13 节。

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
| tool_to_tcp | link6 到夹持中心的平移，在 link6 坐标系表达；当前 0.132 m 已通过 1 cm / 3 cm 物块仿真抓放；实机需标定 |
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
# RMW_IMPLEMENTATION 沿用终端配置，项目不再强制切换。
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

## 11. 执行器验证

测试使用独立 ROS 域 `93`、Gazebo 端口 `11493` 和本机 Fast DDS。会实际移动 Gazebo 中的机械臂、夹爪和物块，不连接实机。运行前确认该域和端口未被其他程序使用。

### 构建和运行

从工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_commander my_robot_bringup
source install/setup.bash
python3 -u test/executor_check.py --mode headless

# 使用 Gazebo 和 RViz 窗口
python3 -u test/executor_check.py --mode gui
```

Python 依赖沿用工作区的 ROS、NumPy / PyYAML 系统环境。测试会临时复制世界文件并添加 Gazebo 状态查询插件，源世界文件不变。

### 检查内容

单脚本会自动生成并编译临时 C++ 调用器，直接链接生产环境的 MotionController 库；不创建 ROS 包、不需要手工配置 CMake。检查：

- 错误关节数量、非有限关节值、无效命名目标、零四元数、错误笛卡尔坐标系、夹爪越界目标。
- 夹爪开合、命名开合目标、自定义夹爪关节目标，并核对实际关节状态。
- 机械臂命名姿态、关节目标、PoseStamped 位姿、XYZ/RPY 位姿、笛卡尔运动，并核对关节或末端 TF。

关节误差门槛为机械臂 0.02 rad、夹爪 0.002 m；位姿接口的位置和角度门槛为 0.005 m / 0.03 rad。实际误差写入 `motion.log`，逐项结果写入 `motion.json`。

`executor_check.py` 检查：

- Action 服务和三个 active 控制器。
- 无效目标拒绝、错误时间戳终止、忙碌时拒绝新目标、步间取消。
- 空载抓放序列的成功结果和四个反馈阶段。
- 真实物块抓放：通过 Gazebo 状态服务采样物块位置，区分运动流程成功与物理抓放成功。

物理验收要求抬升超过 4 cm，最终三维放置误差小于 2 cm。默认测试物块为 3 cm 立方体，手指和物块的摩擦系数均显式设为 1.0。尺寸与摩擦通过脚本参数设置，源 URDF/SDF 不变。物块质量、惯量按尺寸同步调整；也可用 `--cube-mass` 指定质量。测试失败仍会生成报告，并以非零退出码结束；不能只看 Action success。

### 输出与复测

默认输出位于 `log/executor_validation/<模式>_<时间>/`；可用 `--tag` 指定目录名：

| 文件 | 内容 |
| --- | --- |
| report.json | 汇总结果 |
| motion.json / motion.log | 运动接口逐项结果 / 执行日志 |
| bringup.log | 仿真和 PickPlace 日志 |
| object_trace.json / contact_trace.json | 物块位置轨迹 / 接触时间、碰撞体、接触深度和力 |
| manipulation.yaml / controllers.yaml / world.world | 本次实际使用的抓放、控制器参数和测试世界 |
| robot.urdf / cube.sdf | 本次尺寸、质量、摩擦设置后的测试模型 |

仅复测抓放、跳过已经通过的运动接口：

```bash
python3 -u test/executor_check.py --skip-motion --tag action_recheck
```

只复测物理夹持，可再加 `--skip-air`。例如比较尺寸和摩擦：

```bash
python3 -u test/executor_check.py --skip-motion --skip-air \
  --cube-size 0.03 --friction 1.0 --tag cube_3cm_mu1
python3 -u test/executor_check.py --skip-motion --skip-air \
  --cube-size 0.03 --friction 5.0 --tag cube_3cm_mu5
```

未写摩擦标签不代表摩擦为零：Gazebo 会采用默认参数。测试脚本显式设置两侧接触材质以便对照。

`--profile baseline` 可重现旧 TCP / 朝向 / 搬运高度及旧测试点；`--profile ground` 使用地面小物块诊断参数。默认 `current` 使用当前源码 YAML 和地面测试点。

`--tcp-z`、`--compression`、`--contact-layer` 只覆盖本次测试文件，不写回生产参数。报告须注明覆盖项，不能将实验参数下的通过结果视为默认配置通过。

`--mode headless` 不启动窗口；`--mode gui` 启动 Gazebo 客户端和 RViz，需有可用图形桌面。测试世界的默认相机已改为抓取区近景。报告在 PICKING / TRANSFERRING / PLACING 阶段记录实际夹爪关节及物块相对指尖的位置，并统计双指接触采样。

本套测试不验证相机分割、定位精度或实机安全功能。无显示服务器时 Gazebo 可能停用相机渲染，但关节控制和物理状态查询仍可用于执行器测试。

## 12. 执行器检测结果

日期：2026-10-28。环境：Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11，Fast DDS，独立 ROS 域 93 和 Gazebo 端口 11493。

### 问题与修复

旧配置的运动序列成功，但物块基本留在原位，放置误差约 80 mm。增大物块、提高摩擦或单独缩小闭合开口都未解决问题。新增的接触传感器确认双指已经碰到物块。

旧 Gazebo `position` 接口在写入时直接调用 `SetPosition` 并将关节速度设为零；`position_pid` 则根据位置误差施加力，见 [gazebo_ros2_control 0.4.10 实现](https://github.com/ros-controls/gazebo_ros2_control/blob/0.4.10/gazebo_ros2_control/src/gazebo_system.cpp#L786-L806)。对照实验表明，修正驱动方式后，物块能够随夹爪抬升、搬运并放下。

当前仿真配置：

- `gazebo_piper.ros2_control.urdf`：机械臂使用 velocity 命令接口；双指使用 position_pid，P=1000、I=0、D=5。Gazebo 对外仍导出夹爪 position 接口。
- `ros2_controllers.yaml`：机械臂轨迹控制器按位置误差输出速度，P=20；当前版本未配置的速度前馈系数默认为 0，因此没有叠加轨迹速度。控制周期为 1000 Hz。
- `manipulation.yaml` 与 C++ 默认值：总闭合余量从 0.4 mm 调为 2 mm。例如 30 mm 物块的目标开口为 28 mm；接触后实际开口停在物块宽度附近，由 PID 形成夹持力。
- 双指 mu / mu2 显式设为 1.0。TCP 使用 `[0, 0, 0.132] m`，世界接触参数采用 `room.world` 默认值。

这些调整作用于当前 Gazebo 驱动配置。Piper_x 实机仍需使用对应驱动并标定 TCP、夹持力。

### 默认配置回归

以下正式复测均使用源码默认 TCP、闭合余量和接触参数：

| 检查 | 结果 |
| --- | --- |
| MotionController 输入与实际运动 | 15 / 15 通过 |
| Action 无效目标、错误时间戳、忙碌拒绝、步间取消 | 4 / 4 通过 |
| 空载抓放序列及四阶段反馈 | 通过 |
| Headless，3 cm / 27 g 物块 | 抬升 64.27 mm，放置误差 0.069 mm，通过 |
| GUI，1 cm / 1 g 原始尺寸物块 | 抬升 75.08 mm，放置误差 0.107 mm，通过 |

MotionController 回归中，机械臂最大关节误差约 0.00309 rad，夹爪约 0.0000963 m，位姿最大位置误差约 0.443 mm，均在测试门槛内。物块位移来自 Gazebo 实际物理状态；误差数值是仿真测量，不代表实机精度。

日志和报告：

- `log/executor_validation/grasp_fixed_full/`：完整 headless 回归，包含 motion.json、report.json 和轨迹。
- `log/executor_validation/grasp_fixed_gui_1cm/`：GUI 抓放，包含 controllers.yaml、接触记录和实际夹爪宽度。
- `log/executor_validation/diagnose_contacts/`：旧驱动下的接触诊断，双指有接触但未抬起物块。

GUI 复测在虚拟显示器中启动 Gazebo 和 RViz。抓放完成后的 SIGINT 清理阶段，RViz 出现退出崩溃日志；该问题独立于已经通过的抓放过程。

### 验收范围

当前通过的是指定地面位置的单物块抓放，物理验收为抬升超过 4 cm、最终放置误差小于 2 cm。Action success 仍表示运动序列完成，真实抓放是否通过由测试脚本的 physical 结果判断。

任务节点及接口现已实现，见第 13 节。此处物理抓放报告仍只覆盖单物块执行器；没有验证视觉三物块闭环与实机控制。

## 13. 任务规划器与分拣接口

`task_planner.cpp` 负责选择目标和组织执行顺序，MoveIt 继续负责运动轨迹规划。任务节点不依赖 Gazebo 真值，也不直接控制关节。

```text
color_detector → depth_localizer → /perception/objects
                                         ↓
客户端 → /sorting/sort_objects → task_planner
                                         ↓ /manipulation/pick_place
                                pick_place_executor → MotionController → MoveIt
```

接口文件：

| 文件 | 字段与作用 |
| --- | --- |
| DetectedObject.msg | object_id、color、pose、dimensions；pose 是物体中心 |
| DetectedObjectArray.msg | header + objects；同一帧共用采集时间及坐标系 |
| SortObjects.action Goal | strategy、color_order |
| SortObjects.action Result | success、message |
| SortObjects.action Feedback | current_object_id、completed_count |

主仿真 launch 默认启动规划器，收到 Action 才执行。已有仿真也可以单独启动任务节点，两种启动方式选择一种：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# RMW_IMPLEMENTATION 沿用终端配置，项目不再强制切换。
export ROS_LOCALHOST_ONLY=1
ros2 launch my_robot_planner task_planner.launch.py
# 实机或独立接口测试使用 use_sim_time:=false。
```

在 `src/my_robot_planner/config/task_planner.yaml` 配置放置中心位置。默认位置使用 base_link，Z=0.005 m 对应地面上的 1 cm 物块；更换尺寸、支撑高度时应同步修改 `perception.yaml` 的 object_dimensions 和放置位置 Z。距离策略使用的 tool_to_tcp 需与 manipulation.yaml 标定一致。

确认相机能看见工作区、定位列表稳定，并在启动终端使用相同的 RMW / ROS 域设置：

```bash
# RMW_IMPLEMENTATION 沿用终端配置，项目不再强制切换。
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /perception/objects --once

# 按蓝、红、绿顺序抓取；该命令会触发机械臂运动。
ros2 action send_goal /sorting/sort_objects my_robot_interface/action/SortObjects \
  '{strategy: color_order, color_order: [3, 1, 2]}' --feedback

# 在红、绿、蓝候选中，每次选择距离当前 TCP 最近的物块。
ros2 action send_goal /sorting/sort_objects my_robot_interface/action/SortObjects \
  '{strategy: nearest, color_order: [1, 2, 3]}' --feedback
```

strategy 为空时使用 default_strategy，color_order 为空时使用 default_color_order。color_order 同时决定参与任务的颜色；例如 `[1]` 只抓红色。列表不允许重复颜色。收到任务后先等待所需颜色齐全，再固定本批 object_id；执行前从最新观测刷新位姿，已完成 ID 不会在本批重复加入。

color_order 策略在前一种颜色暂时丢失时等待它，保持指定顺序。nearest 每次重新查询当前 TCP 并计算三维距离，不使用初始位置一次性排完整批次。扩展策略主要修改 validPolicy / selectTarget；批次管理与 PickPlace 调用可继续复用。

任务等待与执行使用异步回调，不阻塞 ROS executor。并行任务被拒绝；PickPlace 拒绝/失败会终止本批后续目标。目标或 TF 暂不可用时等待，超过 observation_timeout 后终止；观测采集时间及接收时间均受 max_observation_age 约束。零采集时间允许手工输入，但仍检查接收时间。

分拣取消会转发给当前 PickPlace，等待它返回后结束任务，不再派发下一物块。执行器仍在当前同步动作结束后处理取消。上层客户端可调用 SortObjects goal handle 的 cancel_goal_async()；这不代表立即停止运动。

腕部相机在初始位和搬运后可能看不到物块。规划器现在在每次抓取前，通过 /manipulation/move_to_named 请求执行器到 observation_target（默认 pose_1），等待 observation_settle_time 后的新图像再选择目标；不会使用移动前的观测。固定外部相机可将 observation_target 设为空字符串。任务 success 代表本批运动流程完成，尚未加入抓后视觉复核或自动碰撞场景管理。

### 接口与策略测试

工作区根目录执行：

```bash
python3 test/planner_check.py
# 只复测深度中心转换：
python3 test/planner_check.py PlannerChecks.test_depth_surface_to_center
```

该脚本使用独立 ROS 域 94、模拟 PickPlace 执行端与合成深度图，不连接机械臂，也不创建 ROS 包。测试输出位于 `log/planner_validation/<时间>/`。测试中的 0.60 m 放置点仅用于验证距离重算，不发送给实际机械臂。

2026-10-28：13 项测试全部通过，覆盖颜色顺序/默认参数、距离重算、位姿刷新、忙碌拒绝和取消转发、失败/拒绝停止、错误策略、缺失/过期观测、顺序保持、物体 TF 转换、重复 ID，以及 32FC1 / 16UC1 深度的物体中心转换、最大同色区域选择和空列表输出。

集成新节点后，原有 1 cm 物块 headless 抓放回归也通过：抬升 75.10 mm、放置误差 0.020 mm；日志位于 `log/executor_validation/task_planner_bringup_regression/`。此回归通过手动单物体目标检查执行器兼容性，没有发送三色分拣任务。

完整三色视觉分拣的实际成功率、相机噪声下定位误差、观察姿态和碰撞场景仍需后续验收。


## 14. 一键视觉分拣与观察位

脚本位于 `test/run_sorting.sh`。先构建一次，再从工作区根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to my_robot_bringup my_robot_sim
./test/run_sorting.sh gui color_order 3,1,2
# 无窗口运行；无 DISPLAY 时使用 xvfb-run 保留深度相机渲染。
./test/run_sorting.sh headless nearest
```

默认参数为 gui / color_order / 1,2,3。GUI 模式在任务结束后保留窗口，Ctrl+C 清理本脚本启动的仿真。测试独占 ROS 域 95 和 Gazebo 端口 11495；输出位于 `log/sorting/<时间>/`。不需要另外启动原 Gazebo launch 或 spawn launch。

脚本继承终端的 RMW，使用本机通信和域 95；用 ./ 或 bash 执行不会修改父终端。要在另一个终端查询本次仿真，先 source 脚本打印的 `log/sorting/<时间>/ros_env.sh`，再运行 `ros2 node list --no-daemon`。环境文件记录实际继承的 RMW；未设置时保持 ROS 默认实现。只设置 RMW 而保留域 0，无法查询域 95 的节点。脚本结束后仿真节点会被清理，此时查到空列表是正常结果。

普通 `ros2 node list` 使用 CLI daemon。更换 RMW 后，已启动 daemon 不会自动切换实现；应在对应 ROS 域停止旧 daemon，再以统一环境启动查询，见 [ROS 2 官方排查说明](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Working-with-multiple-RMW-implementations.rst#ensuring-use-of-a-particular-rmw-implementation)。2026-10-29 的本机诊断还发现默认 Cyclone/非 localhost 发现会卡住，而 localhost 直接发现与 Fast DDS 直接发现可正常返回；这不意味着所有 Cyclone 安装都有同样问题。

脚本生成三色 1 cm 物块：X=0.30 m，Y=-0.06/0/0.06 m；测试放置区 X=0.26 m，Y 保持三处间隔。测试参数副本位于日志目录，源模型与默认放置位置不被覆盖。`bringup.log` 记录节点日志，`task.log` 记录反馈，`vision_latest.png` 保存最近一次视觉调试图，`report.json` 区分任务返回结果与物块实际抬升/放置结果。Gazebo 真值查询仅用于测试验收，规划器仍使用视觉结果。

### 信息传递顺序

1. 脚本启动 Gazebo、MoveIt、控制器、感知和任务节点，等待控制器 active，再通过 spawn_entity 生成物块。
2. 脚本向 `/sorting/sort_objects` 发送 SortObjects：策略和颜色列表。
3. task_planner 向 `/manipulation/move_to_named` 发送 MoveToNamed：`target=pose_1`。pick_place_executor 通过 MotionController 和 MoveIt 移到观察位。
4. 相机发布 RGB、深度和 CameraInfo；color_detector 发布 `/perception/color_labels`。depth_localizer 将标签与深度按时间匹配，结合内参和采集时刻 TF，发布 `/perception/objects` 中的物体中心列表。
5. task_planner 等待到位且稳定后的新图像，根据策略选中物体，并把 object_pose、dimensions 和 YAML 对应的 place_pose 组成 PickPlace，发给 `/manipulation/pick_place`。
6. pick_place_executor 按接近、夹紧、抬升、搬运、释放的顺序调用 MotionController；MoveIt 规划轨迹并通过 FollowJointTrajectory 发给 arm_controller / gripper_controller。
7. ros2_control 驱动 Gazebo 关节；joint_states 经 robot_state_publisher 更新 TF，供 MoveIt 状态反馈和相机坐标转换使用。
8. PickPlace 返回后，规划器更新完成计数；还有目标则返回观察位重新定位。SortObjects 将当前目标与完成数反馈给脚本，全部结束后返回结果。

### 为什么需要观察位

当前相机安装在腕部。按模型计算，home 位的光轴偏向前方，当前地面物块落在垂直视场之外；pose_1 的相机光轴接近向下，可覆盖脚本工作区。观察位名称配置在 task_planner.yaml，具体关节角在 piper.srdf；运动由执行器统一完成，与 PickPlace 共用忙碌锁，避免同时控制机械臂。

MoveToNamed 的取消在当前同步运动结束后生效。观察运动失败或被拒绝时，不开始抓取。需要更换场景时，应重新校准观察姿态、视场、遮挡与相机最小测距。

本次还修复了两个视觉链路问题：定位允许等待 0.1 s 以接收采集时刻对应的 TF；仿真相机显式设置 min_depth=0.05、max_depth=3.0，与定位范围一致。相机插件默认 min_depth=0.4 m 会过滤当前近距离物块，见 [Gazebo 相机插件实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_camera.cpp#L432-L435)。这些是仿真设置，实机观察距离仍应符合实际深度相机能力。

2026-10-29：新增观察位失败、取消和旧图像拒绝测试，任务与感知检查共 16 项通过。测试脚本移动到 test/ 后，已同步修正其工作区路径定位。


### 本次三色视觉分拣验证

2026-10-29 使用 `./test/run_sorting.sh headless color_order 1,2,3`，真实相机图像与深度定位驱动抓放，三个物块全部通过物理验收：

| 物块 | 最大抬升 | 最终放置误差 |
| --- | --- | --- |
| 红色 | 75.97 mm | 0.530 mm |
| 绿色 | 75.56 mm | 0.489 mm |
| 蓝色 | 76.07 mm | 0.506 mm |

报告位于 `log/sorting/20261029_171641/report.json`。该结果覆盖脚本定义的三色 1 cm 地面场景，是一次仿真闭环通过记录，不代表任意摆放、任意相机或实机精度。16 项任务接口测试通过，TF 等待调整后的深度中心转换复测也通过。
