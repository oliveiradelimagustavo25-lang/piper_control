# Piper 机械臂三色物块分拣工作区

目标是在 Gazebo 中完成红、绿、蓝物块的视觉定位与抓放，再将感知和任务逻辑迁移到 Piper_x 实机。

当前已验证机器人生成、控制器激活、三个物块生成和视觉数据话题；自动抓放执行节点尚未实现。定位话题有数据不代表定位精度已经验收。

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
│       ├── commander.cpp
│       └── camera_move_arm.cpp
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

主 launch 启动机器人、Gazebo、MoveIt、控制器、commander，以及颜色和深度定位节点。它不自动生成工作物块，也不启动 ArUco 检测。

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

`PickPlace.action` 已生成，目前尚无对应执行服务端：

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
2. 新增 `my_robot_commander/src/pick_place_executor.cpp`，实现 Action 服务端。
3. 新增 `my_robot_commander/config/manipulation.yaml`，配置 TCP、抓取朝向、接近距离和速度。
4. 先用手动目标完成单物块抓放，并验证接触、摩擦和夹持效果。
5. 增加桌面与物块碰撞盒，以及抓取后的附着/解除附着管理。
6. 新增任务节点，接收视觉结果，按颜色选择放置区并执行三物块任务。
7. 完成 Piper_x 驱动适配、相机外参和 TCP 标定，再进行实机验证。

当前 `camera_move_arm.cpp` 仍是 ArUco 位姿桥接工具，新执行链路完成后再退出旧流程。

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
