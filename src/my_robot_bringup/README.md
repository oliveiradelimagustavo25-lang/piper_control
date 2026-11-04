# my_robot_bringup

`mujoco.launch.py` 是完整世界的正式入口，默认加载机器人、棋盘格地面、三色物块、标记板和腕部相机，并启动 ros2_control、MoveIt、`pick_place_executor` 及颜色与深度感知。`ros2_control.launch.py` 是共享的控制启动层，单独启动时只加载机器人本体，供交互控制阶段测试复用。`perception.launch.py` 可单独启动颜色检测和深度定位，输入来自已有 ROS 相机。

```bash
cd /home/ltt/piper_control
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch my_robot_bringup mujoco.launch.py
ros2 launch my_robot_bringup mujoco.launch.py --show-args
```

构建后重新加载 `install/setup.bash`，新终端也需要加载工作区环境。

完整世界启动后，可在同一 ROS 域的另一终端打开 MoveIt RViz：

```bash
source install/setup.bash
ros2 launch my_robot_moveit_pkg moveit_rviz.launch.py use_sim_time:=true
```

统一配置在 `my_robot_moveit_pkg/config/moveit.rviz`，固定坐标系为 `world`；MoveIt 的 MotionPlanning 显示默认开启，RobotModel 和 TF 可在显示面板按需勾选。`mujoco.launch.py` 本身不会自动打开 RViz，RViz 不会额外推进物理。当前没有对象识别 Action 时，RViz 会报告 `/recognize_objects` 不可用；MuJoCo 物块也尚未同步到 MoveIt 规划场景。

若系统更新后提示找不到旧版本 MoveIt 共享库，应重新配置并编译依赖它的工作区包；本次诊断与重建方法见[系统库检查报告](../../docs/2026-11-03_system_library_audit.md)。

## 窗口不显示时

`show_window:=true` 仅打开颜色检测调试窗口；MuJoCo 窗口由 `headless` 控制，默认为 `false`。颜色窗口需要收到相机图像后才会刷新。如果两个窗口都没有出现，先检查启动日志中控制器是否成功加载 `MujocoSystem`、相机插件是否启动，而不是只调整 `show_window`。

当日志显示 `Loading hardware 'webots'` 或找不到 `mujoco_ros2_control/MujocoSystem` 时，控制器接收到的不是本工作区使用 `mujoco_ros2_control/MujocoSystemInterface` 的描述。它与机器人状态发布器通过 `/robot_description` 通信；同一 ROS 域的其他发布器可能导致描述竞争。可先停止自己启动的旧仿真并检查 `./script/clear.sh --dry-run`；若仍有冲突，选择没有其他仿真的 ROS 域：

```bash
# 同一 ROS 域中的其他终端也须设置相同的 ROS_DOMAIN_ID。
export ROS_DOMAIN_ID=227
ros2 launch my_robot_bringup mujoco.launch.py show_window:=true
```

可用 `ros2 node list --no-daemon` 和 `ros2 topic info /robot_description --verbose --no-daemon` 检查当前域。域号 227 只是示例，请使用未被其他仿真占用的域；不要仅因节点名称相同就终止其他机器或其他项目的进程。图形窗口还要求当前终端能连接可用的图形会话。

## 模型与控制的连接

世界加载器把 `my_robot_world/basic/preview.xml` 与机器人 `mjcf/control.xml` 组合，将场景展开为临时 MJCF，并使用绝对资源路径。`my_robot_description/urdf/piper_control.urdf.xacro` 引入机器人 URDF 和独立的 `piper_ros_control.xacro`；其 `model_path` 指向该场景，插件加载的是真实完整世界。控制器配置位于 `my_robot_description/config/ros2_controllers.yaml`，bringup 通过参数选择它。临时文件在 launch 退出时清理，源码与安装目录均不生成运行文件。

ros2_control 插件是唯一的物理推进与 `/clock` 发布者；正式启动不运行 `test/mujoco_world.py`，也不创建第二个 Python 仿真实例。机器人状态发布、MoveIt 和执行端使用相同的 URDF 与仿真时间。

## 启动参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `headless` | `false` | 隐藏 MuJoCo 窗口，保留物理和 ROS 接口 |
| `with_perception` | `true` | 启动颜色检测和深度定位 |
| `with_camera` | 跟随 `with_perception` | 启用 MuJoCo 原生相机，发布 RGB、深度与内参 |
| `with_moveit` | `true` | 启动 `move_group` |
| `with_executor` | 跟随 `with_moveit` | 启动抓放、命名姿态 Action；需要 MoveIt |
| `sim_speed_factor` | `1.0` | 有限正数，目标仿真倍率 |
| `scene_file` | 世界包 `basic/preview.xml` | 地面、物体、标记板组成的场景 |
| `robot_model` | 描述包 `mjcf/control.xml` | 含执行器的机器人 MJCF |
| `camera_config` | 世界包 `config/camera.yaml` | 腕部相机配置 |
| `controllers_file` | description `config/ros2_controllers.yaml` | ros2_control 控制器、关节和容差 |
| `moveit_controllers_file` | MoveIt 包 `config/moveit_controllers.yaml` | MoveIt 到轨迹控制器的映射 |
| `planning_pipeline` | `ompl` | MoveIt 运动规划管线 |
| `manipulation_config` | commander `config/manipulation.yaml` | 执行器及 MotionController 参数 |

感知默认配置位于 `my_robot_perception/config/perception.yaml`。`perception_config` 可以指定替代文件；以下参数留空时使用该文件对应的值，主入口和独立感知入口都支持：

| 参数 | 作用 |
| --- | --- |
| `image_topic`、`depth_topic`、`camera_info_topic` | RGB、对齐深度和内参输入；主入口中的相机插件同步使用这些话题 |
| `labels_topic`、`debug_topic` | 颜色标签及调试图像话题；标签话题同时连接两个节点 |
| `points_topic`、`objects_topic` | 目标点云和物体信息输出话题 |
| `show_window`、`publish_debug` | 是否显示或发布颜色调试图像，传 `true` 或 `false` |

颜色 HSV 阈值、面积、深度过滤、物块尺寸、坐标系和同步容差等均在 `perception_config` 中调整。`perception.launch.py` 的 `use_sim_time` 默认 `false`；由 MuJoCo 主入口启动时固定为 `true`。

以下参数可直接覆盖执行配置，留空表示使用 `manipulation_config` 的值：

| 参数 | 对应功能 |
| --- | --- |
| `action_name`、`move_to_named_action` | 抓放和命名姿态 Action 名称 |
| `arm_group`、`gripper_group` | MoveIt 规划组 |
| `reference_frame`、`end_effector_link` | 目标参考坐标系与末端 link |
| `velocity_scaling`、`acceleration_scaling` | 运动速度和加速度倍率 |
| `planning_time`、`cartesian_step` | 规划时间和笛卡尔路径插值步长 |

TCP、抓取朝向、偏移、抬升距离、夹爪符号等完整参数通过执行 YAML 配置。优先级为配置默认值、显式 launch 覆盖；机器人描述和 `use_sim_time=true` 由启动层统一提供。模型执行器增益仍配置在机器人 MJCF 中。

```bash
# 完整世界，无窗口；覆盖执行端的常用运动参数
ros2 launch my_robot_bringup mujoco.launch.py headless:=true velocity_scaling:=0.15 planning_time:=3.0

# 完整世界和控制器，关闭 MoveIt、执行器及感知
ros2 launch my_robot_bringup mujoco.launch.py with_moveit:=false with_perception:=false

# 外部相机已发布 ROS 图像时，只运行两个感知节点
ros2 launch my_robot_bringup perception.launch.py

# 完整世界中覆盖图像和深度话题，相机与感知会使用相同名称
ros2 launch my_robot_bringup mujoco.launch.py image_topic:=/vision/rgb depth_topic:=/vision/depth

# 保留 MoveIt 接口，关闭抓放执行节点
ros2 launch my_robot_bringup mujoco.launch.py with_executor:=false

# 机器人本体的交互控制阶段测试
python3 test/robot_control.py
```

## 接口承接

| 层 | 对外接口或职责 |
| --- | --- |
| ros2_control | `/joint_states`、`/clock`、控制器管理服务 |
| 机械臂控制器 | `/arm_controller/follow_joint_trajectory` |
| 夹爪控制器 | `/gripper_controller/follow_joint_trajectory` |
| 相机与感知 | 默认 `/camera_sensor/image_raw`、`/camera_sensor/depth/image_raw`、`/camera_sensor/camera_info`；输出 `/perception/color_labels`、`/perception/object_points` 和 `/perception/objects` |
| MoveIt `move_group` | 运动规划与轨迹执行接口 |
| `pick_place_executor` | 默认 `/manipulation/pick_place`、`/manipulation/move_to_named` |
| `motion_controller.cpp` | 由执行器链接使用的运动库，承接 MoveIt 调用 |

当前启动颜色检测和深度定位，不启动 ArUco 检测或 `my_robot_planner` 任务策略。MuJoCo 原生相机插件与控制器共用同一物理世界，独立感知启动文件不会创建相机或推进物理；`with_camera:=false` 可改用外部 ROS 相机。相机即使在 `headless:=true` 时也需要可用的 OpenGL 渲染环境。MuJoCo 物块尚未自动同步为 MoveIt 碰撞物体，画面可见性与定位精度仍需校准，当前启动流程不代表抓取或分拣已验收。

当前环境中，部分 C++ 节点退出时仍可能出现析构异常，不能仅凭 launch 返回 0 判断子进程正常退出。详细测试和限制见[本轮开发报告](../../docs/2026-11-02_world_control_roles.md)。
