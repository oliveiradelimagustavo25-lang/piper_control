# my_robot_bringup

`mujoco.launch.py` 是完整世界的总入口，负责按层组织启动。世界组合和统一机器人描述由 `ros2_control.launch.py` 完成，再传给 motion 层：

1. `ros2_control.launch.py`：MuJoCo、机器人状态、ros2_control 和 MoveIt；物理及 `/clock` 只有一个所有者。
2. `motion_control.launch.py`：抓放执行器 `pick_place_executor` 和任务规划器 `task_planner`；`motion_controller.cpp` 是执行器使用的库。
3. `perception.launch.py`：颜色分割和深度定位。原生相机插件属于仿真控制层。

总入口默认启动三层，但启动节点不会自动抓取；必须向分拣 Action 发送目标。直接启动 `ros2_control.launch.py` 默认也会加载完整世界、相机、控制器和 MoveIt，不会启动 motion 和 perception。机器人本体阶段测试显式使用 `robot_only:=true with_moveit:=false`。

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch my_robot_bringup mujoco.launch.py
ros2 launch my_robot_bringup mujoco.launch.py --show-args

# 只启动完整世界、控制器与 MoveIt，不启动任务和识别节点
ros2 launch my_robot_bringup ros2_control.launch.py

# 各分层的默认参数
ros2 launch my_robot_bringup ros2_control.launch.py --show-args
ros2 launch my_robot_bringup motion_control.launch.py --show-args
ros2 launch my_robot_bringup perception.launch.py --show-args
```

## 外部总配置与覆盖关系

脚本默认把工作区 [config/mujoco_launch.yaml](../../config/mujoco_launch.yaml) 作为 `launch_config` 传入总入口；修改该文件后重新运行脚本，不需要重新构建。也可以直接指定：

```bash
ros2 launch my_robot_bringup mujoco.launch.py launch_config:="$PWD/config/mujoco_launch.yaml"
ros2 launch my_robot_bringup mujoco.launch.py --show-args
```

直接运行 `mujoco.launch.py` 而不传 `launch_config` 时使用其自身默认值，不会自动读取脚本的外部文件。总配置是 launch 参数的平铺映射，支持 `--show-args` 中除 `launch_config` 本身之外的参数；未写入或写为 `null` 的参数使用原默认值。配置路径可为绝对路径或相对于总 YAML 的路径，写错参数名会明确报错。`--show-args` 显示可用参数及原始回退值，文件中的实际配置请直接查看 YAML。`ROS_DOMAIN_ID` 属于环境变量，通过脚本的 `--ros-domain-id` 或终端环境设置，不写入总配置。

常规参数的优先级是：**命令行显式参数 > 外部总 YAML 中同名参数 > 对应节点 YAML 的值 > 节点代码默认值**。launch 参数留空表示不覆盖节点 YAML。总入口还会主动同步跨节点接口：

- 相机 RGB、深度及 CameraInfo 的显式 launch 参数会覆盖感知 YAML 中的对应话题，并同时用于相机发布和感知订阅；留空则使用感知 YAML。
- 感知层启用时，规划器的 `objects_topic` 与深度节点输出一致；显式 launch 参数优先，否则使用感知 YAML 的值，会覆盖规划器 YAML 中的同名输入。关闭感知时则使用规划器 YAML，除非显式指定 `objects_topic`。
- 执行器的抓放和命名动作名称会同步给任务规划器；规划器 YAML 中相应名称因此会被执行器配置或 launch 参数覆盖。`pick_place_action` 和旧别名 `action_name` 同时指定且不一致时会报错。
- 为保持几何一致，规划器的 `reference_frame`、`end_effector_frame`、`tool_to_tcp` 由执行器配置（或相应 launch 参数）统一提供，可能覆盖规划器 YAML。三个分层由总入口启动时统一使用仿真时钟。

默认总 YAML 将下列配置文件指向工作区源码，因此修改这些 YAML 后重新启动脚本即可；总配置和节点 YAML 只在启动时读取，不会动态更新运行中的节点。

| 总配置路径参数 | 配置用途 | 实际读取位置 |
| --- | --- | --- |
| `camera_config` | 相机名称、视野、分辨率和发布频率 | `ros2_control.launch.py`、世界加载器 |
| `controllers_file` | 控制器、关节和容差 | `ros2_control.launch.py`、控制器管理器 |
| `moveit_controllers_file` | MoveIt 到轨迹控制器的映射 | 控制层和 motion 层的 MoveItConfigsBuilder |
| `manipulation_config` | 抓放执行、TCP、运动配置 | `motion_control.launch.py` |
| `planner_config` | 分拣策略、观察姿态、放置位置 | `motion_control.launch.py` |
| `perception_config` | 颜色、深度过滤、话题和物块尺寸 | 总入口、控制层的相机配置、`perception.launch.py` |

`MoveItConfigsBuilder` 自动加载的 `kinematics.yaml`、`joint_limits.yaml`、`sensors_3d.yaml` 等默认来自安装的 MoveIt 配置包；它们不在上述六条路径内。修改这些源码配置后仍需重新构建或使用适当的符号链接安装，修改 Python/C++、URDF 等资源同样不能一概认为不需要构建。直接单独启动各层时，如果没有传路径，默认使用安装目录内的配置。

## 层级与参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `with_moveit` | `true` | 启动 MoveIt；关闭后不能启用 motion |
| `with_motion` | 跟随 `with_executor` | 启动执行器和任务规划器 |
| `with_perception` | 跟随 `with_motion` | 启动颜色、深度节点；依赖 motion |
| `with_executor` | 跟随 `with_moveit` | 旧参数，作为 `with_motion` 的默认值 |
| `with_camera` | 跟随 `with_perception` | 发布 MuJoCo RGB、深度和内参 |
| `headless` | `false` | 隐藏 MuJoCo 窗口 |
| `sim_speed_factor` | `1.0` | 仿真速度倍率 |
| `scene_file`、`robot_model`、`camera_config` | 世界和描述包配置 | 世界、机器人模型及相机配置 |
| `controllers_file`、`moveit_controllers_file` | 各包配置 | ros2_control 和 MoveIt 控制器映射 |
| `planning_pipeline` | `ompl` | MoveIt 规划管线 |
| `manipulation_config`、`planner_config`、`perception_config` | 各包配置 | 执行、任务规划和视觉配置 |

独立 `ros2_control.launch.py` 默认使用世界包的 `basic/preview.xml`、相机配置和描述包的 `mjcf/control.xml`；其 `with_camera:=auto` 在完整世界发布 RGB-D，在 `robot_only:=true` 时关闭相机发布。可通过 `scene_file`、`model_path`、`camera_config`、`perception_config` 和相机话题参数覆盖默认值。`plugins_file` 可指定原生插件配置；`robot_description` 是可选的已展开描述。motion 的参数默认来自对应 YAML，perception 默认从 `perception.yaml` 取话题和阈值；单独启动 motion 需要已有控制和 MoveIt，单独启动 perception 需要已有图像输入。三个分层默认使用仿真时间；如果 perception 单独连接系统时间的外部相机，需要显式传入 `use_sim_time:=false`。

`with_motion:=true with_moveit:=false` 和 `with_perception:=true with_motion:=false` 会明确报错。层级约束适用于总入口；独立的 `perception.launch.py` 可连接已有相机测试。

执行端和任务规划器共享 `pick_place_action`、`move_to_named_action`；`sort_action` 指分拣 Action。旧 `action_name` 是 `pick_place_action` 的兼容别名，同时传入且不一致时会报错。`objects_topic` 同时配置深度定位输出和任务规划输入；关闭感知层时，未显式指定它则使用规划器 YAML 中的输入话题。RGB、深度及 CameraInfo 话题同时用于相机和感知节点。留空时使用对应 YAML；执行和规划节点的坐标系、末端链接与 TCP 从执行配置统一传入。`observation_target` 可覆盖规划器观察姿态。

`image_topic`、`depth_topic`、`camera_info_topic`、`labels_topic`、`debug_topic`、`points_topic`、`objects_topic`、`show_window` 和 `publish_debug` 可直接覆盖感知配置。运动常用覆盖包括 `arm_group`、`gripper_group`、`reference_frame`、`end_effector_link`、`velocity_scaling`、`acceleration_scaling`、`planning_time` 和 `cartesian_step`；详细颜色阈值、抓取距离与放置位置仍在对应 YAML。

```bash
# 完整世界及 MoveIt，不启动后两层
ros2 launch my_robot_bringup mujoco.launch.py with_motion:=false

# 启动 motion，不启动 perception
ros2 launch my_robot_bringup mujoco.launch.py with_perception:=false

# 全部启动并覆盖共同物体话题
ros2 launch my_robot_bringup mujoco.launch.py objects_topic:=/vision/objects

# 只加载机器人本体和控制器，不启动 MoveIt
ros2 launch my_robot_bringup ros2_control.launch.py robot_only:=true with_moveit:=false

# 单独控制阶段测试
python3 test/robot_control.py
```

启动请求的顺序不代表节点已就绪。任务规划器会等待 Action 和观测；外部客户端应确认接口可用后发送目标。

## 分层启动完整项目

可以使用三个终端依次启动。三个终端都要加载 ROS 与工作区，并设置同一个**空闲** ROS 域；以下 `227` 只是示例：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=227
```

三个终端分别执行：

```bash
# 终端 1：完整世界、相机、控制器、MoveIt；等待启动完成
ros2 launch my_robot_bringup ros2_control.launch.py

# 终端 2：执行器、任务规划器；等待 PickPlace executor ready
ros2 launch my_robot_bringup motion_control.launch.py

# 终端 3：颜色检测、深度定位
ros2 launch my_robot_bringup perception.launch.py
```

控制层是唯一的 MuJoCo 物理推进者；另外两层不会创建新的仿真。与一次启动的 `mujoco.launch.py` 一样，节点上线不会自动抓取。三个分层默认的相机话题、物体话题和 Action 名称一致；如果单独覆盖某一层的接口名称，需要在使用同一接口的其他层传入相同值。总入口则会统一传递这些参数。

## 三色任务和调试

[脚本说明](../../script/README.md)介绍环境准备、自动触发分拣，以及直接使用 launch 时如何手动发送 Action。`show_window:=true` 控制颜色窗口；MuJoCo 窗口由 `headless` 控制。即使 `headless:=true`，相机插件仍需要 OpenGL 环境。

若日志显示加载 `webots` 或找不到 `mujoco_ros2_control/MujocoSystem`，可能是同 ROS 域的其他发布者提供了错误的 `/robot_description`。先检查同域节点，使用空闲 `ROS_DOMAIN_ID` 再启动；不要直接全局杀掉其他项目的节点。

MoveIt RViz 可在相同域单独启动：

```bash
ros2 launch my_robot_moveit_pkg moveit_rviz.launch.py use_sim_time:=true
```

MuJoCo 物块尚未自动同步为 MoveIt 碰撞物体，任务运动完成不代表最终放置位置已核验。现有环境中部分 C++ 进程退出时仍可能出现析构异常，需结合子进程日志检查。
