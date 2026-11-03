# my_robot_bringup

`mujoco.launch.py` 是完整世界的正式入口，默认加载机器人、棋盘格地面、三色物块、标记板和腕部相机，并启动 ros2_control、MoveIt 与 `pick_place_executor`。`ros2_control.launch.py` 是共享的控制启动层，单独启动时只加载机器人本体，供交互控制阶段测试复用。

```bash
cd /home/ltt/piper_control
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch my_robot_bringup mujoco.launch.py
ros2 launch my_robot_bringup mujoco.launch.py --show-args
```

构建后重新加载 `install/setup.bash`，新终端也需要加载工作区环境。

若系统更新后提示找不到旧版本 MoveIt 共享库，应重新配置并编译依赖它的工作区包；本次诊断与重建方法见[系统库检查报告](../../docs/2026-11-03_system_library_audit.md)。

## 模型与控制的连接

世界加载器把 `my_robot_world/basic/preview.xml` 与机器人 `mjcf/control.xml` 组合，将场景展开为临时 MJCF，并使用绝对资源路径。`piper_control.urdf.xacro` 的 `model_path` 指向该场景，插件加载的是真实完整世界。临时文件在 launch 退出时清理，源码与安装目录均不生成运行文件。

ros2_control 插件是唯一的物理推进与 `/clock` 发布者；正式启动不运行 `test/mujoco_world.py`，也不创建第二个 Python 仿真实例。机器人状态发布、MoveIt 和执行端使用相同的 URDF 与仿真时间。

## 启动参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `headless` | `false` | 隐藏 MuJoCo 窗口，保留物理和 ROS 接口 |
| `with_moveit` | `true` | 启动 `move_group` |
| `with_executor` | 跟随 `with_moveit` | 启动抓放、命名姿态 Action；需要 MoveIt |
| `sim_speed_factor` | `1.0` | 有限正数，目标仿真倍率 |
| `scene_file` | 世界包 `basic/preview.xml` | 地面、物体、标记板组成的场景 |
| `robot_model` | 描述包 `mjcf/control.xml` | 含执行器的机器人 MJCF |
| `camera_config` | 世界包 `config/camera.yaml` | 腕部相机配置 |
| `controllers_file` | bringup `config/ros2_controllers.yaml` | ros2_control 控制器、关节和容差 |
| `moveit_controllers_file` | MoveIt 包 `config/moveit_controllers.yaml` | MoveIt 到轨迹控制器的映射 |
| `planning_pipeline` | `ompl` | MoveIt 运动规划管线 |
| `manipulation_config` | commander `config/manipulation.yaml` | 执行器及 MotionController 参数 |

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

# 完整世界和控制器，暂不启动 MoveIt 与执行器
ros2 launch my_robot_bringup mujoco.launch.py with_moveit:=false

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
| MoveIt `move_group` | 运动规划与轨迹执行接口 |
| `pick_place_executor` | 默认 `/manipulation/pick_place`、`/manipulation/move_to_named` |
| `motion_controller.cpp` | 由执行器链接使用的运动库，承接 MoveIt 调用 |

不启动检测器或 `my_robot_planner` 任务策略。MuJoCo 中的物块目前不会自动同步为 MoveIt 碰撞物体，相机也尚未发布 ROS 图像或 CameraInfo；完整世界显示与控制接口就绪不等于抓取流程已验收。

当前环境中，部分 C++ 节点退出时仍可能出现析构异常，不能仅凭 launch 返回 0 判断子进程正常退出。详细测试和限制见[本轮开发报告](../../docs/2026-11-02_world_control_roles.md)。
