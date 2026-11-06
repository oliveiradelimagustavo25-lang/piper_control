# Piper MuJoCo 工作区

本分支面向 Piper 机械臂的 MuJoCo 仿真。正式入口组合机器人、地面、物块、标记板和相机，并提供 ros2_control、MoveIt、抓放执行与颜色深度感知。执行器在运动前将视觉物块同步到 MoveIt 规划场景；最终物理抓取仍需独立验证。

## 当前结构

```text
src/
  my_robot_description/       # 机器人描述、控制接口与控制器配置
    urdf/piper_description.urdf
    urdf/piper_ros_control.xacro
    urdf/piper_control.urdf.xacro
    config/ros2_controllers.yaml
    mjcf/                     # 机器人本体与 MuJoCo 执行器
  my_robot_world/            # MuJoCo 世界、物块、标记板和相机参数
    objects/
    basic/preview.xml         # 基础世界组合入口
    scenes/scene.py           # 场景和跨包机器人资源加载
    sensors/camera.py         # 腕部相机
    assets/
    config/camera.yaml
  my_robot_bringup/           # 仿真、控制与执行启动入口
    launch/mujoco.launch.py
    launch/ros2_control.launch.py # 默认完整世界、控制与 MoveIt
    launch/motion_control.launch.py # 执行器与任务规划器
    launch/perception.launch.py   # 颜色检测与深度定位
  my_robot_interface/
  my_robot_perception/
  my_robot_planner/
  my_robot_commander/
  my_robot_moveit_pkg/
    config/moveit.rviz        # MoveIt RViz 唯一配置
    launch/moveit_rviz.launch.py

config/
  mujoco_launch.yaml          # 工作区可直接修改的总启动配置

script/
  pick_three_colors.sh        # 启动仿真、移动到 pose_1 并触发分拣
  clear.sh                    # 清理本工作区已脱离启动进程的 ROS 残留
  README.md

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
| `my_robot_bringup/mujoco.launch.py` | 完整世界、ros2_control、MoveIt、执行接口与感知 |

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
# 完整世界、ros2_control、MoveIt、执行接口和感知
ros2 launch my_robot_bringup mujoco.launch.py

# 完整世界、控制器与 MoveIt，不启动任务和感知节点
ros2 launch my_robot_bringup ros2_control.launch.py

# 只打开机器人本体窗口，保持 ros2_control 接口在线
python3 test/robot_control.py

# 自动检查状态、时钟、控制器和小幅轨迹
python3 test/probes/check_robot_control.py
```

`mujoco.launch.py headless:=true` 可无窗口运行；`with_moveit:=false` 保留完整世界与控制器；`with_motion:=false` 关闭执行、规划与感知层；`sim_speed_factor:=0.5` 设置半速仿真。场景、控制器、执行配置和常用运动参数均提供 arguments，使用 `--show-args` 查看，定义见 [bringup/README.md](src/my_robot_bringup/README.md)。交互测试继承终端 ROS 域，自动探针默认使用域 184。机器人本体测试与完整仿真实例应独立运行，避免同域出现多个控制仿真；控制、运动和感知三个分层可在同一 ROS 域中依次启动并配合。正式入口默认启动任务规划器和颜色、深度感知；任务需要通过 Action 显式触发，不启动 ArUco 检测。

若 MuJoCo 或颜色调试窗口没有出现，先检查启动日志和同域 `/robot_description` 是否冲突；窗口参数及隔离 ROS 域的方法见 [bringup 故障排查](src/my_robot_bringup/README.md#窗口不显示时)。

`robot_description.py --rviz` 打开的是静态预览及关节滑块，滑块不驱动 MuJoCo。正式仿真可通过总配置的 `with_rviz: true` 打开 MoveIt RViz，也可通过命令行临时开启：

```bash
./script/pick_three_colors.sh --rviz
# 或直接启动总入口
ros2 launch my_robot_bringup mujoco.launch.py with_rviz:=true
```

`with_rviz` 默认 `false`，需要 `with_moveit:=true`。`headless` 只控制 MuJoCo 窗口，`show_window` 只控制颜色调试窗口。如果仿真已运行，也可以在相同 ROS 域的另一终端单独打开 MoveIt RViz：

```bash
source install/setup.bash
ros2 launch my_robot_moveit_pkg moveit_rviz.launch.py use_sim_time:=true
```

该 RViz 使用 `world` 固定坐标系，与 MoveIt 连接。执行器在命名姿态或抓放动作开始前同步最近的视觉物块，RViz 的 MotionPlanning 可以查看相应碰撞盒；直接在 RViz 中规划不会触发执行器同步。其他测试选项、相机渲染和安装路径说明见 [test/README.md](test/README.md)。

## 碰撞场景与抓放

[manipulation.yaml](src/my_robot_commander/config/manipulation.yaml) 定义 MoveIt 地面、标记板碰撞盒、夹爪允许接触的连杆和抓取参数。默认盒体与 MuJoCo 场景中的地面和标记板对齐；视觉物块在动作前按检测到的位姿和尺寸同步。抓放时目标在 MoveIt 中附着到工具，释放后回到世界场景；恢复服务 `/manipulation/release_held_object` 用于处理取消或失败后仍保留的负载模型。

规划场景与 MuJoCo 物理世界相互独立：MoveIt 中附着不代表实际夹住，自动 OctoMap 也没有启用。当前只在动作前更新视觉快照，不连续同步，暂时丢失或离开视野的旧物块不会自动删除。场景盒和抓取偏移适用于当前仿真，改变世界或接入实物前需重新核对。

```bash
python3 test/probes/check_planning_scene.py           # 地面、标记板、视觉盒与碰撞状态
python3 test/probes/check_planning_scene.py --grasp   # 额外验证抓放附着、释放位置
python3 test/probes/check_planning_scene.py --recovery # 取消后保留并显式释放
```

## 三色抓放仿真

构建工作区后，运行 `./script/pick_three_colors.sh`，脚本会读取工作区的 `config/mujoco_launch.yaml`、启动完整仿真，并在命名姿态接口就绪后自动移动到 `pose_1`，随后发送一次三色分拣任务；修改该配置后重新运行即可，无需重新构建。默认继承当前 ROS 域，未设置时使用 227；详细选项和限制见 [script/README.md](script/README.md)。

脚本使用的命名姿态 Action 默认名称为 `/manipulation/move_to_named`，类型为 `my_robot_interface/action/MoveToNamed`，目标为 `{target: pose_1}`。在同一 ROS 域中可手动调用：

```bash
ros2 action send_goal --feedback /manipulation/move_to_named \
  my_robot_interface/action/MoveToNamed '{target: pose_1}'
```

接口名称可通过 `move_to_named_action` 配置；脚本会读取执行器的实际名称。脚本还会发送 `/sorting/sort_objects` 分拣目标，规划器再按自身配置前往观察位并抓放；使用脚本时无需另行发送重复目标。

## 总配置与 YAML 覆盖关系

[config/mujoco_launch.yaml](config/mujoco_launch.yaml) 是脚本使用的总启动配置，可调整窗口、仿真速度、分层开关、话题和 Action 名称等。初次构建后，修改该文件或其中默认指向的六份源码 YAML，**重新运行脚本即可生效，无需再次构建**；运行中的节点不会自动加载修改。

```bash
# 使用默认总配置；ROS 域通过选项或环境变量设置
./script/pick_three_colors.sh --ros-domain-id 227

# 指定另一份总配置，或临时覆盖其中的仿真速度
./script/pick_three_colors.sh --config /path/to/mujoco_launch.yaml -- sim_speed_factor:=0.8

# 直接启动总入口时，显式传入总配置；不传则使用 launch 自身默认值
ros2 launch my_robot_bringup mujoco.launch.py launch_config:="$PWD/config/mujoco_launch.yaml"
ros2 launch my_robot_bringup mujoco.launch.py --show-args
```

总配置中填写的相对文件路径以该 YAML 文件所在目录为基准；未知参数名会报错。`--show-args` 展示可用参数及原始回退值，实际总配置请查看 YAML 文件。一般的同名参数优先级是：

```text
命令行显式参数 > 总配置 YAML > 对应节点 YAML > 节点代码默认值
```

例如，总配置中设置 `show_window: false` 后，脚本默认不会覆盖它；显式传入 `-- show_window:=true` 则会临时打开颜色窗口。总配置中未填写或填写为 `null` 的参数使用原默认值。

默认总配置将下面六份配置指向工作区源码：

| 路径参数 | 默认配置 | 使用位置 |
| --- | --- | --- |
| `camera_config` | [camera.yaml](src/my_robot_world/config/camera.yaml) | 控制层与世界加载器：相机模型、视野和发布频率 |
| `controllers_file` | [ros2_controllers.yaml](src/my_robot_description/config/ros2_controllers.yaml) | 控制层与控制器管理器：控制器、关节和容差 |
| `moveit_controllers_file` | [moveit_controllers.yaml](src/my_robot_moveit_pkg/config/moveit_controllers.yaml) | 控制层与 motion 层：MoveIt 控制器映射 |
| `manipulation_config` | [manipulation.yaml](src/my_robot_commander/config/manipulation.yaml) | motion 层：抓放执行和 TCP 等参数 |
| `planner_config` | [task_planner.yaml](src/my_robot_planner/config/task_planner.yaml) | motion 层：任务策略、观察姿态和放置位置 |
| `perception_config` | [perception.yaml](src/my_robot_perception/config/perception.yaml) | 总入口、控制层及 perception 层：话题、颜色和深度配置 |

部分接口会为了保持两端一致而同步：相机话题由 launch 参数或感知 YAML 统一提供给发布与订阅端；感知层启用时，规划器的 `objects_topic` 跟随深度节点输出；规划器的抓放 Action、参考系和 TCP 等跟随执行器配置。因而只修改 `task_planner.yaml` 中这些对应字段时，可能被同步值覆盖。`pick_place_action` 与旧别名 `action_name` 同时指定且不一致时会报错。

MoveIt 自动加载的 `kinematics.yaml`、`joint_limits.yaml` 等默认仍来自安装目录，不属于上述六个源码配置路径；修改源码后通常需要重新构建对应包，或使用适当的符号链接安装。详细读取位置、同步规则和其他例子见 [bringup README](src/my_robot_bringup/README.md#外部总配置与覆盖关系)。

## 清理残留进程

启动入口退出后，如果节点仍留在后台，可以先执行 `./script/clear.sh --dry-run` 查看，再用 `./script/clear.sh` 清理。脚本仅处理本工作区中已被 init 或用户 systemd 接管的已知进程；识别范围、手动终止方法和 ROS 发现信息说明见 [script/README.md](script/README.md)。

## 分工与下一步

`my_robot_description` 保存机械臂和相机外形、安装坐标、ros2_control 描述和控制器配置；`my_robot_bringup` 负责选择配置并启动系统。`my_robot_world` 使用机器人描述组合世界，并在相机光学坐标系上添加 MuJoCo 相机。场景文件通过包内加载器解析机器人引用，避免复制机器人模型。正式入口复用 MuJoCo 原生相机插件发布 RGB、对齐深度和 CameraInfo，默认话题与感知配置一致；画面遮挡、识别效果和相机视角仍需实际校验。

相机与支架网格已修正额外 90° 上轴旋转，两个测试查看器恢复侧栏，世界加入棋盘格地面。查看器退出时等待其线程清理完成；世界窗口支持空格暂停/继续、右方向键单步。修改与验证结果见[相机、查看器与启动参数修正报告](docs/2026-11-02_camera_viewer_launch_fix.md)。

当前完整世界由原生控制插件统一推进，测试脚本不作为正式运行库。`motion_controller.cpp` 作为共享库由 `pick_place_executor` 使用，执行参数由 YAML 配置并允许 launch 覆盖。职责与接口修正见[完整世界启动报告](docs/2026-11-02_world_control_roles.md)。

后续继续校准控制精度、完善视觉物体的持续同步和抓后位置验证。当前环境的部分 C++ 节点在退出时仍有析构异常，详见本轮报告。

正式测试和必要资源建议纳入 Git；临时测试放在 `test/local/`，构建产物和缓存已忽略。临时节点移除及当前执行入口见[开发报告](docs/2026-11-02_remove_temporary_commander.md)，MoveIt 和执行链整合见[整合报告](docs/2026-11-02_mujoco_moveit_execution.md)，此前控制链构建见[控制报告](docs/2026-11-02_mujoco_control.md)，此前世界构建过程见[构建报告](docs/2026-11-02_mujoco_world_construction.md)。
