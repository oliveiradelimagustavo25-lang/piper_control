# 运行与清理脚本

## 三色抓放仿真

先构建工作区：

```bash
source /opt/ros/humble/setup.bash
colcon build
./script/pick_three_colors.sh --ros-domain-id 227
```

脚本读取工作区的 `config/mujoco_launch.yaml`，加载环境、检查 ROS 域并启动 `mujoco.launch.py`；总入口启动仿真、MoveIt、任务规划器和感知节点。脚本等待机械臂控制器与命名姿态 Action 就绪，发送 `pose_1` 目标；到位后自动发送一次分拣任务，使用 `task_planner.yaml` 中的默认策略和颜色顺序。任务完成后仿真继续运行；命名姿态失败会退出；分拣部分失败会显示汇总并保留仿真。

直接使用 `ros2 launch my_robot_bringup mujoco.launch.py` 不会自动开始分拣。需要手动触发时，在同一 ROS 域的另一终端发送目标：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=227
ros2 action send_goal --feedback /sorting/sort_objects my_robot_interface/action/SortObjects \
  '{strategy: class_order, class_order: [1, 2, 3]}'
```

规划器收到分拣目标后会按自身配置再次到观察位 `pose_1`，等待新图像，再按配置的类别优先级执行抓放。Action 完成表示运动序列完成，物块最终位置仍需视觉核验。脚本会读取任务规划器实际配置的 Action 名称；手动调用时，如用 `sort_action` 覆盖了名称，也应使用新名称。已有分拣任务在执行时，不要同时发送重复目标。

```bash
# 将 -- 后的参数传给 mujoco.launch.py
./script/pick_three_colors.sh --ros-domain-id 227 -- sim_speed_factor:=0.5

# 隐藏 MuJoCo 和颜色调试窗口（相机仍需要 OpenGL 环境）
./script/pick_three_colors.sh --headless --no-debug-window

# 随仿真一起打开 MoveIt RViz
./script/pick_three_colors.sh --rviz

./script/pick_three_colors.sh --planner-config /path/to/task_planner.yaml

# 选择另一份总配置；相对路径以该配置文件所在目录为基准
./script/pick_three_colors.sh --config /path/to/mujoco_launch.yaml
./script/pick_three_colors.sh --help
```

默认总配置保存在 [config/mujoco_launch.yaml](../config/mujoco_launch.yaml)，修改后重新运行脚本即可，不需要重新 `colcon build`。其中 `with_rviz: true` 会启动 MoveIt RViz，默认 `false`；`--rviz` 可临时启用。该文件除 `scenes` 嵌套场景配置外，填写需要覆盖的 launch 参数；布尔值使用 `true` / `false`，路径相对于 YAML 自身所在目录。未知参数名会报错。`--headless`、`--no-debug-window`、`--rviz`、`--planner-config` 和 `--` 后的同名参数会覆盖总配置。例如将总配置中的 `headless` 设为 `true` 后，可通过 `-- headless:=false` 临时打开窗口。

默认总配置直接指向工作区的六份节点 YAML，修改它们并重启脚本也无需重新构建。MoveIt 自动读取的 `kinematics.yaml`、`joint_limits.yaml` 等未由总配置的路径参数覆盖，仍来自安装目录，修改源码后需要重新构建对应包（除非原先使用了适用的符号链接安装）。运行中的节点不会自动重新加载 YAML。

脚本默认继承 `ROS_DOMAIN_ID`，未设置时使用 227。启动前如果发现同域已有节点会停止，以减少机器人描述或控制接口冲突；DDS 检查无法保证发现稍后启动的节点。`--headless` 只隐藏 MuJoCo 窗口，不关闭相机渲染。按 Ctrl+C 退出 launch。

## 命名姿态 Action

脚本自动移动到 `pose_1` 使用 `pick_place_executor` 提供的命名姿态接口：

- 默认名称：`/manipulation/move_to_named`。
- Action 类型：`my_robot_interface/action/MoveToNamed`。
- Goal：`target`，例如 `pose_1`；Result 包含 `success` 和 `message`。

在同一 ROS 域中，也可以手动发送：

```bash
ros2 action send_goal --feedback /manipulation/move_to_named \
  my_robot_interface/action/MoveToNamed '{target: pose_1}'
```

脚本会读取运行中执行器的 `move_to_named_action` 参数，因此通过 `manipulation.yaml`、总配置或 launch 参数修改接口名称后，脚本会使用实际名称。`pose_1` 定义在 MoveIt 的 `piper.srdf` 中。抓放和命名姿态共用执行器，执行器忙时新的目标可能被拒绝。

## 清理退出后残留的 ROS 进程

```bash
export ROS_DOMAIN_ID=227
./script/clear.sh --dry-run
./script/clear.sh
# 也可显式指定域
./script/clear.sh --ros-domain-id 227 --dry-run
```

`clear.sh` 是入口，进程识别位于 `script/build/clear_processes.py`，只使用 Python 标准库，无需加载 ROS 环境。默认使用当前 `ROS_DOMAIN_ID`，未设置时为域 0。

只清理匹配当前用户、指定 ROS 域和本工作区的已知进程。用工作目录、工作区内的节点可执行文件或明确的配置路径判定归属。进程链被 init 或用户 systemd 接管后，清理脚本会选中该入口及匹配的子进程。

覆盖机器人状态、MuJoCo 控制、spawner、MoveIt、抓放执行器、task_planner、CV、depth、ArUco、YOLO 和 GraspNet 候选发布器。也识别工作区 ROS 节点的 `ros2 run` 包装进程，以及 GraspNet 在线推理的 shell 和 Python 进程。RViz 可以随残留启动链清理，孤立 RViz 需使用本工作区的 `moveit.rviz`。在对应终端正常退出仍在使用的任务。

先只向孤立 launch/独立孤立节点发送一次 SIGINT，等待 10 秒；随后对仍存活的所选进程依次发送 TERM、KILL。每次发送前检查 PID 启动时间、域、目录和可执行文件，降低 PID 复用造成误伤的风险；僵尸进程不算仍运行的节点。最终核验所选 PID 已退出，不停止 ROS daemon。

`ros2 node list` 的条目与系统进程并非一一对应，一个进程可以产生多个 ROS 节点。若清理后仍显示名字，先使用相同域的 `ros2 node list --no-daemon` 检查实时发现；缓存、同域其他进程或其他主机不能通过本地残留清理一概删除。工作目录是归属线索，并不等同于独占证明，建议先看 `--dry-run` 的 PID 和选中原因。

## 单独检查和终止节点

ROS 节点不是 `ros2 node kill` 可以直接终止的对象。先查明系统进程及其父进程，再对确认不再需要的 PID 发信号：

```bash
ps -eo pid,ppid,pgid,stat,args | grep '[r]obot_state_publisher'
kill -INT <PID>
# 如果进程仍不退出，可依次使用：
kill -TERM <PID>
kill -KILL <PID>
```

节点列表依赖 ROS 域和 DDS 发现，进程退出后名字可能暂时仍出现在列表中，也可能来自同域其他机器。可以用 `ros2 node list --no-daemon`、`ros2 node info /robot_state_publisher --no-daemon` 进一步检查，并结合 `ps` 判断本机进程是否还在。不要直接按名称全局 `pkill`，以免影响其他项目。

## 场景与一键启动

检测器可在总配置中选 `detector: cv`（默认）或 `detector: yolo`。
YOLO 的外部工具与 ROS 适配节点需要预先分别构建，模型需包含项目所用类别，见
[tools/yolo/README.md](../tools/yolo/README.md)。入口不变，也可临时覆盖：

```bash
./script/pick_three_colors.sh -- detector:=yolo yolo_model:=/absolute/path/sorting.onnx
```

`yolo_model` 总配置路径相对于总 YAML，显式命令行相对于当前目录；未提供则使用感知 YAML 的
`yolo_detector.model_path`，该路径相对于感知 YAML。选择 CV 时无需配置模型。

只需修改 [config/mujoco_launch.yaml](../config/mujoco_launch.yaml)，然后运行：

```bash
./script/pick_three_colors.sh
```

总配置中的 `scenes` 按共同类别表的 ID 填写位置（m，世界坐标），每个数组是一个实例：

```yaml
scenes:
  1: [[0.32, -0.12, 0.03], [0.32, -0.04, 0.03]]
  2: [[0.32, 0.04, 0.03]]
  3: []
```

- 不写 `scenes` 或为 `null`：使用类别表 `simulation.default_position` 定义的默认实例，当前为三色各一。
- 显式映射只生成列出的类别；`{}` 是空场景，某类别 `[]` 为零个，不会暗中补齐其他类别。
- 未定义类别、无效位置或暂不支持的模型只跳过自身，打印原因；不把 ID 猜成其他模型。
- 显式 `scene_file` 优先于自动生成。

启动链在所有参数覆盖完成后读取有效类别表，调用现有 [scene_config.py](build/scene_config.py) 生成 XML。几何和尺寸来自 `object_classes.yaml`，当前支持 box、cylinder、sphere 的 MuJoCo 生成；圆柱尺寸约定为 `[直径, 直径, 高]`，不表示当前 CV/depth 已支持圆柱识别抓取。`simulation.mass` 默认 0.001 kg，`simulation.rgba` 可选覆盖外观。

临时目录包含 `scene_report.json`，分别记录请求数、生成数和跳过原因，退出时清理；启动日志也打印汇总。生成器使用 Python 读取同一 YAML，不另建类别表。场景数量不传给 Planner，后者按实际观测选择目标。中途增删和聚集/统一放置不在本轮范围。

## 放置坐标规则与优先级

```yaml
place_position_modes: [relative, relative, relative]  # X/Y/Z
place_positions:
  1: [-0.050, -0.010, 0.0005]
  2: [-0.050, -0.010, 0.0005]
  3: [-0.050, -0.010, 0.0005]
```

键是类别 ID；整数键和字符串数字等价，可不连续。只填写需要处理的类别。未定义类别的规则被忽略；规则缺失、null 或数组无效的类别在抓取前跳过，不回退其他类别或旧仿真坐标。表中存在但场景未出现的类别不会阻塞。

优先级：**显式 launch 参数 > 总 YAML > planner YAML**。高优先级 `place_positions` 整体替换低优先级规则，缺失项不会从低层补回。显式 `{}` 表示不启用任何目标抓取；所有层都未提供 `place_positions` 时，才使用 Planner 中 `classes.<id>.place_position/place_region/place_slots` 的显式配置，没有硬编码默认放置点。

relative 使用物体首次派发前在 `reference_frame`（当前 base_link）中的中心加偏移，重试只刷新抓取位置，绝对放置目标冻结。新 SortObjects 任务重新建立基准。absolute 直接使用给定中心。Z 是物体中心或中心偏移，不是桌面或 TCP；例如桌面 -0.145 m、物高 0.05 m 的中心为 -0.120 m。混合模式 `[relative, absolute, absolute]` 和 `[-0.24,-0.06,-0.12]` 得到 `[首次x-0.24,-0.06,-0.12]`。

```bash
./script/pick_three_colors.sh --dry-run
./script/pick_three_colors.sh --dry-run -- 'place_position_modes:=[relative, absolute, absolute]'
./script/pick_three_colors.sh --dry-run -- 'place_positions:={1: [-0.05, -0.01, 0.0005]}'
```

相对 dry-run 只显示规则；最终绝对目标见 Planner 的 `Frozen placement` 日志。多个绝对目标重合、目标被占用或不可达时仍按现有策略处理，不强行重叠。跳过的可靠三维物体仍参与放置占用和 MoveIt 障碍检查。
