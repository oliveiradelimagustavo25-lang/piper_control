# 运行与清理脚本

## 三色抓放仿真

先构建工作区：

```bash
source /opt/ros/humble/setup.bash
colcon build
./script/pick_three_colors.sh --ros-domain-id 227
```

脚本读取工作区的 `config/mujoco_launch.yaml`，加载环境、检查 ROS 域并启动 `mujoco.launch.py`；总入口启动仿真、MoveIt、任务规划器和感知节点。脚本等待机械臂控制器与命名姿态 Action 就绪，发送 `pose_1` 目标；到位后自动发送一次分拣任务，使用 `task_planner.yaml` 中的默认策略和颜色顺序。任务完成后仿真继续运行；如果动作或分拣失败，脚本会显示结果并退出。

直接使用 `ros2 launch my_robot_bringup mujoco.launch.py` 不会自动开始分拣。需要手动触发时，在同一 ROS 域的另一终端发送目标：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=227
ros2 action send_goal --feedback /sorting/sort_objects my_robot_interface/action/SortObjects \
  '{strategy: color_order, color_order: [1, 2, 3]}'
```

规划器收到分拣目标后会按自身配置再次到观察位 `pose_1`，等待新图像，再按默认红、绿、蓝顺序执行抓放。Action 完成表示运动序列完成，物块最终位置仍需视觉核验。脚本会读取任务规划器实际配置的 Action 名称；手动调用时，如用 `sort_action` 覆盖了名称，也应使用新名称。已有分拣任务在执行时，不要同时发送重复目标。

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

清理范围同时满足当前用户、工作目录在本工作区内、ROS 域匹配，以及属于已知启动链。识别两种残留：节点直接被 init/用户 systemd 接管；或者 `my_robot_bringup` 的已知 launch 被接管、它下面的节点仍由这个 launch 管理。后一种情况正是“node list 还有很多节点，但旧 clear 什么都找不到”的原因。

覆盖机器人状态、MuJoCo 控制、spawner、MoveIt、抓放执行器、task_planner 和感知节点。RViz 只随选中的孤立 launch 清理，不单独清理。仍受正常活动启动链管理的任务、其他工作区和其他域不处理；可在对应终端正常退出。

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

只需修改 [config/mujoco_launch.yaml](../config/mujoco_launch.yaml)，然后运行：

```bash
./script/pick_three_colors.sh
```

总配置中的 `scenes` 按颜色填写坐标，每个 `[x, y, z]` 生成一个原有模型尺寸的物块（m）：

```yaml
scenes:
  red: [[0.32, -0.12, 0.03], [0.32, -0.06, 0.03], [0.32, 0.0, 0.03]]
  green: [[0.32, 0.06, 0.03]]
  blue: [[0.32, 0.12, 0.03]]
```

数量由列表长度决定，不需要另外创建场景配置文件或 XML。四个绿色也只需填写四个绿色坐标，同时将 `red`、`blue` 设为 `[]`。

- 不写 `scenes`、写 `{}` 或 `null`：使用原有默认三色。
- 某颜色未写或为 `null`：沿用该颜色默认一个物块。
- 某颜色为 `[]`：不生成该颜色。
- 显式传入 `scene_file`（命令行优先，其次总配置）时，使用该世界文件。

启动链自动调用 [script/build/scene_config.py](build/scene_config.py)，在临时目录生成 `objects.xml` 和 `scene.xml`，退出时自动清理；直接用 `ros2 launch ... launch_config:=...` 也支持相同规则。`script/build/` 的辅助 Python 源码已纳入版本管理范围，缓存和运行产物仍忽略。这里的 build 是脚本辅助目录，与 colcon 的根目录 `build/` 不同。

场景列表只定义仿真输入；planner 按实际观测工作。各色放置区域、原点与间距定义放置容量，不再要求按物体数量手写坐标列表。配置与异常处理见 [planner 说明](../src/my_robot_planner/README.md)。中途增删不在本轮 MuJoCo 改造范围内。

## 放置坐标规则与优先级

在 `config/mujoco_launch.yaml` 配置：

```yaml
place_position_modes: [relative, relative, relative]  # 按 X/Y/Z，不是按颜色
place_positions:
  red: [-0.050, -0.010, 0.0005]
  green: [-0.050, -0.010, 0.0005]
  blue: [-0.050, -0.010, 0.0005]
```

在执行器统一的 `reference_frame`（当前 `base_link`）中逐轴计算。relative 是物体首次派发前观测中心加偏移，absolute 是直接使用给定中心坐标；重试更新抓取位置，放置目标冻结，不反复累加。全 absolute 表示每色固定绝对中心。相对规则不会自动聚集或把同色物体压到统一位置。

Z 始终是物体中心/中心偏移，不是桌面高度或 TCP。桌面 -0.145 m、方块高 0.05 m 时理论中心是 -0.120 m。模式 `[relative, absolute, absolute]`、红色 `[-0.24,-0.06,-0.12]` 表示 `[首次派发x-0.24,-0.06,-0.12]`，不是三个方向都相加。

优先级为：**显式 launch 参数 > 总配置对应字段 > planner YAML > 节点默认值**。`--planner-config` 选择基础 planner YAML，不能覆盖总配置里已经显式填写的放置规则。模式和值分别按优先级合并；省略模式使用低优先级模式，默认全 absolute。

显式 `place_positions` 一旦生效，就覆盖旧的 `<color>.place_position/place_region/place_slots` 选择策略，不与其生成位置混用。删除总配置两项后仍会读取 planner YAML；若各层均没有新 `place_positions`，才使用原区域/槽位策略。三色数组必须完整、每个是三个有限数字；null、缺色、错误长度直接在启动校验中报错，不能静默套用旧仿真值。

```bash
# 只显示有效规则，不检查空闲域，不启动仿真、节点或动作
./script/pick_three_colors.sh --dry-run
# 命令行按轴覆盖总配置中的模式；值仍来自总配置
./script/pick_three_colors.sh --dry-run -- 'place_position_modes:=[relative, absolute, absolute]'
```

相对 dry-run 缺少实时物体位置，只打印规则；最终绝对目标由 Planner 首次派发前计算并打印。完整覆盖值也可传 `place_positions:={red: [...], green: [...], blue: [...]}`。目标被占用或不可达时按现有等待/错误策略处理，本次不自动扩展为聚集或统一放置。
