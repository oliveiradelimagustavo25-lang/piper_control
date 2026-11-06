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

默认总配置保存在 [config/mujoco_launch.yaml](../config/mujoco_launch.yaml)，修改后重新运行脚本即可，不需要重新 `colcon build`。其中 `with_rviz: true` 会启动 MoveIt RViz，默认 `false`；`--rviz` 可临时启用。该文件中只写需要覆盖的 launch 参数；布尔值使用 `true` / `false`，路径相对于 YAML 自身所在目录。未知参数名会报错。`--headless`、`--no-debug-window`、`--rviz`、`--planner-config` 和 `--` 后的同名参数会覆盖总配置。例如将总配置中的 `headless` 设为 `true` 后，可通过 `-- headless:=false` 临时打开窗口。

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
# 只查看将被处理的进程
./script/clear.sh --dry-run

# 确认不再需要这些进程后，清理残留
./script/clear.sh
```

脚本只检查当前用户的进程：工作目录位于本工作区内、父进程已经变为 init 或用户 systemd，并且可执行文件属于本项目启动链中的已知节点或控制器 spawner。它覆盖 `robot_state_publisher`、MuJoCo 控制节点、三个控制器的 spawner、MoveIt `move_group`、抓放执行端及颜色和深度节点。工作目录和父进程是识别线索，不能绝对证明节点由本项目创建；如果在本工作区目录中运行了其他 ROS 项目，先用 `--dry-run` 检查结果。

脚本先发送 `SIGINT`，等待后依次尝试 `SIGTERM` 和 `SIGKILL`。它不清理仍由 launch 管理的节点、其他工作目录中的进程、RViz 或 ROS daemon，也不会按 ROS 域筛选。先正常关闭仿真；如果 launch 本身仍然运行，应回到其终端按 Ctrl+C 停止。强制终止解决的是残留进程，并不修复其退出异常的根因。

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
