# 运行与清理脚本

## 三色抓放仿真

先构建工作区：

```bash
source /opt/ros/humble/setup.bash
colcon build
./script/pick_three_colors.sh
```

脚本自动加载 ROS Humble 和本工作区，使用当前 `ROS_DOMAIN_ID`；未设置时使用域 227。它启动 `mujoco.launch.py`，等待三个控制器、执行 Action 和相机图像就绪，接着启动任务规划器并发送三色任务。规划器先移动到 `pose_1`，然后按红、绿、蓝执行抓放；每次抓取前会返回观察位。任务结束后仿真保持打开，按 Ctrl+C 退出。Action 失败会显示错误，动作完成也需要检查物块实际放置位置。

```bash
# 指定 ROS 域，并将参数传给 mujoco.launch.py
./script/pick_three_colors.sh --ros-domain-id 227 -- sim_speed_factor:=0.5

# 隐藏 MuJoCo 与颜色调试窗口（相机仍需要 OpenGL 环境）
./script/pick_three_colors.sh --headless --no-debug-window

# 按距离选择目标，或替换规划器的配置
./script/pick_three_colors.sh --strategy nearest --planner-config /path/to/task_planner.yaml

./script/pick_three_colors.sh --help
```

`--` 后的 `name:=value` 原样传给 `mujoco.launch.py`。脚本从实际运行节点读取抓放、命名姿态 Action 及物体、图像话题名称，传给任务规划器；高级参数仍在各自 YAML 中配置，若修改 TCP、坐标系或物体尺寸，应保持规划与执行配置一致。脚本会覆盖规划器的观察姿态为 `pose_1`，其目标消息始终请求红、绿、蓝三个颜色。

脚本启动前检查目标 ROS 域；如果发现已有节点，会停止并要求选择空闲域，避免连接到其他仿真或设备。DDS 发现检查不能保证排除稍后启动的节点，在其他终端调试时需要设置相同域号。`headless` 只隐藏 MuJoCo 窗口，并不关闭相机渲染；无图形环境需要可用的 OpenGL 渲染方案。此脚本不会启动 RViz、自动验证物块最终位置，或修复已有节点退出异常。

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
