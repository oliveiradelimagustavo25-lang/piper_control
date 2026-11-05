# 运行入口

本目录集中保存手写 `.sh`：一键脚本启动仿真和任务，物理验收流程由 `test/` 中的 Python 模块负责。

## 直接启动并发送三色任务

```bash
./script/pick_three_colors.sh
./script/pick_three_colors.sh gui:=false use_rviz:=false show_window:=false
```

脚本加载 `/opt/ros/humble` 和 工作区 `install/setup.bash`，使用 [gazebo_launch.yaml](../config/gazebo_launch.yaml) 启动总入口。脚本先确认 ROS 域没有现有节点，然后等待控制器和命名姿态接口，移动到 `pose_1`；随后读取节点实际的 Action 名称并发送分拣任务。分拣策略和颜色顺序由规划器 YAML 决定，每次抓取前规划器仍会进入观察位。脚本检查 Action 状态和 `success`，任务成功后保留仿真，按 Ctrl+C 退出。

`--headless` 隐藏 Gazebo 和 RViz，`--no-debug-window` 隐藏颜色窗口，`--config FILE` 及 `--planner-config FILE` 可指定配置；`--help` 显示完整用法。`--` 后可传入 launch 参数，也支持直接传入 `gui:=false` 等参数。环境变量 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 默认分别为 227、1；Action 名称通过 launch 参数或 YAML 设置。无显示服务时相机仍需虚拟显示，例如 `xvfb-run -a ./script/pick_three_colors.sh --headless --no-debug-window`。

Gazebo 服务端默认使用 `http://127.0.0.1:11345`，它与 ROS 域编号是两个独立设置。端口已有服务时，启动会提前报错；使用 `ss -ltnp` 检查占用，确认后停止对应的旧仿真，或为新仿真设置独立 `GAZEBO_MASTER_URI`。

## 三色物理验收

先按[项目 README](../README.md) 安装依赖并完成 `colcon build`，从工作区运行：

```bash
./script/run_sorting.sh gui color_order 3,1,2  # 蓝 → 红 → 绿
./script/run_sorting.sh headless nearest
./script/run_sorting.sh --help
```

默认参数为 `gui color_order 1,2,3`。红=1、绿=2、蓝=3；颜色可以只选一部分，但不能重复。脚本可从其他目录通过绝对路径调用。

`run_sorting.sh` 调用 [`test/sorting_run.py`](../test/sorting_run.py)，依次准备临时场景、启动仿真、等待控制器及 Action、生成物块、发送任务并验收。GUI 模式任务结束后保留窗口，按 Ctrl+C 清理本次仿真；headless 模式验收后退出。无 `DISPLAY` 时通过 `xvfb-run` 渲染相机，需先安装 `xvfb`。即便不显示 Gazebo/RViz，配置中的 OpenCV 检测窗口仍需要显示环境。

## 通信、输出和退出

- 默认 ROS 域 **95**、Gazebo 端口 **11495**、仅本机通信；同一入口不要同时运行两份。`--ros-domain-id` 或 `SORTING_ROS_DOMAIN_ID` 可改域，Gazebo 端口仍为 11495。
- 保留当前 RMW；使用 Cyclone 且没有指定 `CYCLONEDDS_URI` 时，补充项目的本机发现配置。
- 输出写入 `log/sorting/<北京时间>/`，可用 `--tag 名称` 指定一个未使用的目录名。
- `bringup.log` 为仿真日志，`spawn.log` 为物块生成日志，`task.log` 为任务反馈，`report.json` 为验收结果，`vision_latest.png` 为最近保存的检测图。
- 当次世界与参数也保存在输出目录，源模型和 `src/` 下的配置不会被覆盖。
- 退出码 **0** 表示请求颜色的任务和物理验收通过；参数错误为 **2**，运行失败或任务中断为 **1**。GUI 在任务通过后按 Ctrl+C 关闭仍返回 0。

在其他已加载 ROS/工作区环境的终端中，使用脚本打印的路径查询：

```bash
source log/sorting/<本次目录>/ros_env.bash
ros2 node list --no-daemon
```

`ros_env.bash` 是当次生成的通信环境片段，不是项目运行入口。不要把整个 `log/` 加入 Git。

## 场景与验收范围

一键回归使用的三色物块为 1 cm，初始 x=0.30 m、红/绿/蓝 y=-0.06/0/0.06 m；目标中心 x=0.26 m、y 不变、z=0.005 m，与手动 launch 当前的默认位置一致。

验收同时检查 Action 成功、物块抬升超过 4 cm、最终位置误差小于 2 cm。位置取自 Gazebo 状态服务，不代表业务节点已实现抓后视觉复核，也不代表实机精度。

原 `test/run_sorting.sh` 已迁移到这里；测试阶段和临时 Python 脚本的复用方法见 [`test/README.md`](../test/README.md)。ArUco 运行入口不在本次目标一的范围内。

## 清理残留进程

```bash
./script/clear.sh --dry-run  # 先查看候选进程
./script/clear.sh            # 尝试停止这些残留进程
```

清理范围与 MuJoCo 分支一致：当前用户、工作目录在本项目内、启动父进程已经退出且被系统接管的已知节点；Gazebo 分支额外识别 `gzserver`、`gzclient` 和物块生成进程。仍由正常启动器管理的进程不会被当作残留处理。工作区外或仍在运行的其他 Gazebo 实例需要确认后自行停止，脚本不会按进程名全局清理。
