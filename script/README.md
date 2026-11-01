# 运行入口

本目录集中保存手写 `.sh`。Shell 只加载环境、定位工作区和传递参数；运行流程放在 `test/` 的 Python 模块中。

## 三色分拣

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

一键回归使用的三色物块为 1 cm，初始 x=0.30 m、红/绿/蓝 y=-0.06/0/0.06 m；目标中心 x=0.26 m、y 不变、z=0.005 m。这与手动 launch 的默认场景不同。

验收同时检查 Action 成功、物块抬升超过 4 cm、最终位置误差小于 2 cm。位置取自 Gazebo 状态服务，不代表业务节点已实现抓后视觉复核，也不代表实机精度。

原 `test/run_sorting.sh` 已迁移到这里；测试阶段和临时 Python 脚本的复用方法见 [`test/README.md`](../test/README.md)。ArUco 运行入口不在本次目标一的范围内。
