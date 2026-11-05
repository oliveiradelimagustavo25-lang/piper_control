# 测试与分阶段验证

测试入口负责选择阶段和管理生命周期；`stages/` 保存检查步骤，`support/` 提供 ROS 客户端、模拟环境、临时场景与进程管理。导入模块不会自动创建日志、设置通信环境、启动节点或发送运动。

## 前提

按[项目 README](../README.md) 安装并构建依赖，每个测试终端加载：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Python 测试不需要随 `colcon` 安装，修改后直接运行；C++ 或 ROS 配置变更仍需重新构建。`COLCON_IGNORE` 防止 `colcon` 把本目录内的独立 C++ 探针当成工作区包。

`planner_check.py`、`executor_check.py` 和 `sorting_run.py` 的 `--help` 不加载 ROS 类型支持。

## 目录职责

| 路径 | 内容 |
| --- | --- |
| `robot_description.py` | RViz 中查看机器人及关节滑块 |
| `gazebo_world.py` | 只显示 Gazebo 世界和三个物块 |
| `robot_control.py` | 机器人、Gazebo 控制器与 MoveIt，不生成物块 |
| `planner_check.py` | 任务/深度感知总入口，可选择阶段或单个用例 |
| `executor_check.py` | Gazebo 执行器总入口，可选择独立阶段 |
| `sorting_run.py` | 完整三色分拣入口，由 `script/run_sorting.sh` 调用 |
| `tooling_check.py` | 无 ROS 的导入与进程清理回归 |
| `stages/planner_checks.py` | 策略、观测更新、错误传播、取消等 15 个任务用例 |
| `stages/perception_checks.py` | 合成标签/深度的中心定位用例，覆盖浮点/整型深度与空观测 |
| `stages/executor_*.py` | 协议、运动探针、碰撞与取消、空载/物理抓放和恢复 |
| `stages/sorting.py` | 三色运行的就绪、生成、执行、采样及验收 |
| `support/planner_environment.py` | 模拟执行端、可控观测与 TF |
| `support/planner_fixture.py` | unittest 初始化、资源释放和请求辅助方法 |
| `support/executor_session.py` | 执行器 ROS 连接、物体状态和物理采样 |
| `support/scenarios.py` | 测试专用世界、控制器、模型与参数生成 |
| `support/ros.py` | 有超时的 ROS 等待及控制器就绪检查 |
| `support/runtime.py` | 输出目录、通信环境和本次进程组清理 |
| `probes/motion_controller/` | 独立 C++ 源码与 CMake，直接验证运动库 |
| `local/` | 自己的临时实验脚本，已忽略，不存放正式测试 |

## 任务与感知：不驱动机械臂

```bash
python3 test/planner_check.py                         # 全部 16 项
python3 test/planner_check.py --stage planner         # 15 项任务接口
python3 test/planner_check.py --stage perception      # 仅深度中心定位
python3 test/planner_check.py PlannerChecks.test_nearest_rechecks_current_tcp
python3 test/planner_check.py PerceptionChecks.test_depth_surface_to_center
```

任务用例启动真实任务规划节点，以模拟的执行 Action 响应它；感知阶段只启动深度定位节点及测试发布端。感知用例使用合成标签图，不包含真实相机或 HSV 检测的完整验证。

使用域 **94**、Fast DDS、系统时间，输出为 `log/planner_validation/<时间>/`。`report.json` 记录运行项数、失败和错误；单用例名称优先于 `--stage` 选择。深度用例现在属于 `PerceptionChecks`。

## 执行器：会驱动本次 Gazebo 仿真

```bash
python3 test/executor_check.py --mode headless             # 全部阶段
python3 test/executor_check.py --stage contract             # 仅 Action 协议
python3 test/executor_check.py --stage scene                # 碰撞与运动中取消
python3 test/executor_check.py --stage physical --cube-size 0.01
python3 test/executor_check.py --stage recovery              # 独立创建物块并验证带载恢复
python3 test/executor_check.py --mode gui --stage motion
```

| 阶段 | 做什么 | 前置处理 |
| --- | --- | --- |
| `ready` | 检查两个 Action、三个控制器与关节状态 | 所有阶段自动执行 |
| `motion` | 15 项 C++ 运动库直接调用 | 在 `build/motion_controller_probe` 构建探针 |
| `contract` | 无效目标、未来时间戳、忙状态与取消协议 | 不要求成功执行实际轨迹 |
| `scene` | 导入观测、检查地面、运动中取消及障碍阻挡 | 使用独立仿真，不依赖此前阶段 |
| `air` | 空载抓放，确认四个反馈阶段 | 创建测试用规划场景对象 |
| `physical` | 实际夹持、抬升、释放及位置验收 | 创建测试物块并读取 Gazebo 真值 |
| `recovery` | 夹持后取消、保留负载、拒绝新抓取、显式释放 | 单独运行时自行创建物块；完整回归复用 physical 的物块 |

每次调用独立启动并清理仿真。单阶段报告只代表所选阶段通过，不能当作完整回归结果。GUI 模式在测试结束后同样关闭本次仿真。

默认域 **93**、Gazebo 端口 **11493**、Fast DDS、仅本机通信。不要并行运行两份执行器测试。无显示器时入口会为 headless 仿真添加 `xvfb-run`，需已安装 `xvfb`。

默认物块为 **3 cm**，按体积缩放质量，指尖和物块摩擦系数均为 1。该设置与三色流程的 1 cm 物块不同。保留诊断参数：

```bash
python3 test/executor_check.py --skip-motion --skip-air
python3 test/executor_check.py --stage physical --cube-size 0.03 --friction 1 --tag grasp_check
```

`--skip-motion`、`--skip-air` 只用于完整回归；跳过项在报告中明确标记，不计作通过。`--profile baseline` 用于复现早期参数，可能失败；`current` 为默认值，`ground` 显式指定当前地面抓放参数。其他参数见 `--help`。

输出为 `log/executor_validation/<时间或tag>/`。已有 tag 不会覆盖。除 `report.json` 外，还保留临时世界、模型、控制器、抓放参数、启动日志；物理阶段保存轨迹与接触记录。物理通过条件为抬升大于 4 cm、最终误差小于 2 cm，同时检查搬运附着与释放后的世界场景状态。

## 工具自身的回归

从工作区根目录运行，不依赖 ROS：

```bash
python3 -m unittest test.tooling_check -v
```

检查入口导入不修改环境、不创建目录或进程；失败后回收自身进程且不影响旁边进程；启动父进程提前退出后仍清理同组子进程。

## 组合临时脚本

从工作区根目录以模块方式运行临时脚本，保证能够导入 `test`：

```bash
python3 -m test.local.my_check
```

可以复用 `RosWait`、`Processes`、`ExecutorSession` 或 `SortingCheck`，但由临时脚本明确负责环境、节点和仿真的生命周期。`configure_ros()` 必须在导入 `rclpy` 与 ROS 消息模块之前调用。优先复用现成阶段；确需改变场景时，把新参数写入本次输出目录，不覆盖源文件。

新增正式阶段时写清输入、前置条件、检查依据与报告字段；保持步骤可读，不用一个函数隐式触发其他无关阶段。正式测试及探针建议随 Git 保存，`test/local/`、`log/` 和构建产物继续忽略。
