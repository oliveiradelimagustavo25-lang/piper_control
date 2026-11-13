# 自适应分拣 Planner

沿用一个 `TaskPlanner` 节点和现有 `SortObjects` Action。每次抓放后回观察位，等待新的 RGB-D 观测，再选择一个目标；不维护冻结批次或长期排序队列。缺少指定类别不阻塞其他类别，后来出现的物体按当前策略参与选择。

## 使用

先构建、加载工作区（接口消息已扩展，旧二进制需要重建）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to my_robot_bringup my_robot_planner my_robot_perception my_robot_commander
source install/setup.bash
./script/pick_three_colors.sh
```

默认 `class_order`，按共同类别表顺序；可用 `default_class_order` 筛选并排序，同类按当前 TCP 距离选择，TCP 暂不可用时使用基座距离并告警。`nearest` 在允许类别中按距离选择。Action 新增 `class_order`；旧 `color_order` 仍按颜色筛选后映射到类别，与 `class_order` 不可同时填写。配置在启动时加载。

| 参数 | 行为 |
| --- | --- |
| `finish_mode: batch / watch` | batch 在健康观测下无候选并空闲超时后结束；watch 等待新目标 |
| `idle_timeout` | batch 空闲秒数；0 不自动结束；断流、关联歧义、冷却不算健康空闲 |
| `retry_policy: priority / immediate / disabled` | 按当前策略重选 / 优先重试失败目标 / 不自动重试 |
| `max_attempts` | 每个已关联物体的总尝试次数，包含首次；被执行器拒绝不计数 |
| `retry_cooldown` | 失败后再次参与选择前的等待秒数，默认 0 |
| `on_attempts_exhausted: continue / stop` | 默认保留失败记录并继续其他目标；全部处理完仍有未解决尝试则结果为失败 |
| `workspace_min / workspace_max` | 可选 base_link 下 xyz 边界；两者留空不额外限制，不代表可达保证 |
| `classes.<id>.place_region` | `[xmin, ymin, xmax, ymax, z]`，放置区域与支撑高度；容量不依赖场景物体数量 |
| `placement_step` | 区域内 x/y 间距；以 `classes.<id>.place_position` 为原点生成候选，优先使用原点附近空位 |
| `classes.<id>.place_slots` | 可选手工扁平 xyz 三元组，显式设置时优先；未设置 region 时兼容旧单点 |
| `slot_tolerance` | 当前同类物体中心距槽位的容差，满足则已在目标位置，不重复抓取 |
| `slot_clearance` | 为目标尺寸之外预留的放置净空；实际路径继续由 MoveIt 检查 |
| `on_slots_full: wait / finish` | 默认等待空位，finish 在 batch 空闲结束时报告未完成 |
| `placement_verification: visual / motion` | 默认核验来源消失且空闲目标槽出现同类物块；motion 只计运动，不能证明抓取成功 |
| `max_objects` | 已核验搬运次数上限，0 不限；原本已在目标槽的物体不计本次搬运 |

没有全局“已抓过 ID”黑名单。抓取与放置可位于同一工作区；按当前物体是否已在本类别槽位筛选候选。槽位每轮读取所有有效物体的占用，释放后保留预留记录，避免漏检导致重叠放置；已关联的占用物体明确移开时释放缓存。记录跨 SortObjects 任务保留，但不跨节点重启持久化。槽位被移空而身份无法关联时，保守保留占用；需要新鲜可见观测和明确的场景管理，不能把遮挡当作空位。

视觉核验需要来源和目标位置在观察位可观测。未看到目标位置不等于放置成功：无法确认时保留预留、记录异常，继续有条件处理其他目标。`released` 只表示执行过松爪和模型释放，即使随后撤离失败也先观察核验，避免盲目重抓。

## 逐轴相对 / 绝对放置

`place_position_modes` 按 X/Y/Z 指定 relative/absolute，`place_positions` 按类别 ID 配置中心/偏移。命令行 > 总 YAML > Planner YAML；显式规则整体替换低优先级规则，包括空映射。未定义 ID 被忽略，缺少/无效规则的类别在抓取前跳过，不从区域/其他类别兜底。只有所有层均未提供显式规则时才使用 `classes.<id>.place_position/place_region/place_slots`，缺少这些配置也不发起抓取。

Planner 与 CV/depth 读取同一类别表；内部类别容器支持非连续编号，数量不受类别数限制。可靠的未选中/暂不抓取物体仍参与避障。放置核验和视觉 ID 别名关联要求类别一致；预留记录保存物体的包围尺寸，放置净空使用双方尺寸与朝向计算。

每个目标首次派发前在 `reference_frame` 中计算并冻结绝对中心；重试只刷新抓取位姿，新 SortObjects 任务重新建立基准。核验后记录目标位置，视觉 ID 改变时关联同一记录，防止每一帧继续平移。未确认释放位置仍保留预留；已关联物体确实移开后可以再次处理到同一冻结目标。该行为不解决同外观物体任意交叉/大幅移动下的身份歧义。

同色多个物体应用各自初始中心；不会在本轮自动聚集。若多个 absolute 目标重合或目标被其他物体占用，等待可用空间或按 `on_slots_full` 结束，不强行重叠。

`./script/pick_three_colors.sh --dry-run` 仅显示最终有效规则；相对值没有实时观测时不猜绝对坐标。最终绝对目标在 Planner 日志 `Frozen placement` 中查看，Z 为物体中心而非桌面/TCP。详细示例见 [脚本说明](../../script/README.md#放置坐标规则与优先级)。

## 异常与身份边界

- 感知在 base_link 下按类别、距离进行一对一跨帧关联。轻微移动和漏检保留 ID；不因腕部相机离开观察位自动注销。近似等距匹配通过 `tracking_uncertain` 标记，等待后续可区分的观测。
- 相同外观物体的交叉移动、长距离移动或严重遮挡无法保证物理身份；超出关联距离可能产生新 ID。重试次数按关联 ID 保存，不能把该实现视为任意移动下的严格逐物体计数。需要唯一身份时应使用标记/外观特征。
- 当前检测仍假设已知尺寸、平放、顶部可见、颜色区域可分离。紧贴同色、堆叠、任意形状及未知障碍物识别不在本轮覆盖内。
- `DetectedObjectArray.issues` 区分未知标签、定位不支持、深度不足与健康空场景；有可靠目标时继续处理，batch 结束仍有跳过项/感知问题会报告部分未完成，不计入完成数。
- 非法单条观测被忽略；整帧无有效条目、时间戳/TF 异常、感知断流、服务暂不可用时保留任务并限频告警。`observation_timeout` 现在是等待告警间隔，不再使缺色或短时断流直接结束任务。
- 失败后先确认执行器状态。空载、夹爪已张开且碰撞许可恢复才继续；否则异步请求恢复。带载恢复先尝试送到本次目标卸载，不在故障位置直接松爪。无法确认恢复超过 `recovery_timeout` 时报告失败并保留可能的负载。
- 取消使用 ROS Action 的取消接口；当前同步运动结束后停止后续步骤，保留负载，不自动为取消启动卸载。已开始的恢复服务等待返回后结束取消。动作看门狗请求取消后仍等待执行器状态，不立即派发新目标。
- MoveIt 场景只在动作前同步。仅合并近乎重合的视觉别名，不因物体离开视野就删除碰撞障碍；未实现动态未知环境建图。

## 场景与一键启动

场景在工作区 `config/mujoco_launch.yaml` 的 `scenes` 中按 ID 填写 `[x,y,z]` 列表，XML 由 `script/build/scene_config.py` 按共同类别表自动生成。未填写/null 使用类别表默认位置，显式映射只生成列出的类别；未定义类别跳过并报告。见 [脚本说明](../../script/README.md#场景与一键启动)。

```bash
./script/pick_three_colors.sh
```

planner 不读取场景里的数量。未配置新放置规则时，在各类别 `place_region` 中按统一 `placement_step` 生成候选，按当前观测和预留记录寻找空位；可以整体移动或扩大区域，不用为每个物体新增一个放置坐标。区域范围、可见性、可达性和夹爪净空仍由实际工作台决定，物理空间不够时按 `on_slots_full` 等待或结束。执行器在抓取前检查放置位及其上方的 IK 可达性；不可达候选在本任务内暂不使用，planner 换位置或先处理其他目标，不消耗物体抓取尝试次数。IK 检查不替代后续路径和碰撞检查。

## 验证入口

```bash
# 离线检查数量、默认值、清空类别与显式世界文件优先级
python3 test/probes/check_scene_config.py
python3 test/probes/check_placement_config.py
# 合成数据与模拟执行器，不证明物理抓取
python3 test/probes/check_multi_object_perception.py
python3 test/probes/check_adaptive_planner.py
# 真实运行唯一 .sh 入口，测试输入临时写入总配置，不传 XML
xvfb-run -a python3 test/probes/check_multi_object_sorting.py
xvfb-run -a python3 test/probes/check_multi_object_sorting.py --count 2=4
xvfb-run -a python3 test/probes/check_multi_object_sorting.py --count 1=3 --count 2=1 --count 3=1
```

数量选项只属于测试探针，日常使用直接修改总配置中的坐标列表。探针从运行中 planner 读取生成后的候选位置，按输入计算验收数量，不写死四物体/两槽。合成用例覆盖重试、抓空、移动、重现、取消、不同颜色数量；集成用例通过相机核验，不读取 MuJoCo 运行中物理真值。

测试使用独立空闲域，日志保存到 `log/`，仅清理自身启动链。原抓取偏移额外增加的 2 mm 只适用于当前仿真；碰撞检查保持启用。中途新增/删除物体仍留待后续仿真能力验证。

当前物理验证主要覆盖 x≈0.32 的视野中部。测试曾在 x≈0.38 的边缘抓取点出现夹爪触地，视觉核验正确报告物体仍在来源位置；此类位置仍需校准控制误差或调整观察/抓取姿态，不能用颜色数量配置掩盖。数量探针在已校准范围内按总数均匀排布，复杂边缘位置可通过总配置单独检验。

本轮一键入口验证（2026-11-12）：八包构建、场景配置离线检查、21 个模拟调度用例通过；默认三色核验 3 次、四绿色核验 4 次、三红一绿一蓝核验 5 次。全部通过实际 `pick_three_colors.sh` 启动，未向脚本传 XML，未使用独立场景配置文件。混合场景还验证了先排除不可达候选、处理占位物体、再使用腾出位置的流程。碰撞场景、坏观测隔离与单物抓放回归通过。
