# my_robot_perception

`color_detector` 做二维颜色分割，`yolo_detector` 调用独立 ONNX 推理库；二者按配置选一个，发布同样的类别标签图。
`depth_localizer` 使用类别区域、对齐深度、CameraInfo 和 TF 定位；`aruco_detector` 独立处理标记板。
YOLO 节点留在本 ROS 包，训练、导出及不依赖 ROS 的推理库在 [tools/yolo](../../tools/yolo/README.md)，分别构建和安装。

## 检测器选择

默认保持 CV，不要求 ONNX Runtime 或模型文件。可在总配置 `config/mujoco_launch.yaml` 设置
`detector: cv` / `detector: yolo`，或通过显式 launch 参数覆盖。YOLO 专用参数放在
`perception.yaml` 的 `yolo_detector` 部分，首次启用需按照工具说明构建外部 SDK 和 `BUILD_YOLO=ON` 的 ROS 适配器。

```bash
./script/pick_three_colors.sh -- detector:=cv
./script/pick_three_colors.sh -- detector:=yolo yolo_model:=/absolute/path/sorting.onnx
ros2 launch my_robot_perception yolo_detector.launch.py \
  yolo_model:=/absolute/path/sorting.onnx use_sim_time:=false
```

配置优先级：显式 launch 参数 > 总配置 YAML > 感知 YAML > 节点默认值。
总配置 `yolo_model` 相对于总 YAML，感知配置 `model_path` 相对于感知 YAML，显式 `yolo_model` 相对于当前目录。
模型的类别下标根据 ONNX `names` 匹配共同类别表的 `name`，不会直接当作项目 ID。
需使用训练了相应物块类别的分割模型，官方 COCO 权重不能直接等同于四色方块模型。
未映射的模型类别被列出并排除，没有任何匹配类别则报告配置错误。

两种检测器都保留原 RGB 时间戳和原图尺寸。`labels_topic` 的显式覆盖同时作用于检测器与 depth；
仿真相机发布到所选检测器的 RGB 话题。depth 的 `sync_queue_size` 控制推理等待缓存，
`sync_slop` 仍只约束采集时间差。YOLO 帧处理失败会跳过该帧并等待后续图像，不发布健康空观测；
真实无目标才发布全零图。Planner 继续检查观测时效。
当前仍用 mono8 类别图，同类物体接触可能合并，尚未升级二维实例协议。

## 共同类别表

[config/object_classes.yaml](config/object_classes.yaml) 是类别编号、名称、分拣颜色、标称尺寸及 HSV 的唯一来源。CV、YOLO、depth 和 MoveIt 碰撞层共用 `include/my_robot_perception/object_catalog.hpp` 解析这张表；算法阈值和话题仍在 [config/perception.yaml](config/perception.yaml)。原来的 `red.lower/upper` 等 HSV 参数和统一 `object_dimensions` 参数已移除，请改类别表。

当前红、绿、蓝、黄方块统一为 3×3×3 cm，完整尺寸为 `[0.03, 0.03, 0.03]` m。黄色为类别 4，HSV 和仿真 RGBA 均在类别表配置；其旧颜色字段为 0，检测和 Planner 使用 class_id=4。默认地面 Z=0，落稳后的物体中心约为 Z=0.015 m；相对放置以实际观测中心计算。

CV 形态学参数在 `perception.yaml`：`opening_kernel: 3` 用于去除小噪点，`closing_kernel: 5` 用于连接小断裂；均为正奇数，1 关闭对应操作。它们替代原先共用的 `morphology_kernel`。闭运算过大会合并相邻同色物体，需要结合图像分辨率和物体间距调整，不能保证遮挡后的实例分离。

- `id`：稳定的类别编号，可不连续；当前 `mono8` 标签图使用 1..255，0 为背景。
- `name`：类别名称；实例 ID 为 `<name>_<序号>`，同类多个物体各有 ID。ID 不是跨遮挡或大幅移动的物理身份保证。
- `color`：可选外观属性；red/green/blue 对应 1/2/3，其他/未知为 0。Planner 按 class_id 工作，不依赖颜色编号。
- `geometry.type/dimensions`：标称包围尺寸，单位 m；圆柱为 `[直径, 直径, 高]`，球为三个相同直径。depth 将掩码深度转换到 Z 向上的 objects_frame，以 90% 分位高度附近的点估计顶部和 XY 中心，顶部高度减标称半高得到中心 Z。附近背景若呈稳定水平面，则检查估计底部是否低于该面；异常区域通过 issues 报告，不生成抓取目标。背景不可靠时不强制依赖平面。顶部完全不可见、倾斜或标称尺寸错误时仍可能有偏差。MoveIt 按类型建立 box/cylinder/sphere，其他类型使用标称包围盒。尺寸不由视觉测量。自动生成的 MuJoCo 模型读取同一尺寸；手工 XML 仍需自行保持一致。
- `cv.hsv_lower/hsv_upper`：OpenCV HSV，H=0..179，S/V=0..255。H 下界大于上界表示跨零点。没有 `cv` 的类别不参与 CV 分割；重叠阈值按列表顺序分配。CV 仅识别颜色，不能区分颜色相同而形状不同的类别。

CV 输出的类别图保留原图尺寸、光学坐标系和采集时间。depth 按表中类别逐个提取连通区域，查询各自标称尺寸，输出中心位姿、`class_id`、`color`、`object_id` 及关联歧义标记。点云中的 `label` 也表示类别编号。本帧未定位的已检出区域通过 `DetectedObjectArray.issues` 报告，避免被误判为清场；未定义类别/无效类别条目只影响自身。其他检测器以后可以遵守这个契约接入；同类物体接触粘连的实例分离尚未实现。

类别名称和几何类型是配置先验，不是视觉形状判别结果。当前 CV 没有圆柱/方块分类算法：红圆柱可能命中红方块的 HSV；为另一颜色配置专属 ID 可以按颜色区分，但同色其他形状也会命中。depth 不按几何类型拒绝定位。当前夹爪统一采用 manipulation.yaml 中的固定朝向、gripper_open_width 和 gripper_grasp_width，闭合目标为 gripper_grasp_width 减去 gripper_compression；释放时另用物体宽度加 gripper_release_margin 的开度（不超过 gripper_open_width）。更换尺寸时需调整默认夹持策略。移除类型过滤不代表已验证各类物体的实际抓取成功率。

## 配置与启动

`perception.yaml` 的 `/**/ros__parameters/class_config` 只写一次，路径相对于该 YAML。完整入口与独立 launch 只传递参数，路径解析和类别读取由检测节点通过公共头文件完成，没有额外 Python 配置模块。

优先级：显式 `class_config:=...` > 总配置 `class_config` > 感知 YAML 的共享 `class_config` > 安装包默认表。总配置中的相对路径相对于总 YAML；显式命令行相对路径相对于当前目录。两个感知节点始终获得同一条解析后的绝对路径，并在启动时打印路径及类别。

默认总配置引用源码中的感知 YAML，因此修改源码类别表后重新运行 `./script/pick_three_colors.sh` 即可，无需重新构建。独立 launch 默认读取安装目录，修改源码后需要重新构建，或显式指定源码配置：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_perception color_detector.launch.py perception_config:="$PWD/src/my_robot_perception/config/perception.yaml"
ros2 launch my_robot_perception depth_localizer.launch.py perception_config:="$PWD/src/my_robot_perception/config/perception.yaml"
ros2 launch my_robot_perception aruco_detector.launch.py

# 在完整入口同时覆盖 CV 和 depth 的类别表
./script/pick_three_colors.sh -- class_config:=/absolute/path/object_classes.yaml
```

上述独立 launch 默认 `use_sim_time:=true`；连接系统时钟的相机需传 `use_sim_time:=false`。depth 仍需标签图、对齐深度、对应内参及采集时刻的 TF；相同尺寸和 frame_id 本身不能证明深度已物理对齐。

直接使用 `ros2 run` 时，`class_config` 相对于 `perception_config` 所指 YAML；未提供 `perception_config` 则相对于当前目录。需要按文件相对解析时，同时传入 `--params-file /path/perception.yaml -p perception_config:=/path/perception.yaml`。launch 内部用 `class_config_override` 保留显式覆盖的来源，节点解析后可通过 `class_config` 参数查看实际绝对路径。

## C++ 节点复用

公共头文件随包安装，并导出带依赖的 CMake 接口目标。其他 C++ 包（包括后续 C++ YOLO 节点）在 `package.xml` 声明对 `my_robot_perception` 的依赖后可使用：

```cmake
find_package(my_robot_perception REQUIRED)
target_link_libraries(yolo_node my_robot_perception::object_catalog)
```

```cpp
#include <my_robot_perception/object_catalog.hpp>
// 在节点初始化中读取公共类别定义。
auto classes = my_robot_perception::loadCatalog(*this);
```

接口要求 C++17，当前返回类别、颜色、标称尺寸和可选 HSV。Python 节点不能直接包含 C++ 头文件；若后续选择 Python YOLO，需读取同一份 YAML 或另做绑定，不会自动获得此 C++ 接口。

## 验证

```bash
python3 test/probes/check_class_catalog.py
xvfb-run -a python3 test/probes/check_multi_object_sorting.py --count 2=4
```

第一个探针运行真实 CV 和 depth 节点，验证四色五实例、时间戳、正常编号及非连续编号、颜色映射和不同标称高度。第二个使用原单脚本入口验证 MuJoCo 抓放。数量没有三个的上限；当前 Planner 的 `max_objects: 0` 表示不限制核验搬运次数，实际受识别、可达性和空间约束。

`test/probes/check_yolo_adapter.py` 使用输入相关的微型 ONNX 图，验证独立工具执行、两种输出布局、
letterbox 恢复、通道顺序、类别名称映射、配置优先级，以及 CV/YOLO 对接同一个真实 depth 节点。
需要测试环境可导入 `onnx`，并事先构建适配器。它验证接口契约，不验证训练模型的识别精度，也不启动机械臂。
