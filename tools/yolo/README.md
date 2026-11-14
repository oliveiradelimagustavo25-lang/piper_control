# YOLO 外部工具

本目录不依赖 ROS，可单独构建、安装和处理图片。`tools/COLCON_IGNORE` 将外部工程排除出
colcon；ROS 节点仍在 `src/my_robot_perception/src/yolo_detector.cpp`。
目录中不维护第二份项目类别表：模型输出自己的类别下标与名称，ROS 节点读取共同类别表，
按名称映射到 `class_id`。例如模型下标 0 可以对应 `green_cube`，并不等于项目 ID 0。

## 独立 C++ 推理库

依赖 C++17、OpenCV、yaml-cpp，以及官方 ONNX Runtime C/C++ CPU SDK。
不需要 Python、PyTorch、ROS 或 GPU；当前实现使用 CPU provider。
可参考 [ONNX Runtime C++ 安装说明](https://onnxruntime.ai/docs/get-started/with-cpp.html)。
本次验证使用 Linux x64 SDK 1.23.2；下载和解压到 `.deps/` 后，在工作区根目录执行：

```bash
cmake -S tools/yolo -B tools/yolo/.deps/build \
  -DONNXRUNTIME_ROOT="$PWD/tools/yolo/.deps/onnxruntime-linux-x64-1.23.2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/tools/yolo/.deps/install"
cmake --build tools/yolo/.deps/build -j2
cmake --install tools/yolo/.deps/build

# 不启动 ROS；标签 PNG 为 uint16，0=背景，其他=模型下标+1，并非项目 class_id。
tools/yolo/.deps/install/bin/yolo_predict MODEL.onnx IMAGE.png LABELS.png
```

安装后导出 CMake 包 `yolo_tools` 和目标 `yolo_tools::inference`，调用接口见
`include/yolo_tools/segmenter.hpp`。输入 BGR8，输出原图尺寸的实例掩膜、框、置信度和模型类别下标。
预处理包含 RGB 通道转换、归一化和 letterbox；后处理恢复原图坐标。
安装库的运行路径指向所选 ONNX Runtime SDK；部署到另一台机器时也需安装兼容 SDK，重新构建，
或正确配置其动态库搜索路径。

首版接受静态 batch=1、FP32 NCHW 的 Ultralytics 分割 ONNX，要求包含 `names` 元数据。
支持 raw `[1,4+类别数+mask维度,候选数]` 和端到端 `[1,候选数,6+mask维度]` 输出，
第二输出为 `[1,mask维度,H,W]`。raw 按类做 NMS，端到端不重复 NMS。
输入尺寸从模型读取，不在 ROS YAML 再写一份。训练、导出与部署精度需要分别验证。

## 训练与导出

离线采集使用项目的 MuJoCo 腕相机和 `pose_1` 附近的小幅关节变化；每个场景从 MuJoCo geom segmentation 自动产生实例多边形标签，前 12 张标签叠加图写入 `preview/` 供人工快速核对。类别名称与下标直接从 `object_classes.yaml` 生成。训练依赖仅在 Conda `train` 环境或独立 Python 环境安装，避免改动 ROS 的 Python 环境：

```bash
MUJOCO_GL=egl conda run -n train python tools/yolo/scripts/gen_dataset.py \
  --train 1200 --val 300 --cylinder-weight 4 --seed 28 --output tools/yolo/data/round_02 --force
conda run -n train python tools/yolo/scripts/train.py --epochs 50 --batch 8 --device auto
```

将官方 `yolo26n-seg.pt` 放在 `tools/yolo/models/` 后，脚本会以它为预训练初始化，类别名必须匹配公共类别表。训练集、权重、runs 和 ONNX 模型都是本地产物，由 `.gitignore` 排除，不提交大型数据文件。导出使用训练得到的 `best.pt`：

```bash
PYTHONPATH=tools/yolo/.deps/test-python conda run -n train \
  python tools/yolo/scripts/export.py tools/yolo/runs/round_02_TIMESTAMP/weights/best.pt --raw
```

训练目录按 `round_NN_YYYYMMDD_HHMMSS` 递增；ONNX 文件再包含 epoch 数，例如
`round_02_e050_20261114_150000.onnx`。`manifest.yaml` 指向最新模型，每个版本也保存同名 YAML。

`requirements.txt` 是依赖范围，尚非训练环境的精确版本锁；确定实际训练组合后保存版本记录。
训练数据集 `names` 应与共同类别表的 `name` 对应，而非把项目 ID 当作从零开始的训练下标。
官方 COCO 预训练模型不直接提供红、绿、蓝、黄方块类别，需相应训练/微调。
见 [Ultralytics 分割说明](https://docs.ultralytics.com/tasks/segment)。
导出脚本只接受已存在的本地权重，不负责自动下载；输出模型和可提交的来源记录。

## 接入 ROS

普通 `colcon build` 默认 `BUILD_YOLO=OFF`，CV 不需要此库。安装 SDK 后启用 ROS 适配器：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to my_robot_perception my_robot_bringup \
  --cmake-args -DBUILD_YOLO=ON -Dyolo_tools_DIR="$PWD/tools/yolo/.deps/install/lib/cmake/yolo_tools"
source install/setup.bash
./script/pick_three_colors.sh -- detector:=yolo yolo_model:=/absolute/path/sorting.onnx
```

也可在 `config/mujoco_launch.yaml` 设置 `detector: yolo` 和 `yolo_model`，再使用同一个脚本。
独立节点入口为 `ros2 launch my_robot_perception yolo_detector.launch.py`，接受
`perception_config`、`class_config`、`yolo_model`、`use_sim_time`。
默认配置保持 CV；缺模型/SDK 或类别名称完全不匹配时明确报错，不自动切换回 CV。

配置优先级：显式 launch 参数 > 总配置 YAML > `perception.yaml` > 节点默认值。
总配置的模型路径相对于总配置文件；感知配置的 `model_path` 相对于感知 YAML；
显式 `yolo_model:=...` 相对于命令工作目录。`model_path` 参数显示最终绝对路径。
共同类别表仍使用原有 `class_config` 优先级，CV、YOLO、depth 和 Planner 共用同一份。

正常启动只选一种标签图发布者；`labels_topic` 覆盖同时作用于检测器与 depth。
仿真相机的 RGB 话题跟随所选检测器，显式 `image_topic` 覆盖相机与检测器。
YOLO 保留 RGB 原始时间戳，depth 的 `sync_queue_size` 可容纳推理延迟，`sync_slop` 仍约束采集时间差。
图像订阅只保留最新一帧；推理错误跳过该帧，真实无目标才发布全零标签图。
Planner 的观测时效检查继续生效。未映射到类别表的模型类别在启动时列出并排除。

本阶段输出仍为 `mono8` 项目类别图。同类物体接触会丢失实例边界，检测器替换验证应先用分离物块。
depth 仍只支持顶部可见的平放 box，抓取姿态仍由现有执行器生成；接入模型不自动扩展形状定位或抓取能力。
