# GraspNet 外部工具

离线推理与评估，不依赖 ROS。`tools/COLCON_IGNORE` 将其排除出 colcon。
当前交付还不包含 ROS 适配器或常驻 worker；JSON + 子进程是离线评估边界。

## 目录与来源

```text
third_party/graspnet-baseline/
  models/ utils/                  推理及其导入依赖，保留上游结构
  pointnet2/ knn/                  推理所需 Python 包装；原生扩展使用已有 Conda 环境
  LICENSE                         上游许可证
models/README.md                  权重来源和校验值
models/artifacts/                 本地权重，忽略
scripts/predict.py                JSON manifest → results.json + candidates/*.npz
requirements.txt                 本机验证版本记录，非自动安装清单
```

来源为 `~/graspnet/graspnet-baseline`，上游基线 commit `280c215`。
保留模型导入所需的 Python 文件，原生扩展复用已有环境。模型内部文件名如 `loss.py`、
`label_generation.py` 仍是当前导入链的依赖，不按“仅推理”字面直接删除。
上游 LICENSE 随源码保留；使用范围见该文件。

## 使用已有环境

直接复用已安装依赖的 `graspnet` Conda 环境，不重新安装或编译 PointNet2/KNN。
本仓库只保留模型导入所需的 Python 包装；`pointnet2._ext` 与 `knn_pytorch`
由环境中的已安装扩展提供。

```bash
conda run -n graspnet python -c 'import torch, pointnet2._ext; from knn_pytorch import knn_pytorch; from graspnetAPI import GraspGroup; print(torch.__version__)'
```

本目录根部的 `requirements.txt` 仅记录已验证环境版本，不作为从零安装清单，
也不维护第二份上游训练依赖清单。若换新机器，应先按上游说明准备独立推理环境；
本目录不提供原生扩展源码或一键从零构建环境的功能。
权重来源及校验见 `models/README.md`。评估脚本使用系统 Python 的 numpy、PyYAML、
OpenCV；推理进程不依赖 rclpy 或 PyYAML。默认 `log/scenarios` 快照不随 Git 分发，
需使用已采集数据或通过 `--roots` 指定输入。

## 用法

从仓库根目录运行。默认通过 `conda run -n graspnet python` 启动模型，也可设置
`GRASPNET_PYTHON` 或 `--python /path/to/environment/bin/python`；不再硬编码本机解释器路径。

```bash
python3 test/graspnet_eval.py --limit 1 --inputs masked_random
python3 test/graspnet_eval.py
python3 test/graspnet_eval.py --inputs masked_random --collision-cloud masked
# 指定自备快照、模型和解释器
python3 test/graspnet_eval.py --roots /path/to/scenario_batches \
  --weights /path/to/checkpoint-rs.tar --python /path/to/graspnet/bin/python
```

`--roots` 指向包含批次目录的根，结构为 `ROOT/批次/场景运行/snapshots/001/`；
每份快照需 depth.npy、masks.png、objects.yaml、camera_info.yaml、transforms.yaml。
当前读取的是米制 float32 深度和类别编号图，不接受未转换的毫米 PNG。
没有这些输入时会明确报错，不把“没有样本”当成评估通过。

输出位于 `log/graspnet/<时间>/<模式>/`：manifest.json、results.json、predict.log、
candidates/*.npz；顶层为 summary.yaml / summary.md。模式子进程默认最多运行
1800 秒，可用 `--timeout` 修改；CUDA 初始化故障直接报告，不将全部 RuntimeError
当成 GPU 被其他程序占用并反复等待。

## 评估契约与指标

当前是 **类别掩码级** 对照，尚不是针对稳定 `object_id` 的实例级抓取评估。
同类多个物体共用一张类别掩码；实例请求和 Piper 执行将在后续 ROS 适配阶段验证。

输入为 `params + items[]` 的 JSON manifest，包含米制深度、类别图、内参、
`M(base←camera_optical)` 和仿真目标中心。网络输入独立选择
`scene_random / masked_random / masked_voxel`；碰撞点云独立选择 `scene / masked`。
scene 模式保留可见场景点（按上限采样），不代表重建了不可见部分。

schema v2 分别报告两个阶段：

- `after_nms`：场景碰撞过滤 + 正确接收 NMS 返回值，尚未过滤宽度。
- `width_filtered`：在上一阶段上保留 `0 < width <= width_max`，默认上限 0.07 m。
  它表示宽度符合当前阈值，不意味着已完成 Piper 几何标定或可执行性验证。

每个阶段分别计算目标归属、Top-10、宽度、depth 和 approach 分布。
`top1/top5` 仅检查模型抓取参考点到真值中心的三轴距离是否在指定容差内，
是粗略的空间归属代理指标，不代表接触质量、稳定实例关联或抓取成功。
参考点不叫 Piper TCP，不对 width 盲目除以 1.2，不用 depth 推导通用执行规则。

汇总列出总数、有效数、错误数和无候选数；错误项对 top1/top5 按 0 计入分母，
避免丢弃错误使结果变好。发现阶段按最小有效类别像素跳过的项单独统计，
因此这些指标仍是“目标类别可见”条件下的评估，不代表系统整体识别成功率。

计时分为 cloud / forward / post，首次两次预热不计入 forward；模型加载和候选文件
写入不计入这些时间。采样种子由全局 seed 与 item ID 派生，不受 --limit 或遍历顺序影响；
保留种子和版本可重放采样，但不承诺所有 GPU 扩展逐位确定。

## 原始候选与回放

results.json 保存每阶段 Top-10 的完整 base 系旋转矩阵、模型参考点、接近方向、
width、height、depth、score，不进行人为的小数位截断。
每项的 `candidate_arrays` 指向 NPZ，完整保存 `raw / after_collision / after_nms /
width_filtered` 的原生精度数组和 `base_from_camera` 变换。
数组沿用 GraspNet 的 17 列布局：score、width、height、depth、旋转矩阵 9 项、
translation 3 项、模型 object_id（不等于项目实例 ID）。Piper TCP 映射仍需另行标定。
旧 schema 的报告保留供参考，但不能与 v2 的去重、宽度过滤后结果混用。
