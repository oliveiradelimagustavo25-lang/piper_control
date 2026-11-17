# GraspNet 权重记录

| 项 | 值 |
| --- | --- |
| 文件 | `artifacts/checkpoint-rs.tar` |
| 来源 | `~/graspnet/checkpoints/checkpoint-rs.tar`（GraspNet-baseline 官方发布权重） |
| 上游代码 | `third_party/graspnet-baseline/`，commit `280c215`（origin/main） |
| 训练域 | RealSense 相机（`checkpoint-rs`），2026-11-17 推理实测 epoch 18 |
| sha256 | `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868` |
| 大小 | 12 MB |
| 许可 | 随上游 LICENSE：学术/非营利非商业研究用途 |

## 使用约束

- 该权重对应 RealSense 域（0.3–1.6 m、1280×720）；本项目相机为 0.19 m 俯视、
  800×600、fx≈429，属于域外使用，候选质量以 `test/graspnet_eval.py` 的对照
  实测为准，不做先验假设。
- `predict.py` 以 `weights_only=True` 加载；权重文件来源变更时必须重算 sha256
  并更新本表。

## 获取与安装

权重被 Git 忽略，克隆仓库不会获得该文件。请从上游官方说明的预训练模型入口取得
RealSense 权重，手动放到 `tools/graspnet/models/artifacts/checkpoint-rs.tar`，或通过
`--weights` 指定已有文件：

- 官方项目与预训练权重入口：https://github.com/graspnet/graspnet-baseline#installation
- 校验命令：`sha256sum tools/graspnet/models/artifacts/checkpoint-rs.tar`

本地验证用的 SHA256 为上表值。下载得到不同校验值时先确认权重版本与来源，不静默
替换记录。当前未提供本项目的 Release 镜像或自动下载脚本，不能声称模型已随仓库分发。
