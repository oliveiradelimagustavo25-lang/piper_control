模型二进制不提交 Git：官方预训练权重由本机保存，微调权重位于
`../runs/round_NN_YYYYMMDD_HHMMSS/weights/best.pt`，部署 ONNX 位于
`artifacts/round_NN_eEEE_YYYYMMDD_HHMMSS.onnx`。`manifest.yaml` 指向最新模型；每个版本的同名
YAML 记录类别、SHA256、导出布局和工具版本。仿真数据训练结果尚未用真实相机图像验证。

`yolo26n.pt` 是 Ultralytics 检查 CUDA 混合精度（AMP）时自动下载的辅助权重，不参与本项目分割训练。train.py 将其下载目录设为本目录，Ultralytics 设置缓存在 ../.deps/config/，不会修改全局设置。
