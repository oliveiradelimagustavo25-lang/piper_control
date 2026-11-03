# my_robot_world

MuJoCo 世界资源与传感器代码：

```text
basic/preview.xml        # 基础世界组合
scenes/scene.py          # 世界加载与机器人资源解析
sensors/camera.py        # 创建并配置腕部相机
objects/                 # 三色物块和 ArUco 标记板
assets/                  # 标记板图片
config/camera.yaml       # 腕部相机参数
```

安装并加载 ROS 工作区后，可使用 `from my_robot_world.scenes.scene import load_scene`。世界包根目录同时作为 Python 包，通过 CMake 的 `PACKAGE_DIR .` 安装；工作区中的 `test/mujoco_world.py` 也可以直接使用源码运行。

`load_scene(world_dir, description_dir)` 组合场景和机器人，并在 `camera_optical_link` 上添加 MuJoCo 相机。`preview.xml` 中的 `robot_description.xml` 是加载器解析的引用，因此需通过加载器打开，不能直接用 MuJoCo 原生命令加载。物体与纹理路径相对于场景文件解析。

`build_scene` 可选择机器人 MJCF、世界场景和相机配置；无控制预览默认选择 `piper.xml`。正式 bringup 选择 `control.xml`，通过 `export_scene` 展开为临时 MJCF，其中保留地面、物块、标记板、相机、控制执行器及接触配置，并把资源路径转为绝对路径。导出本身不推进物理，随后由单个原生 ros2_control 插件加载和运行。

基础世界包含棋盘格地面。相机只提供 MuJoCo 渲染，不发布 ROS 图像或 CameraInfo。相机和支架网格已统一到 RViz 的坐标规则，默认姿态下可看到标记板，夹爪仍占据部分视场；裁剪范围、深度范围和频率暂为后续配置，尚未接入发布链。
