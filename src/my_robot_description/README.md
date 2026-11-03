# my_robot_description

Piper 机器人本体资源包，供 ROS 描述和 MuJoCo 静态预览使用：

- `urdf/piper_description.urdf`：从 `demo_robot_pkg` 迁移，包路径改为本包；保留关节、惯量、相机及原始视觉/碰撞引用。
- `meshes/`：URDF 引用的网格，以及供 MuJoCo 加载的相机与支架转换网格；原 DAE 保留给 URDF。
- `mjcf/piper.xml`：由 URDF 转换的机器人本体和 TCP。场景入口 `preview.xml` 已迁移到 `my_robot_world/basic/`。
- `rviz/preview.rviz`：RViz 机器人模型查看配置。

相机与支架的 MuJoCo 网格转换使用 Assimp 的 `IMPORT_COLLADA_IGNORE_UP_DIRECTION=1`，与 RViz 的 DAE 导入规则一致，同时保留节点变换和单位；不再叠加额外的上轴旋转。安装位置来自 URDF，转换修正使外壳恢复到该安装坐标下；尚不代表已完成实物外参标定。

测试和显示命令见工作区 [test/README.md](../../test/README.md)。本包的 `mjcf/control.xml` 为机器人本体叠加仿真执行器；控制器配置和运行入口位于 bringup。MuJoCo 中使用网格碰撞时会采用凸包，不能认为接触表现已与原模型一致；摩擦、惯量及抓取效果需在后续物理场景中验证。

原 `demo_robot_pkg` 在工作区中已删除，现有 ROS 包对旧包的引用仍待清理。当前只迁移描述文件实际引用的网格，其他旧资源可从 Git 历史核对。
