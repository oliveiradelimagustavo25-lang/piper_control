# GraspNet 候选：机械臂可直接执行的完整抓取目标。
# planner 只做查表、透传与记录；姿态解释（接近轴、开合轴）由执行端完成。
# 数据驱动降级：本话题断流时，planner/executor 行为自动退化为传统深度链路。
std_msgs/Header header              # stamp = 候选对应观测帧的时间（新鲜度判据），非发布时间
string object_id                    # 与 /perception/objects 的 object_id 同源，不做第二次关联
geometry_msgs/PoseStamped tcp_pose  # 夹持中心目标位姿（base_link）
                                    # 姿态 = 抓取坐标系：+z 为接近轴（手腕→指尖），
                                    # +x 为开合轴，+y 由右手系确定
float64 width                       # 候选开口，m；仅作为开合指令输入与限位过滤，
                                    # 不直接等同于真实物体宽度
float64 score                       # 模型分；仅记录排序，不参与任何决策
