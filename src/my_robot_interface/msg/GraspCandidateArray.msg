# 一次观测帧上某个物体（或全部物体）的抓取候选集合。
# 顺序即优先级：模型 score 降序。执行端按顺序尝试，取第一个通过全部准入的候选。
std_msgs/Header header              # stamp = 候选对应观测帧的时间
string object_id                    # 非空时本批候选属于同一物体；空表示按物体分组发布
my_robot_interface/GraspCandidate[] candidates
