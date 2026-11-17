#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <my_robot_interface/action/pick_place.hpp>
#include <my_robot_interface/action/move_to_named.hpp>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "my_robot_commander/motion_controller.hpp"
#include "my_robot_commander/pick_place_scene.hpp"

namespace
{
template<typename T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, T fallback)
{
  return node->has_parameter(name) ? node->get_parameter(name).get_value<T>() :
         node->declare_parameter<T>(name, fallback);
}

tf2::Vector3 vector3(const std::vector<double> & values)
{
  if (values.size() != 3) {
    throw std::invalid_argument("Expected an xyz or rpy parameter with 3 values");
  }
  return {values[0], values[1], values[2]};
}

// 输入是夹持中心，MoveIt 目标是 link6 原点，两者相差一个旋转后的 TCP 偏移。
geometry_msgs::msg::PoseStamped toolPose(
  const tf2::Vector3 & tcp, const tf2::Quaternion & rotation,
  const tf2::Vector3 & tool_to_tcp, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  const auto origin = tcp - tf2::quatRotate(rotation, tool_to_tcp);
  result.pose.position.x = origin.x();
  result.pose.position.y = origin.y();
  result.pose.position.z = origin.z();
  result.pose.orientation = tf2::toMsg(rotation);
  return result;
}

// 盒子在夹爪局部 Y（闭合方向）上的投影宽度。
double graspWidth(const geometry_msgs::msg::Pose & object,
  const geometry_msgs::msg::Vector3 & dimensions, const tf2::Quaternion & tool_rotation)
{
  tf2::Quaternion object_rotation;
  tf2::fromMsg(object.orientation, object_rotation);
  object_rotation.normalize();
  const tf2::Matrix3x3 relative(tool_rotation.inverse() * object_rotation);
  return std::abs(relative[1][0]) * dimensions.x +
         std::abs(relative[1][1]) * dimensions.y +
         std::abs(relative[1][2]) * dimensions.z;
}
}  // namespace

class PickPlaceExecutor
{
  using Action = my_robot_interface::action::PickPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using NamedAction = my_robot_interface::action::MoveToNamed;
  using NamedHandle = rclcpp_action::ServerGoalHandle<NamedAction>;
  using Motion = my_robot_commander::MotionController;
  using Scene = my_robot_commander::PickPlaceScene;

public:
  explicit PickPlaceExecutor(const rclcpp::Node::SharedPtr & node)
  : node_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    Motion::Options options;
    options.arm_group = parameter(node_, "arm_group", options.arm_group);
    options.gripper_group = parameter(node_, "gripper_group", options.gripper_group);
    frame_ = options.reference_frame = parameter(node_, "reference_frame", options.reference_frame);
    tool_frame_ = options.end_effector_link = parameter<std::string>(node_, "end_effector_link", "link6");
    options.velocity_scaling = parameter(node_, "velocity_scaling", options.velocity_scaling);
    options.acceleration_scaling = parameter(node_, "acceleration_scaling", options.acceleration_scaling);
    options.planning_time = parameter(node_, "planning_time", options.planning_time);
    options.cartesian_step = parameter(node_, "cartesian_step", options.cartesian_step);
    options.ik_timeout = parameter(node_, "ik_timeout", options.ik_timeout);
    tool_to_tcp_ = vector3(parameter<std::vector<double>>(node_, "tool_to_tcp", {0, 0, 0.132}));
    offset_ = vector3(parameter<std::vector<double>>(node_, "grasp_offset", {0, 0, 0}));
    const auto rpy = vector3(parameter<std::vector<double>>(
        node_, "grasp_rpy", {0, 3.141592653589793, 0}));
    tool_rotation_.setRPY(rpy.x(), rpy.y(), rpy.z());
    approach_ = parameter(node_, "approach_distance", 0.04);
    // 候选表面种子点 → 夹持中心的内移量：GraspNet 参考点落在物体表面，
    // 而 Piper 指尖仅超出夹持中心 3.8 mm，直接当夹持中心会停在表面上抓空。
    candidate_depth_offset_ = parameter(node_, "candidate_depth_offset", 0.005);
    // 抓取坐标系（+x 开合轴）→ 工具坐标系（+y 开合轴，Piper 手指沿 link6 ±Y 滑动）的绕自身 z 轴旋转。
    const double candidate_roll_deg = parameter(node_, "candidate_roll_deg", -90.0);
    candidate_roll_.setRPY(0.0, 0.0, candidate_roll_deg * M_PI / 180.0);
    // 候选开口的粗筛倍数：模型 width 系统性偏大（对 30 mm 方块输出 78–88 mm），
    // 按同一阈值卡会拒掉全部候选，所以只拦远超出夹爪能力的值。
    candidate_width_limit_ = parameter(node_, "candidate_width_limit", 1.5);
    // 姿态准入：路径已沿接近轴退出/进给，不再假设竖直，故只拦极端倾斜。
    candidate_max_tilt_deg_ = parameter(node_, "candidate_max_tilt_deg", 45.0);
    // 偏心准入：候选索引点与物体观测中心的水平距离。当前候选实测落 11–21 mm，
    // 阈值取 20 mm 才能观察到斜抓效果；该值尚未按抓取成功率标定。
    candidate_max_offset_ = parameter(node_, "candidate_max_offset", 0.02);
    // 隔离实验开关：只取候选姿态，位置改用观测中心。
    // 用于区分“抓不中是因为参考点偏心”还是“倾斜姿态本身抓不住”，非正式配置。
    candidate_centre_snap_ = parameter(node_, "candidate_centre_snap", false);
    lift_ = parameter(node_, "lift_distance", 0.06);
    transport_z_ = parameter(node_, "transport_height", 0.08);
    clearance_ = parameter(node_, "release_clearance", 0.002);
    retreat_ = parameter(node_, "retreat_distance", 0.04);
    open_width_ = parameter(node_, "gripper_open_width", 0.07);
    grasp_width_ = parameter(node_, "gripper_grasp_width", 0.03);
    release_margin_ = parameter(node_, "gripper_release_margin", 0.006);
    compression_ = parameter(node_, "gripper_compression", 0.002);
    signs_ = parameter<std::vector<double>>(node_, "gripper_joint_signs", {1.0, -1.0});
    max_age_ = parameter(node_, "max_target_age", 5.0);
    tf_timeout_ = parameter(node_, "tf_timeout", 0.3);
    if (signs_.size() != 2 || open_width_ <= 0 || compression_ < 0 ||
      !std::isfinite(grasp_width_) || grasp_width_ <= 0 || grasp_width_ >= open_width_ ||
      !std::isfinite(release_margin_) || release_margin_ <= 0 ||
      approach_ <= 0 || lift_ <= 0 || retreat_ <= 0 || clearance_ < 0)
    {
      throw std::invalid_argument("Invalid gripper configuration or approach/lift/retreat distances");
    }
    if (!std::isfinite(candidate_depth_offset_) || candidate_depth_offset_ < 0) {
      throw std::invalid_argument("candidate_depth_offset must be non-negative");
    }
    if (!std::isfinite(candidate_width_limit_) || candidate_width_limit_ <= 1.0) {
      throw std::invalid_argument("candidate_width_limit must exceed 1");
    }
    if (!std::isfinite(candidate_max_tilt_deg_) || candidate_max_tilt_deg_ <= 0.0 ||
      candidate_max_tilt_deg_ >= 90.0)
    {
      throw std::invalid_argument("candidate_max_tilt_deg must be within (0, 90)");
    }
    if (!std::isfinite(candidate_max_offset_) || candidate_max_offset_ < 0.0) {
      throw std::invalid_argument("candidate_max_offset must be non-negative");
    }
    motion_ = std::make_unique<Motion>(node_, options);
    scene_ = std::make_unique<Scene>(node_, tf_buffer_, frame_, options.end_effector_link);
    objects_sub_ = node_->create_subscription<my_robot_interface::msg::DetectedObjectArray>(
      parameter<std::string>(node_, "objects_topic", "/perception/objects"), 10,
      [this](my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observations_mutex_);
        observations_ = msg;
      });

    const auto name = parameter<std::string>(node_, "action_name", "/manipulation/pick_place");
    server_ = rclcpp_action::create_server<Action>(node_, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        // 共享状态只在持有 busy_ 后读写；重排后拒绝分支必须归还，否则永久锁死。
        if (busy_.exchange(true)) {
          RCLCPP_WARN(node_->get_logger(), "Rejected busy goal");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (!validGoal(*goal) || holding_ || scene_->holding()) {
          busy_ = false;
          RCLCPP_WARN(node_->get_logger(), "Rejected invalid goal or object still held");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        // 当前运动调用为同步执行：接受取消，在当前动作结束后停止后续动作。
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        if (worker_.joinable()) {
          worker_.join();
        }
        worker_ = std::thread([this, handle]() { execute(handle); });
      });
    RCLCPP_INFO(node_->get_logger(), "PickPlace executor ready: %s", name.c_str());
    named_server_ = rclcpp_action::create_server<NamedAction>(node_,
      parameter<std::string>(node_, "move_to_named_action", "/manipulation/move_to_named"),
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NamedAction::Goal> goal) {
        if (busy_.exchange(true)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->target.empty()) {
          busy_ = false;
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<NamedHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<NamedHandle> handle) {
        if (worker_.joinable()) { worker_.join(); }
        worker_ = std::thread([this, handle]() { executeNamed(handle); });
      });

    // If an action stops with a carried object, release it explicitly after recovery.
    recovery_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    release_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/manipulation/release_held_object",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (busy_.exchange(true)) {
          response->message = "Executor is busy";
          return;
        }
        try {
          const bool loaded = holding_ || scene_->holding();
          double width = open_width_;
          bool scene_only = false;
          // 带载时先沿已配置的目标卸载；不在失败点直接松爪。
          if (loaded) {
            if (!recovery_) {
              throw std::runtime_error(
                "Load present without a recovery target; move the arm manually before releasing");
            }
            const auto target = *recovery_;
            scene_only = target.scene_only;
            // scene_only 表示夹爪已张开、只剩 MoveIt 附着；重走放置路径会去卸载早已落定的物块。
            if (!scene_only) {
              const auto transform = tf_buffer_.lookupTransform(frame_, tool_frame_, tf2::TimePointZero);
              tf2::Transform tool;
              tf2::fromMsg(transform.transform, tool);
              auto lifted = tool * tool_to_tcp_;
              lifted.setZ(std::max(lifted.z(), target.above.z()));
              // 恢复路径沿用抓取时的姿态：物体仍被夹住，不能相对夹爪转动。
              if (!motion_->moveToPose(toolPose(lifted, target.rotation, tool_to_tcp_, frame_), true) ||
                !motion_->moveToPose(toolPose(target.above, target.rotation, tool_to_tcp_, frame_)) ||
                !motion_->moveToPose(toolPose(target.tcp, target.rotation, tool_to_tcp_, frame_), true))
              {
                throw std::runtime_error("Cannot reach recovery placement; load retained");
              }
              width = target.release_width;
            }
          }
          if (!scene_only && !motion_->moveGripperToJoints(gripperJoints(width))) {
            throw std::runtime_error("Opening gripper failed");
          }
          if (loaded && !scene_only) {
            // 张爪命令已完成；场景更新失败时重试只处理附着状态。
            holding_ = false;
            recovery_->scene_only = true;
          }
          if (scene_->holding()) {
            try {
              scene_->release();
            } catch (const std::exception & error) {
              throw std::runtime_error(
                (scene_only ? "Stale MoveIt attachment could not be released: "
                 : "Gripper opened but MoveIt attachment release failed: ") + std::string(error.what()));
            }
          }
          holding_ = false;
          scene_->restoreTouch();
          recovery_.reset();
          response->success = true;
          response->message = scene_only ?
            "Gripper was already open; stale MoveIt attachment cleared without moving the arm" :
            "Recovery placement/opening completed; verify using fresh observations";
        } catch (const std::exception & error) {
          response->message = error.what();
        }
        busy_ = false;
      }, rmw_qos_profile_services_default, recovery_group_);
  }

  ~PickPlaceExecutor()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  std::vector<double> gripperJoints(double width) const
  {
    std::vector<double> joints;
    for (const auto sign : signs_) {joints.push_back(sign * width / 2.0);}
    return joints;
  }

  // Cache perception callbacks; apply a consistent snapshot before each motion.
  void syncScene()
  {
    my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr observations;
    {
      std::lock_guard<std::mutex> lock(observations_mutex_);
      observations = observations_;
    }
    scene_->sync(observations);
  }

  void executeNamed(const std::shared_ptr<NamedHandle> & handle)
  {
    auto result = std::make_shared<NamedAction::Result>();
    try {
      syncScene();
      if (!handle->is_canceling()) {
        result->success = motion_->moveToNamedTarget(handle->get_goal()->target);
      }
      result->message = result->success ? "Named target reached" : "Named motion failed";
    } catch (const std::exception & error) {
      result->message = error.what();
    }
    busy_ = false;
    if (rclcpp::ok()) {
      if (handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled after current named motion";
        handle->canceled(result);
      } else if (result->success) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
    }
  }

  bool validGoal(const Action::Goal & goal) const
  {
    const auto pose_valid = [](const geometry_msgs::msg::PoseStamped & p) {
      const auto & v = p.pose.position;
      const auto & q = p.pose.orientation;
      return !p.header.frame_id.empty() && std::isfinite(v.x) && std::isfinite(v.y) &&
             std::isfinite(v.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
             std::isfinite(q.z) && std::isfinite(q.w) &&
             std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w)) > 1e-9;
    };
    const auto & d = goal.object_dimensions;
    return !goal.object_id.empty() && pose_valid(goal.object_pose) && pose_valid(goal.place_pose) &&
           std::isfinite(d.x) && d.x > 0 && std::isfinite(d.y) && d.y > 0 &&
           std::isfinite(d.z) && d.z > 0;
  }

  geometry_msgs::msg::PoseStamped inBase(const geometry_msgs::msg::PoseStamped & pose)
  {
    auto result = pose.header.frame_id == frame_ ? pose :
      tf_buffer_.transform(pose, frame_, tf2::durationFromSec(tf_timeout_));
    tf2::Quaternion q;
    tf2::fromMsg(result.pose.orientation, q);
    q.normalize();
    result.pose.orientation = tf2::toMsg(q);
    return result;
  }

  // 本次抓取的完整目标：姿态、夹持中心与闭合开口。
  // 候选存在时整条透传；候选缺失或本身不合法时退回固定向下，并在日志里留因。
  struct GraspTarget
  {
    tf2::Quaternion rotation;
    tf2::Vector3 point;
    double closed_width = 0.0;
    std::string source;
  };

  GraspTarget resolveGrasp(const Action::Goal & goal, const geometry_msgs::msg::PoseStamped & object)
  {
    GraspTarget target;
    target.rotation = tool_rotation_;
    target.closed_width = std::max(0.0, grasp_width_ - compression_);
    target.source = "traditional";
    target.point = tf2::Vector3(object.pose.position.x, object.pose.position.y,
      object.pose.position.z) + offset_;
    if (goal.grasp_pose.header.frame_id.empty()) {return target;}
    try {
      const auto candidate = inBase(goal.grasp_pose);
      if (!std::isfinite(goal.grasp_width) || goal.grasp_width <= 0.0) {
        throw std::runtime_error("candidate opening is not finite or positive");
      }
      // width 是模型的回归值（解码时已含安全放大），与真实开口的关系未标定。
      // 因此不用它做硬拒绝，只排除对任何夹爪都无意义的粗大值。
      if (goal.grasp_width > open_width_ * candidate_width_limit_) {
        throw std::runtime_error("candidate opening far exceeds gripper capability");
      }
      tf2::Quaternion grasp_frame;
      tf2::fromMsg(candidate.pose.orientation, grasp_frame);
      grasp_frame.normalize();
      const tf2::Vector3 approach = tf2::quatRotate(grasp_frame, tf2::Vector3(0, 0, 1));
      // 接近轴偏离竖直的夹角：现有放置路径只会竖直方向几何一致。
      const double tilt = std::acos(std::clamp(-approach.z(), -1.0, 1.0)) * 180.0 / M_PI;
      if (tilt > candidate_max_tilt_deg_) {
        throw std::runtime_error("candidate tilt " + std::to_string(tilt) + " deg exceeds limit");
      }
      const tf2::Vector3 point(candidate.pose.position.x, candidate.pose.position.y,
        candidate.pose.position.z);
      const double offset = std::hypot(
        point.x() - object.pose.position.x, point.y() - object.pose.position.y);
      if (offset > candidate_max_offset_) {
        throw std::runtime_error("candidate lies " + std::to_string(offset) + " m from object centre");
      }
      const auto rotation = (grasp_frame * candidate_roll_).normalized();
      target.rotation = rotation;
      // 参考点落在可见表面上，沿接近轴（手腕→指尖）前进后才是夹持中心；
      // 方向写反会让夹持点停在表面上方，指尖悬空、抓空。
      if (candidate_centre_snap_) {
        target.point = tf2::Vector3(object.pose.position.x, object.pose.position.y,
          object.pose.position.z) + offset_;
      } else {
        target.point = point + approach * candidate_depth_offset_;
      }
      // 候选只能要求更小开口：标称闭合宽度仍是上限，不把预测宽度当作真实物体宽度。
      target.closed_width = std::min(target.closed_width, goal.grasp_width);
      target.source = "graspnet";
      RCLCPP_INFO(node_->get_logger(),
        "Using grasp candidate: tcp (%.4f, %.4f, %.4f) approach (%.3f, %.3f, %.3f) closed %.4f m",
        target.point.x(), target.point.y(), target.point.z(),
        approach.x(), approach.y(), approach.z(), target.closed_width);
    } catch (const std::exception & error) {
      RCLCPP_WARN(node_->get_logger(), "Ignoring grasp candidate: %s", error.what());
    }
    return target;
  }

  // 滚转选择：立方体的开合轴四向等价，而腕关节限位对滚转非常敏感。
  // 实测同一接近轴（20°）同一位置，roll 0° 返回 NO_IK_SOLUTION，而 90/180/270 均可解——
  // 候选被拒的主因是滚转而非倾斜。取第一个预放置位可解的变体。
  tf2::Quaternion selectReachableRoll(
    const GraspTarget & grasp, const tf2::Vector3 & pick, const tf2::Vector3 & put)
  {
    if (grasp.source != "graspnet") {return grasp.rotation;}
    for (const double degrees : {0.0, 90.0, 180.0, 270.0}) {
      tf2::Quaternion spin;
      spin.setRPY(0.0, 0.0, degrees * M_PI / 180.0);
      const auto rotation = (grasp.rotation * spin).normalized();
      const auto approach = tf2::quatRotate(rotation, tf2::Vector3(0, 0, 1));
      const auto cleared = pick - approach * lift_;
      const auto pre_put = put - approach * retreat_;
      const tf2::Vector3 pre_place(pre_put.x(), pre_put.y(),
        std::max({transport_z_, cleared.z(), pre_put.z()}));
      if (motion_->canReachPose(toolPose(pre_place, rotation, tool_to_tcp_, frame_))) {
        if (degrees > 0.0) {
          RCLCPP_INFO(node_->get_logger(), "Candidate roll %.0f deg is reachable; using it", degrees);
        }
        return rotation;
      }
    }
    RCLCPP_WARN(node_->get_logger(), "No reachable roll variant; precheck will decide");
    return grasp.rotation;
  }

  bool interrupted(const std::shared_ptr<GoalHandle> & handle) const
  {
    return !rclcpp::ok() || handle->is_canceling();
  }

  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    auto result = std::make_shared<Action::Result>();
    bool gripper_open = false;
    recovery_.reset();
    // 记录失败发生在哪个预检阶段，随结果返回；运动阶段的失败保持 NONE。
    uint8_t precheck = Action::Result::PRECHECK_NONE;
    auto feedback = std::make_shared<Action::Feedback>();
    const auto stage = [&](uint8_t value, const std::string & message) {
      feedback->stage = value;
      feedback->message = message;
      if (rclcpp::ok()) {
        handle->publish_feedback(feedback);
      }
    };
    const auto step = [&](const std::string & name, const std::function<bool()> & operation) {
      if (interrupted(handle)) {
        throw std::runtime_error("Canceled before " + name);
      }
      if (!operation()) {
        throw std::runtime_error(name + " failed");
      }
    };
    const auto move = [&](const tf2::Vector3 & tcp, const tf2::Quaternion & rotation, bool cartesian) {
      const auto pose = toolPose(tcp, rotation, tool_to_tcp_, frame_);
      if (!cartesian) {return motion_->moveToPose(pose, false);}
      if (motion_->moveToPose(pose, true)) {return true;}
      // 斜姿态下直线插值容易中途碰撞；退到自由规划重试一次，并留下归因。
      RCLCPP_WARN(node_->get_logger(), "Cartesian motion failed; retrying with free planning");
      return motion_->moveToPose(pose, false);
    };
    const auto grip = [&](double width) {
      const bool moved = motion_->moveGripperToJoints(gripperJoints(width));
      if (moved) {gripper_open = std::abs(width - open_width_) < 1e-6;}
      return moved;
    };

    try {
      stage(Action::Feedback::PREPARING, "Preparing object and tool targets");
      precheck = Action::Result::PRECHECK_PRECONDITION;
      const auto goal = handle->get_goal();
      const auto stamp = rclcpp::Time(goal->object_pose.header.stamp, node_->get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 && max_age_ > 0) {
        const double age = (node_->now() - stamp).seconds();
        if (age < -0.1 || age > max_age_) {
          throw std::runtime_error("Object observation is stale or uses a different clock");
        }
      }
      const auto object = inBase(goal->object_pose);
      const auto place = inBase(goal->place_pose);
      const tf2::Vector3 put = tf2::Vector3(place.pose.position.x,
        place.pose.position.y, place.pose.position.z + clearance_) + offset_;
      // 抓取姿态决定开合轴，因此物体投影宽度必须按抓取姿态算，不能固定用向下姿态。
      auto grasp = resolveGrasp(*goal, object);
      grasp.rotation = selectReachableRoll(grasp, grasp.point, put);
      const double width = graspWidth(object.pose, goal->object_dimensions, grasp.rotation);
      if (width >= open_width_) {
        throw std::runtime_error("Object is wider than configured gripper opening");
      }
      const double closed_width = grasp.closed_width;
      const double release_width = std::min(open_width_, std::max(width, grasp_width_) + release_margin_);
      const tf2::Vector3 pick = grasp.point;
      // 固定工具朝向下，搬运不会改变物块姿态。
      tf2::Quaternion object_q, place_q;
      tf2::fromMsg(object.pose.orientation, object_q);
      tf2::fromMsg(place.pose.orientation, place_q);
      if (object_q.angleShortestPath(place_q) > 0.01) {
        throw std::runtime_error("This executor requires matching pick/place object orientation");
      }
      // 前提检查全部通过；此后的失败与预检前提无关。
      precheck = Action::Result::PRECHECK_NONE;
      result->grasp_source_used = grasp.source;
      // 接近方向由当前抓取姿态导出：固定向下时退化为世界 +Z，候选斜抓时沿其接近轴。
      const tf2::Vector3 approach = tf2::quatRotate(grasp.rotation, tf2::Vector3(0, 0, 1));
      // 沿接近轴退开的偏移：斜抓时不再让夹爪横着扫过支撑面。固定向下时与旧行为重合。
      const auto withdraw = [&](const tf2::Vector3 & p, double distance) {return p - approach * distance;};
      const auto pre_pick = withdraw(pick, approach_);
      const auto cleared = withdraw(pick, lift_);
      const auto pre_put = withdraw(put, retreat_);
      const double safe_z = std::max({transport_z_, cleared.z(), pre_put.z()});
      // 退开点抬到搬运平面：先独立退出、再自由规划，避免斜姿态下沿世界 Z 扫碍邻物。
      const tf2::Vector3 lifted(cleared.x(), cleared.y(), safe_z);
      const tf2::Vector3 pre_place(pre_put.x(), pre_put.y(), safe_z);
      recovery_ = RecoveryTarget{put, pre_place, release_width, grasp.rotation};

      syncScene();
      scene_->prepare(goal->object_id, object, goal->object_dimensions, goal->class_id);

      stage(Action::Feedback::PICKING, "Opening, approaching and gripping " + goal->object_id);
      step("Open gripper", [&]() { return grip(open_width_); });
      auto placed_object = place;
      placed_object.pose.position.z += clearance_;
      const auto pre_place_tool = toolPose(pre_place, grasp.rotation, tool_to_tcp_, frame_);
      const auto put_tool = toolPose(put, grasp.rotation, tool_to_tcp_, frame_);
      // 单次 IK 未求出不代表物理不可达：记录阶段与工具目标位姿，供对照实验归因。
      const auto logTarget = [&](const char * stage_name, const geometry_msgs::msg::PoseStamped & pose) {
        RCLCPP_WARN(node_->get_logger(),
          "%s failed: tool pose (%.4f, %.4f, %.4f) quat (%.3f, %.3f, %.3f, %.3f) in %s",
          stage_name, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
          pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
          pose.pose.orientation.w, pose.header.frame_id.c_str());
      };
      precheck = Action::Result::PRECHECK_PRE_GRASP_IK;
      if (!motion_->canReachPose(pre_place_tool)) {
        logTarget("Pre-grasp placement IK", pre_place_tool);
        result->unreachable_placement = true;
        throw std::runtime_error("Pre-grasp placement IK failed; choose another position before picking");
      }
      precheck = Action::Result::PRECHECK_PLACE_IK;
      const auto release = motion_->canReleaseAt(put_tool, gripperJoints(closed_width),
        gripperJoints(release_width), scene_->placementScene(placed_object));
      if (release != Motion::ReleaseCheck::OK) {
        if (release == Motion::ReleaseCheck::BLOCKED) {
          precheck = Action::Result::PRECHECK_RELEASE_SPACE;
          logTarget("Placement release space", put_tool);
          result->unreachable_placement = true;
          throw std::runtime_error("Gripper release space unavailable; choose another position before picking");
        }
        logTarget("Placement IK", put_tool);
        result->unreachable_placement = true;
        throw std::runtime_error("Placement IK failed; choose another position before picking");
      }
      RCLCPP_INFO(node_->get_logger(), "Placement release checked: width=%.4f m", release_width);
      // 两项预检都通过；此后的失败与放置预检无关。
      precheck = Action::Result::PRECHECK_NONE;
      step("Move to pre-grasp", [&]() { return move(pre_pick, grasp.rotation, false); });
      step("Descend to grasp", [&]() { return move(pick, grasp.rotation, true); });
      step("Close gripper", [&]() {
        // The MoveIt attachment is a planning model, not proof of a physical grasp.
        scene_->attach();
        holding_ = true;
        return grip(closed_width);
      });

      stage(Action::Feedback::TRANSFERRING, "Withdrawing, lifting and transferring");
      // 先沿接近轴退出夹持点：斜姿态下沿世界 Z 抬升等于用指板横扫物体与邻物。
      step("Withdraw from grasp", [&]() { return move(cleared, grasp.rotation, true); });
      step("Lift object", [&]() { return move(lifted, grasp.rotation, false); });
      step("Transfer object", [&]() { return move(pre_place, grasp.rotation, false); });

      stage(Action::Feedback::PLACING, "Lowering, releasing and retreating");
      // 下降分两段：先竖直落到沿轴接近点，再沿接近轴进给到放置位。
      step("Approach placement", [&]() { return move(pre_put, grasp.rotation, true); });
      step("Descend to place", [&]() { return move(put, grasp.rotation, true); });
      step("Release object", [&]() {
        if (!grip(release_width)) {return false;}
        gripper_open = true;  // 足够释放当前物体即可，不要求全开。
        holding_ = false;
        // 物理负载已结束；此后若场景解除附着失败，只剩 MoveIt 失步，不该重走放置路径。
        *recovery_ = RecoveryTarget{put, pre_place, release_width, grasp.rotation, true};
        try {
          scene_->release();
        } catch (const std::exception & error) {
          throw std::runtime_error(
            "Gripper opened but MoveIt attachment release failed: " + std::string(error.what()));
        }
        result->released = true;
        return true;
      });
      step("Retreat", [&]() { return move(pre_put, grasp.rotation, true); });
      result->success = true;
      result->message = "Motion sequence completed; object placement needs visual verification";
    } catch (const std::exception & error) {
      result->message = error.what();
      RCLCPP_WARN(node_->get_logger(), "%s", error.what());
    }
    bool touch_restored = true;
    try {
      scene_->restoreTouch();
    } catch (const std::exception & error) {
      touch_restored = false;
      result->success = false;
      result->message += "; restoring collision checks failed: " + std::string(error.what());
    }
    result->holding = holding_ || scene_->holding();
    result->ready_for_next_pick = touch_restored && !result->holding && gripper_open;
    result->precheck_failure = precheck;
    // 恢复目标随持物状态存亡：空载不留可用目标，带载保留给恢复服务；须在交还 busy_ 前定稿。
    if (!result->holding) {recovery_.reset();}
    busy_ = false;  // 在结果可见前交还执行权，避免下一请求撞上旧 busy。
    if (rclcpp::ok()) {
      if (handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled after current motion; gripper state retained";
        handle->canceled(result);
      } else if (result->success) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<Motion> motion_;
  std::unique_ptr<Scene> scene_;
  std::mutex observations_mutex_;
  my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr observations_;
  rclcpp::Subscription<my_robot_interface::msg::DetectedObjectArray>::SharedPtr objects_sub_;
  rclcpp::CallbackGroup::SharedPtr recovery_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Server<NamedAction>::SharedPtr named_server_;
  std::thread worker_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> holding_{false};
  std::string frame_, tool_frame_;
  tf2::Vector3 tool_to_tcp_, offset_;
  tf2::Quaternion tool_rotation_;
  tf2::Quaternion candidate_roll_;
  double candidate_depth_offset_;
  double candidate_width_limit_;
  double candidate_max_tilt_deg_;
  double candidate_max_offset_;
  bool candidate_centre_snap_;
  std::vector<double> signs_;
  double approach_, lift_, transport_z_, clearance_, retreat_;
  double open_width_, grasp_width_, compression_, release_margin_, max_age_, tf_timeout_;
  struct RecoveryTarget
  {
    tf2::Vector3 tcp, above;
    double release_width;
    tf2::Quaternion rotation;
    bool scene_only = false;  // 夹爪已张开，只剩 MoveIt 附着待清理
  };
  // 与恢复服务线程共享：只在持有 busy_ 时读写；随持物状态存亡，确认释放后清除。
  std::optional<RecoveryTarget> recovery_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_place_executor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&]() { executor.spin(); });
  int result = 0;
  try {
    PickPlaceExecutor server(node);
    spin_thread.join();
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "%s", error.what());
      result = 1;
    }
  }
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
