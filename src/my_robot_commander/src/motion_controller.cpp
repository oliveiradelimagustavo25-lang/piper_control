#include "my_robot_commander/motion_controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace my_robot_commander
{
namespace
{
bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}
}  // namespace

struct MotionController::Impl
{
  using Group = moveit::planning_interface::MoveGroupInterface;
  rclcpp::Node::SharedPtr node;
  Options options;
  std::unique_ptr<Group> arm;
  std::unique_ptr<Group> gripper;
  std::mutex mutex;
  std::atomic<bool> stop_requested{false};
  using Execute = moveit_msgs::action::ExecuteTrajectory;
  rclcpp_action::Client<Execute>::SharedPtr execution_client;

  Impl(const rclcpp::Node::SharedPtr & input_node, const Options & input_options)
  : node(input_node), options(input_options)
  {
    if (!node) {
      throw std::invalid_argument("MotionController requires a ROS node");
    }
    if (!(options.velocity_scaling > 0 && options.velocity_scaling <= 1 &&
      options.acceleration_scaling > 0 && options.acceleration_scaling <= 1 &&
      std::isfinite(options.planning_time) && options.planning_time > 0 &&
      std::isfinite(options.cartesian_step) && options.cartesian_step > 0))
    {
      throw std::invalid_argument("Invalid motion scaling, planning time or Cartesian step");
    }
    arm = std::make_unique<Group>(node, options.arm_group);
    gripper = std::make_unique<Group>(node, options.gripper_group);
    execution_client = rclcpp_action::create_client<Execute>(node, "execute_trajectory");
    for (auto * group : {arm.get(), gripper.get()}) {
      group->setMaxVelocityScalingFactor(options.velocity_scaling);
      group->setMaxAccelerationScalingFactor(options.acceleration_scaling);
      group->setPlanningTime(options.planning_time);
    }
    arm->setPoseReferenceFrame(options.reference_frame);
    if (!options.end_effector_link.empty() &&
      !arm->setEndEffectorLink(options.end_effector_link))
    {
      throw std::invalid_argument("Invalid end effector link: " + options.end_effector_link);
    }
  }

  bool planAndExecute(Group & group)
  {
    if (stop_requested) {return false;}
    Group::Plan plan;
    const auto planned = group.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Planning failed, MoveIt code: %d", planned.val);
      return false;
    }
    return execute(plan.trajectory_);
  }

  bool execute(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    using namespace std::chrono_literals;
    if (stop_requested || !execution_client->wait_for_action_server(2s)) {return false;}
    Execute::Goal goal;
    goal.trajectory = trajectory;
    auto accepted = execution_client->async_send_goal(goal);
    while (rclcpp::ok() && accepted.wait_for(20ms) != std::future_status::ready) {}
    if (!rclcpp::ok()) {return false;}
    const auto handle = accepted.get();
    if (!handle) {return false;}
    auto result = execution_client->async_get_result(handle);
    bool cancel_sent = false;
    while (rclcpp::ok() && result.wait_for(20ms) != std::future_status::ready) {
      if (stop_requested && !cancel_sent) {
        // Humble 的 ExecuteTrajectory 取消回调可能被同步执行阻塞。
        // stop 事件直接交给轨迹执行管理器；拿到句柄后再发一次以覆盖发送目标时的竞态。
        arm->stop();
        execution_client->async_cancel_goal(handle);
        cancel_sent = true;
      }
    }
    if (!rclcpp::ok()) {return false;}
    const auto outcome = result.get();
    return !stop_requested && outcome.code == rclcpp_action::ResultCode::SUCCEEDED &&
           outcome.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  bool namedTarget(Group & group, const std::string & name)
  {
    group.clearPoseTargets();
    group.setStartStateToCurrentState();
    if (!group.setNamedTarget(name)) {
      RCLCPP_ERROR(node->get_logger(), "Unknown named target: %s", name.c_str());
      return false;
    }
    return planAndExecute(group);
  }
};

MotionController::MotionController(const rclcpp::Node::SharedPtr & node)
: MotionController(node, Options{}) {}

MotionController::MotionController(const rclcpp::Node::SharedPtr & node, const Options & options)
: impl_(std::make_unique<Impl>(node, options)) {}

MotionController::~MotionController() = default;

bool MotionController::moveToNamedTarget(const std::string & name)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->namedTarget(*impl_->arm, name);
}

bool MotionController::moveToJoints(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto & arm = *impl_->arm;
  if (joints.size() != arm.getVariableCount() ||
    !std::all_of(joints.begin(), joints.end(), [](double v) {return std::isfinite(v);}))
  {
    RCLCPP_ERROR(impl_->node->get_logger(), "Invalid joint count or non-finite joint value");
    return false;
  }
  arm.clearPoseTargets();
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(joints)) {
    RCLCPP_ERROR(impl_->node->get_logger(), "Joint target is outside configured bounds");
    return false;
  }
  return impl_->planAndExecute(arm);
}

bool MotionController::moveToPose(
  const std::array<double, 3> & position, const std::array<double, 3> & rpy, bool cartesian)
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = impl_->options.reference_frame;
  target.pose.position.x = position[0];
  target.pose.position.y = position[1];
  target.pose.position.z = position[2];
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  target.pose.orientation.x = q.x();
  target.pose.orientation.y = q.y();
  target.pose.orientation.z = q.z();
  target.pose.orientation.w = q.w();
  return moveToPose(target, cartesian);
}

bool MotionController::moveToPose(const geometry_msgs::msg::PoseStamped & pose, bool cartesian)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto target = pose;
  auto & q = target.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (target.header.frame_id.empty() || !finitePose(target.pose) ||
    !std::isfinite(norm) || norm < 1e-9)
  {
    RCLCPP_ERROR(impl_->node->get_logger(), "Invalid pose, quaternion or empty frame_id");
    return false;
  }
  q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
  auto & arm = *impl_->arm;
  arm.clearPoseTargets();
  arm.setStartStateToCurrentState();
  bool success = false;
  if (!cartesian) {
    if (arm.setPoseTarget(target)) {
      success = impl_->planAndExecute(arm);
    } else {
      RCLCPP_ERROR(impl_->node->get_logger(), "MoveIt rejected pose target");
    }
  } else {
    if (impl_->stop_requested) {return false;}
    // Cartesian API 的 waypoints 不含 Header，必须明确在统一参考系下输入。
    if (target.header.frame_id != impl_->options.reference_frame) {
      RCLCPP_ERROR(
        impl_->node->get_logger(), "Cartesian target must be in %s",
        impl_->options.reference_frame.c_str());
      return false;
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm.computeCartesianPath(
      {target.pose}, impl_->options.cartesian_step, 0.0, trajectory, true);
    if (fraction >= 1.0 - 1e-6 && !trajectory.joint_trajectory.points.empty()) {
      success = impl_->execute(trajectory);
    } else {
      RCLCPP_ERROR(impl_->node->get_logger(), "Cartesian path incomplete: %.1f%%", fraction * 100);
    }
  }
  arm.clearPoseTargets();
  return success;
}

bool MotionController::moveGripperToNamedTarget(const std::string & name)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->namedTarget(*impl_->gripper, name);
}

bool MotionController::openGripper()
{
  return moveGripperToNamedTarget(impl_->options.gripper_open_target);
}

bool MotionController::moveGripperToJoints(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto & gripper = *impl_->gripper;
  if (joints.size() != gripper.getVariableCount() ||
    !std::all_of(joints.begin(), joints.end(), [](double v) {return std::isfinite(v);}))
  {
    RCLCPP_ERROR(impl_->node->get_logger(), "Invalid gripper joint target");
    return false;
  }
  gripper.setStartStateToCurrentState();
  if (!gripper.setJointValueTarget(joints)) {
    RCLCPP_ERROR(impl_->node->get_logger(), "Gripper target is outside configured bounds");
    return false;
  }
  return impl_->planAndExecute(gripper);
}

bool MotionController::closeGripper()
{
  return moveGripperToNamedTarget(impl_->options.gripper_close_target);
}

void MotionController::stop()
{
  // 不取运动互斥锁：执行线程可能正在等待轨迹结果。
  impl_->stop_requested = true;
  impl_->arm->stop();
}

void MotionController::clearStop()
{
  impl_->stop_requested = false;
}

}  // namespace my_robot_commander
