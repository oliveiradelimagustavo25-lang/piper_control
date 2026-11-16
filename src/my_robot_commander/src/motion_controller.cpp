#include "my_robot_commander/motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.h>
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

  Impl(const rclcpp::Node::SharedPtr & input_node, const Options & input_options)
  : node(input_node), options(input_options)
  {
    if (!node) {
      throw std::invalid_argument("MotionController requires a ROS node");
    }
    if (!(options.velocity_scaling > 0 && options.velocity_scaling <= 1 &&
      options.acceleration_scaling > 0 && options.acceleration_scaling <= 1 &&
      std::isfinite(options.planning_time) && options.planning_time > 0 &&
      std::isfinite(options.cartesian_step) && options.cartesian_step > 0 &&
      std::isfinite(options.ik_timeout) && options.ik_timeout > 0))
    {
      throw std::invalid_argument("Invalid motion scaling, planning time, Cartesian step or IK timeout");
    }
    arm = std::make_unique<Group>(node, options.arm_group);
    gripper = std::make_unique<Group>(node, options.gripper_group);
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

  bool solvePose(moveit::core::RobotState & state,
    const geometry_msgs::msg::PoseStamped & target)
  {
    const auto & q = target.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!finitePose(target.pose) || norm < 1e-9 || target.header.frame_id.empty()) {return false;}
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    const auto & p = target.pose.position;
    pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    if (target.header.frame_id != state.getRobotModel()->getModelFrame()) {
      if (!state.getRobotModel()->hasLinkModel(target.header.frame_id)) {return false;}
      pose = state.getGlobalLinkTransform(target.header.frame_id) * pose;
    }
    const auto * group = state.getJointModelGroup(options.arm_group);
    return group && state.setFromIK(group, pose, arm->getEndEffectorLink(), options.ik_timeout);
  }

  bool planAndExecute(Group & group)
  {
    Group::Plan plan;
    const auto planned = group.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Planning failed, MoveIt code: %d", planned.val);
      return false;
    }
    const auto executed = group.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Execution failed, MoveIt code: %d", executed.val);
      return false;
    }
    return true;
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

bool MotionController::canReachPose(const geometry_msgs::msg::PoseStamped & target)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto state = impl_->arm->getCurrentState();
  return state && impl_->solvePose(*state, target);
}

MotionController::ReleaseCheck MotionController::canReleaseAt(const geometry_msgs::msg::PoseStamped & target,
  const std::vector<double> & closed, const std::vector<double> & released,
  const moveit_msgs::msg::PlanningScene & scene_message)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  planning_scene::PlanningScene scene(impl_->arm->getRobotModel());
  scene.setPlanningSceneMsg(scene_message);
  auto & state = scene.getCurrentStateNonConst();
  const auto * gripper = state.getJointModelGroup(impl_->options.gripper_group);
  if (!gripper || closed.size() != gripper->getVariableCount() ||
    released.size() != closed.size() || !impl_->solvePose(state, target)) {return ReleaseCheck::IK_FAILED;}
  double distance = 0;
  for (size_t i = 0; i < closed.size(); ++i) {
    if (!std::isfinite(closed[i]) || !std::isfinite(released[i])) {return ReleaseCheck::IK_FAILED;}
    distance = std::max(distance, std::abs(released[i] - closed[i]));
  }
  // 每根手指最多移动 1 mm 后检查一次；同时检查手臂、夹爪和周围物体。
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / 0.001)));
  for (int step = 0; step <= steps; ++step) {
    std::vector<double> joints(closed.size());
    for (size_t i = 0; i < joints.size(); ++i) {
      joints[i] = closed[i] + (released[i] - closed[i]) * step / steps;
    }
    state.setJointGroupPositions(gripper, joints);
    state.update();
    if (!state.satisfiesBounds(gripper) || scene.isStateColliding(state)) {
      RCLCPP_WARN(impl_->node->get_logger(), "Placement gripper release is blocked");
      return ReleaseCheck::BLOCKED;
    }
  }
  return ReleaseCheck::OK;
}

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
    !std::all_of(joints.begin(), joints.end(), [](double v) { return std::isfinite(v); }))
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
    // Cartesian API 的 waypoints 不含 Header，必须明确在统一参考系下输入。
    if (target.header.frame_id != impl_->options.reference_frame) {
      RCLCPP_ERROR(impl_->node->get_logger(), "Cartesian target must be in %s",
        impl_->options.reference_frame.c_str());
      return false;
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm.computeCartesianPath(
      {target.pose}, impl_->options.cartesian_step, 0.0, trajectory, true);
    if (fraction >= 1.0 - 1e-6 && !trajectory.joint_trajectory.points.empty()) {
      const auto executed = arm.execute(trajectory);
      success = executed == moveit::core::MoveItErrorCode::SUCCESS;
      if (!success) {
        RCLCPP_ERROR(impl_->node->get_logger(), "Cartesian execution failed: %d", executed.val);
      }
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
    !std::all_of(joints.begin(), joints.end(), [](double v) { return std::isfinite(v); }))
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

}  // namespace my_robot_commander
