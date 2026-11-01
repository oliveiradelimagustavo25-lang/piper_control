#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <my_robot_commander/motion_controller.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_controller_probe",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  std::mutex state_mutex;
  std::map<std::string, double> joints;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex);
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joints[msg->name[i]] = msg->position[i];
      }
    });
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread thread([&]() { executor.spin(); });
  std::ofstream report(node->get_parameter("report_file").as_string());
  int failed = 0;
  report << "[\n";
  bool first = true;
  auto check = [&](const std::string & name, const std::function<bool()> & test) {
    bool passed = false;
    std::string message;
    try {
      passed = test();
    } catch (const std::exception & error) {
      message = error.what();
    }
    if (!passed) { ++failed; }
    if (!first) { report << ",\n"; }
    first = false;
    report << "{\"name\":" << std::quoted(name) << ",\"passed\":" << (passed ? "true" : "false")
           << ",\"message\":" << std::quoted(message) << "}";
    report.flush();
    std::cout << "CHECK " << name << ": " << (passed ? "PASS" : "FAIL") << std::endl;
  };
  auto joint_error = [&](const std::vector<double> & target, int start) {
    std::this_thread::sleep_for(300ms);
    std::lock_guard<std::mutex> lock(state_mutex);
    double error = 0;
    for (size_t i = 0; i < target.size(); ++i) {
      const auto it = joints.find("joint" + std::to_string(start + i));
      if (it == joints.end()) { return 1000.0; }
      error = std::max(error, std::abs(it->second - target[i]));
    }
    std::cout << "  max joint error: " << error << std::endl;
    return error;
  };
  auto current_pose = [&]() {
    auto tf = buffer.lookupTransform("base_link", "link6", tf2::TimePointZero,
      tf2::durationFromSec(2));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return pose;
  };
  auto at_pose = [&](const geometry_msgs::msg::PoseStamped & target) {
    std::this_thread::sleep_for(300ms);
    const auto actual = current_pose();
    const auto & a = actual.pose.position;
    const auto & b = target.pose.position;
    const double distance = std::hypot(std::hypot(a.x-b.x, a.y-b.y), a.z-b.z);
    tf2::Quaternion aq, bq;
    tf2::fromMsg(actual.pose.orientation, aq);
    tf2::fromMsg(target.pose.orientation, bq);
    const double angle = aq.angleShortestPath(bq);
    std::cout << "  pose error: " << distance << " m, " << angle << " rad" << std::endl;
    return distance < 0.005 && angle < 0.03;
  };
  try {
    my_robot_commander::MotionController::Options options;
    options.end_effector_link = "link6";
    options.velocity_scaling = 0.4;
    options.acceleration_scaling = 0.4;
    options.planning_time = 3;
    my_robot_commander::MotionController motion(node, options);
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < deadline) {
      { std::lock_guard<std::mutex> lock(state_mutex); if (joints.size() >= 8) { break; } }
      std::this_thread::sleep_for(100ms);
    }
    check("reject_wrong_joint_count", [&]() { return !motion.moveToJoints({0,0}); });
    check("reject_nonfinite_joints", [&]() {
      return !motion.moveToJoints({std::numeric_limits<double>::quiet_NaN(),0,0,0,0,0}); });
    check("reject_unknown_named_target", [&]() { return !motion.moveToNamedTarget("missing_target"); });
    check("reject_zero_quaternion", [&]() {
      auto pose = current_pose(); pose.pose.orientation = geometry_msgs::msg::Quaternion(); pose.pose.orientation.w = 0;
      return !motion.moveToPose(pose); });
    check("reject_cartesian_wrong_frame", [&]() {
      auto pose = current_pose(); pose.header.frame_id="world";
      return !motion.moveToPose(pose, true); });
    check("reject_gripper_out_of_bounds", [&]() { return !motion.moveGripperToJoints({1,-1}); });
    check("open_gripper", [&]() { return motion.openGripper() && joint_error({0.035,-0.035},7)<0.002; });
    check("close_gripper", [&]() { return motion.closeGripper() && joint_error({0,0},7)<0.002; });
    check("named_gripper_target", [&]() {
      return motion.moveGripperToNamedTarget("open") && joint_error({0.035,-0.035},7)<0.002; });
    check("gripper_joint_target", [&]() {
      return motion.moveGripperToJoints({0.02,-0.02}) && joint_error({0.02,-0.02},7)<0.002; });
    std::vector<double> arm = {0,1.3006,-0.6496,0.0103,0.7436,0};
    check("arm_named_target", [&]() { return motion.moveToNamedTarget("pose_1") && joint_error(arm,1)<0.02; });
    arm[0]=0.10;
    check("arm_joint_target", [&]() { return motion.moveToJoints(arm) && joint_error(arm,1)<0.02; });
    check("arm_pose_stamped", [&]() {
      auto pose = current_pose(); pose.pose.position.y += 0.015;
      return motion.moveToPose(pose) && at_pose(pose); });
    check("arm_xyz_rpy", [&]() {
      auto pose = current_pose(); pose.pose.position.y -= 0.01;
      tf2::Quaternion q; tf2::fromMsg(pose.pose.orientation,q);
      double roll,pitch,yaw; tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
      return motion.moveToPose({pose.pose.position.x,pose.pose.position.y,pose.pose.position.z},
        {roll,pitch,yaw}) && at_pose(pose); });
    check("arm_cartesian", [&]() {
      auto pose = current_pose(); pose.pose.position.z += 0.015;
      return motion.moveToPose(pose,true) && at_pose(pose); });
  } catch (const std::exception & error) {
    check("probe_exception", [&]() -> bool { throw std::runtime_error(error.what()); });
  }
  report << "\n]\n";
  report.close();
  executor.cancel();
  thread.join();
  rclcpp::shutdown();
  return failed == 0 ? 0 : 1;
}
