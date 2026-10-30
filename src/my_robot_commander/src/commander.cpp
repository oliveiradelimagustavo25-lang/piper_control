#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float32_multi_array.hpp>
#include <example_interfaces/msg/string.hpp>
#include <my_robot_interface/msg/robotinterface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "my_robot_commander/motion_controller.hpp"

namespace
{
template<typename T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & fallback)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, fallback);
  }
  return node->get_parameter(name).get_value<T>();
}

my_robot_commander::MotionController::Options motionOptions(const rclcpp::Node::SharedPtr & node)
{
  my_robot_commander::MotionController::Options options;
  options.arm_group = parameter(node, "arm_group", options.arm_group);
  options.gripper_group = parameter(node, "gripper_group", options.gripper_group);
  options.reference_frame = parameter(node, "reference_frame", options.reference_frame);
  options.end_effector_link = parameter(node, "end_effector_link", options.end_effector_link);
  options.gripper_open_target = parameter(node, "gripper_open_target", options.gripper_open_target);
  options.gripper_close_target = parameter(
    node, "gripper_close_target",
    options.gripper_close_target);
  options.velocity_scaling = parameter(node, "velocity_scaling", options.velocity_scaling);
  options.acceleration_scaling = parameter(
    node, "acceleration_scaling",
    options.acceleration_scaling);
  options.planning_time = parameter(node, "planning_time", options.planning_time);
  options.cartesian_step = parameter(node, "cartesian_step", options.cartesian_step);
  return options;
}
}  // namespace

// 兼容原有四个 Topic；抓放 Action 节点可直接复用 MotionController。
class Commander
{
public:
  explicit Commander(const rclcpp::Node::SharedPtr & node)
  : node_(node), motion_(node, motionOptions(node))
  {
    name_sub_ = node_->create_subscription<example_interfaces::msg::String>(
      "name_target", 10, [this](example_interfaces::msg::String::ConstSharedPtr msg) {
        motion_.moveToNamedTarget(msg->data);
      });
    gripper_sub_ = node_->create_subscription<example_interfaces::msg::Bool>(
      "open_gripper", 10, [this](example_interfaces::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          motion_.openGripper();
        } else {
          motion_.closeGripper();
        }
      });
    joints_sub_ = node_->create_subscription<example_interfaces::msg::Float32MultiArray>(
      "joint_cmd", 10, [this](example_interfaces::msg::Float32MultiArray::ConstSharedPtr msg) {
        motion_.moveToJoints(std::vector<double>(msg->data.begin(), msg->data.end()));
      });
    pose_sub_ = node_->create_subscription<my_robot_interface::msg::Robotinterface>(
      "pose_command", 10, [this](my_robot_interface::msg::Robotinterface::ConstSharedPtr msg) {
        if (msg->position.size() != 3 || msg->radian.size() != 3) {
          RCLCPP_ERROR(node_->get_logger(), "pose_command requires 3 position and 3 radian values");
          return;
        }
        motion_.moveToPose(
          std::array<double, 3>{msg->position[0], msg->position[1], msg->position[2]},
          std::array<double, 3>{msg->radian[0], msg->radian[1], msg->radian[2]}, msg->cartesian);
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  my_robot_commander::MotionController motion_;
  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr name_sub_;
  rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32MultiArray>::SharedPtr joints_sub_;
  rclcpp::Subscription<my_robot_interface::msg::Robotinterface>::SharedPtr pose_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "commander", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  // 运动调用是阻塞的，executor 持续处理 MoveIt 状态和执行结果。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  int result = 0;
  std::thread spin_thread([&]() {
      try {
        executor.spin();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(node->get_logger(), "Executor stopped: %s", error.what());
      }
    });
  try {
    Commander commander(node);
    RCLCPP_INFO(node->get_logger(), "Commander ready");
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
