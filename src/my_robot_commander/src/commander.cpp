#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"     //四元函数转化
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/float32_multi_array.hpp"
#include "example_interfaces/msg/string.hpp"
#include "my_robot_interface/msg/robotinterface.hpp"

using RobotPoseinterface = my_robot_interface::msg::Robotinterface;

class Commander
{
    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
        //std::thread spin_thread;
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr name_target_sub;
        rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr open_gripper_sub;
        rclcpp::Subscription<example_interfaces::msg::Float32MultiArray>::SharedPtr joint_cmd_sub;
        rclcpp::Subscription<RobotPoseinterface>::SharedPtr pose_cmd_sub;

    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            this->node_ = node ;

            this->arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface> (node_,"arm");
            this->arm_->setMaxAccelerationScalingFactor(1.0);
            this->arm_->setMaxVelocityScalingFactor(1.0);

            this->gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface> (node_,"gripper");
            this->gripper_->setMaxAccelerationScalingFactor(1.0);
            this->gripper_->setMaxVelocityScalingFactor(1.0);
            
            name_target_sub = node->create_subscription<example_interfaces::msg::String>("name_target",10,std::bind(&Commander::NameTargetCallback,this,std::placeholders::_1));
            open_gripper_sub = node_->create_subscription<example_interfaces::msg::Bool>("open_gripper",10,std::bind(&Commander::OpenGripperCallback,this,std::placeholders::_1));
            joint_cmd_sub = node_->create_subscription<example_interfaces::msg::Float32MultiArray>("joint_cmd",10,std::bind(&Commander::JointCmdCallback,this,std::placeholders::_1));
            pose_cmd_sub = node->create_subscription<RobotPoseinterface>("pose_command",10,std::bind(&Commander::PoseCmdCallback,this,std::placeholders::_1));
            
            /*
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(this->node_);
            this->spin_thread = std::thread([&executor]()
                {
                    executor.spin();
                }
            );
            */
        }
        void GoToNameTarget(const std::string& name)
        {
            this->arm_->setStartStateToCurrentState();
            this->arm_->setNamedTarget(name);
            PlanAndExecute(arm_);
        }
        void GoToJointTarget(const std::vector<double>& joints)
        {
            this->arm_->setStartStateToCurrentState();
            this->arm_->setJointValueTarget(joints);
            PlanAndExecute(arm_);       
        }
        void GoToPoseTarget(const double* XYZ_position,const double* RXY_radian,bool cartesian=false)
        {
            geometry_msgs::msg::PoseStamped target_pose;
            tf2::Quaternion q;

            q.setRPY(*RXY_radian,*(RXY_radian+1),*(RXY_radian+2));
            q = q.normalize();
            target_pose.header.frame_id = "base_link";
            target_pose.pose.position.x = XYZ_position[0];
            target_pose.pose.position.y = XYZ_position[1];
            target_pose.pose.position.z = XYZ_position[2];
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();
            if(!cartesian)
            {
                this->arm_->setStartStateToCurrentState();
                this->arm_->setPoseTarget(target_pose);
                PlanAndExecute(this->arm_);
            }
            else
            {
                std::vector<geometry_msgs::msg::Pose>  waypose;
                waypose.push_back(target_pose.pose);
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = this->arm_->computeCartesianPath(waypose,0.01,0.0,trajectory);
                if(fraction == 1 )
                {
                    arm_->execute(trajectory);
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Cartesian path incomplete: %.1f%%", fraction * 100.0);
                }
            }
        }

        void PlanAndExecute(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group)
        {
            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

            if(group->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                group->execute(arm_plan);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "MoveIt failed to plan a path to the requested target");
            }
        }

        void Gripper_open()
        {
            this->gripper_->setStartStateToCurrentState();
            this->gripper_->setNamedTarget("open");
            PlanAndExecute(gripper_);
        }
        void Gripper_close()
        {
            this->gripper_->setStartStateToCurrentState();
            this->gripper_->setNamedTarget("close");
            PlanAndExecute(gripper_);
        }
        void NameTargetCallback(const example_interfaces::msg::String& name )
        {
            GoToNameTarget(name.data);
        }

        void OpenGripperCallback(const example_interfaces::msg::Bool& open )
        {
            if(open.data)
            {
                this->Gripper_open();
            }
            else{
                this->Gripper_close();
            }
        }

        void JointCmdCallback(const example_interfaces::msg::Float32MultiArray& msg)
        {
            //将msg.data中的std::vector<float>迁移成std::vector<double> joints
            std::vector<double> joints(msg.data.begin(), msg.data.end());
            if( joints.size() == 6)
            {
                GoToJointTarget(joints);
            }
        }
        void PoseCmdCallback(const RobotPoseinterface& msg)
        {
            if (msg.position.size() != 3 || msg.radian.size() != 3)
            {
                RCLCPP_ERROR(node_->get_logger(), "pose_command requires exactly 3 position and 3 radian values");
                return;
            }
            GoToPoseTarget(msg.position.data(), msg.radian.data(),msg.cartesian);
        }
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commmand = Commander(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
