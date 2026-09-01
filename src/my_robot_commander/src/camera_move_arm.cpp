#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "std_srvs/srv/trigger.hpp"
#include "my_robot_interface/msg/robotinterface.hpp"



using RobotPoseinterface = my_robot_interface::msg::Robotinterface;



class VisionServoBridge : public rclcpp::Node
{
    private:

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<RobotPoseinterface>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calculate_srv_;
    std::string base_frame_;
    std::string marker_frame_;
    double tool_offset_;

    public:

    VisionServoBridge()
    :Node("vision_servo_bridge")
    {
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        marker_frame_ = this->declare_parameter<std::string>("marker_frame", "aruco_marker_0");
        // Distance from link6/gripper_base to the fingertip contact point.
        tool_offset_ = this->declare_parameter<double>("tool_offset", 0.18);

        tf_buffer_ =std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_pub_ =this->create_publisher<RobotPoseinterface>("pose_command",10);

        calculate_srv_ =this->create_service<std_srvs::srv::Trigger>("calculate_visual_pose",
            std::bind(
                &VisionServoBridge::CalculatePose,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )

        );
        RCLCPP_INFO(
            this->get_logger(),
            "Vision servo bridge started: %s -> %s, tool_offset=%.3f m",
            base_frame_.c_str(), marker_frame_.c_str(), tool_offset_);
    }



    private:

    void CalculatePose(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        try
        {
            auto marker_tf =tf_buffer_->lookupTransform(
                base_frame_,
                marker_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)
            );

            tf2::Quaternion marker_q(
                marker_tf.transform.rotation.x,
                marker_tf.transform.rotation.y,
                marker_tf.transform.rotation.z,
                marker_tf.transform.rotation.w
            );
            marker_q.normalize();

            tf2::Transform base_to_marker(
                marker_q,
                tf2::Vector3(
                    marker_tf.transform.translation.x,
                    marker_tf.transform.translation.y,
                    marker_tf.transform.translation.z));

            // OpenCV's marker +Z points out of the visible marker face. Place
            // link6 in front of it and turn the gripper +Z axis toward the face.
            tf2::Quaternion tool_q;
            tool_q.setRPY(3.14159265358979323846, 0.0, 0.0);
            tf2::Transform marker_to_tool(tool_q, tf2::Vector3(0.0, 0.0, tool_offset_));
            const tf2::Transform base_to_tool = base_to_marker * marker_to_tool;
            const tf2::Quaternion target_q = base_to_tool.getRotation();

            double roll;
            double pitch;
            double yaw;

            tf2::Matrix3x3(target_q).getRPY(
                roll,
                pitch,
                yaw
            );

            RobotPoseinterface msg;

            msg.position.resize(3);

            msg.position[0] = base_to_tool.getOrigin().x();

            msg.position[1] = base_to_tool.getOrigin().y();

            msg.position[2] = base_to_tool.getOrigin().z();

            msg.radian.resize(3);

            msg.radian[0]=roll;

            msg.radian[1]=pitch;

            msg.radian[2]=yaw;


            msg.cartesian=false;

            pose_pub_->publish(msg);

            response->success=true;

            response->message="marker detected; gripper target published";

            RCLCPP_INFO(

                this->get_logger(),

                "Published gripper target: xyz=[%.3f, %.3f, %.3f]",
                msg.position[0], msg.position[1], msg.position[2]
            );

        }



        catch(const tf2::TransformException &e)

        {
            response->success=false;

            response->message = std::string("Marker TF unavailable. Check that the marker is visible: ") + e.what();

            RCLCPP_ERROR(this->get_logger(),"%s",e.what()
            );
        }

    }

};



    int main(int argc,char **argv)

    {

        rclcpp::init(argc,argv);

        auto node=std::make_shared<VisionServoBridge>();

        rclcpp::spin(node);

        rclcpp::shutdown();

        return 0;

    }
