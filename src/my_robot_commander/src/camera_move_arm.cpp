#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_srvs/srv/trigger.hpp"
#include "my_robot_interface/msg/robotinterface.hpp"



using RobotPoseinterface = my_robot_interface::msg::Robotinterface;



class VisionServoBridge : public rclcpp::Node
{
    private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;



rclcpp::Publisher<RobotPoseinterface>::SharedPtr

pose_pub_;



rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr

calculate_srv_;




public:


    VisionServoBridge()
    :Node("vision_servo_bridge")
    {


        // TF buffer

        tf_buffer_ =
        std::make_shared<tf2_ros::Buffer>(
            this->get_clock()
        );


        tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_
        );



        // 发布给commander

        pose_pub_ =
        this->create_publisher<RobotPoseinterface>(
            "pose_command",
            10
        );



        // service

        calculate_srv_ =
        this->create_service<std_srvs::srv::Trigger>(

            "calculate_visual_pose",

            std::bind(
                &VisionServoBridge::CalculatePose,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )

        );



        RCLCPP_INFO(
            this->get_logger(),
            "vision servo bridge started"
        );


    }



private:



void CalculatePose(

const std::shared_ptr<std_srvs::srv::Trigger::Request> request,

std::shared_ptr<std_srvs::srv::Trigger::Response> response

)

{

    (void)request;


    try
    {


        /*
            获取:

            base_link
                |
                |
          aruco_marker_0

        */


        auto marker_tf =

        tf_buffer_->lookupTransform(

            "base_link",

            "aruco_marker_0",

            tf2::TimePointZero

        );



        /*
            quaternion

            转 RPY
        */


        tf2::Quaternion q(

            marker_tf.transform.rotation.x,

            marker_tf.transform.rotation.y,

            marker_tf.transform.rotation.z,

            marker_tf.transform.rotation.w

        );



        double roll;
        double pitch;
        double yaw;



        tf2::Matrix3x3(q).getRPY(

            roll,

            pitch,

            yaw

        );




        /*
            发布RobotPoseinterface

        */


        RobotPoseinterface msg;



        // xyz

        msg.position.resize(3);


        msg.position[0] =

        marker_tf.transform.translation.x;



        msg.position[1] =

        marker_tf.transform.translation.y;



        msg.position[2] =

        marker_tf.transform.translation.z;




        // RPY

        msg.radian.resize(3);



        msg.radian[0]=roll;

        msg.radian[1]=pitch;

        msg.radian[2]=yaw;




        /*
            不使用笛卡尔

        */


        msg.cartesian=false;



        pose_pub_->publish(msg);



        response->success=true;


        response->message=

        "visual pose calculated";



        RCLCPP_INFO(

            this->get_logger(),

            "publish visual pose"

        );


    }



    catch(tf2::TransformException &e)

    {


        response->success=false;


        response->message=e.what();



        RCLCPP_ERROR(

            this->get_logger(),

            "%s",

            e.what()

        );


    }



}




};



int main(int argc,char **argv)

{


    rclcpp::init(argc,argv);



    auto node=

    std::make_shared<VisionServoBridge>();



    rclcpp::spin(node);



    rclcpp::shutdown();



    return 0;

}