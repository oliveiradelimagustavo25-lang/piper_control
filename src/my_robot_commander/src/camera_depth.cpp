#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>


class CameraNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

     public:
        CameraNode(): Node("camera_depth_node")
        {
           
            subscription_ =this->create_subscription<sensor_msgs::msg::Image>("/camera_sensor/depth/image_raw",10,std::bind(
                        &CameraNode::image_callback,
                        this,
                        std::placeholders::_1
                    )
                );
            RCLCPP_INFO(this->get_logger(),"Camera node started" );
            
        }
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            cv::Mat image;
            try
            {
                image =cv_bridge::toCvCopy(msg,"32FC1")->image;

            }
            catch(cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "cv_bridge error"
                );
                return;
            }
            double minVal, maxVal;
            cv::minMaxLoc(image, &minVal, &maxVal);

            cv::Mat depth_display;
            image.convertTo(
                depth_display,
                CV_8UC1,
                255.0 / maxVal
            );
            cv::imshow("Gazebo Camera Depth",depth_display);
            cv::waitKey(1);

        }

};

int main(int agrc,char ** agrv)
{
    rclcpp::init(agrc,agrv);
    auto camera = std::make_shared<CameraNode>();
    rclcpp::spin(camera);
    rclcpp::shutdown();

}