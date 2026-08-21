#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <opencv2/aruco.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2/LinearMath/Quaternion.h>





class CameraNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        double marker_size_;



    public:
        CameraNode(): Node("camera_node")
        {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            subscription_ =
                this->create_subscription<sensor_msgs::msg::Image>("/camera_sensor/image_raw",10,std::bind(
                        &CameraNode::image_callback,
                        this,
                        std::placeholders::_1
                    )
                );
            RCLCPP_INFO(this->get_logger(),"Camera node started" );
            dictionary_ =cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
            camera_matrix_ =(cv::Mat_<double>(3,3)<<400.2,0,400,
                                                    0,400.2,300,
                                                    0,0,1);
            dist_coeffs_ =cv::Mat::zeros(1,5,CV_64F);
            marker_size_=0.2;
            cv::namedWindow("Gazebo Camera",cv::WINDOW_NORMAL);
        }

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            cv::Mat image;
            try
            {
                image =cv_bridge::toCvCopy(msg,"rgb8")->image;

            }
            catch(cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "cv_bridge error"
                );
                return;
            }

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            cv::aruco::detectMarkers(image,dictionary_,corners,ids);



            if(!ids.empty())
            {
                // 1. 绘制marker边框
                cv::aruco::drawDetectedMarkers(
                    image,
                    corners,
                    ids
                );


                // 2. 计算姿态
                std::vector<cv::Vec3d> rvecs;
                std::vector<cv::Vec3d> tvecs;

                cv::aruco::estimatePoseSingleMarkers(
                    corners,
                    marker_size_,
                    camera_matrix_,
                    dist_coeffs_,
                    rvecs,
                    tvecs
                );

                for(size_t i=0;i<ids.size();i++)
                {
                    // 绘制坐标轴
                    cv::aruco::drawAxis(
                        image,
                        camera_matrix_,
                        dist_coeffs_,
                        rvecs[i],
                        tvecs[i],
                        0.1
                    );
                    // 显示文字
                    std::string text ="ID:"+std::to_string(ids[i]);

                    text +=" X:"+std::to_string(tvecs[i][0]);

                    text +=" Y:"+std::to_string(tvecs[i][1]);

                    text +=" Z:"+std::to_string(tvecs[i][2]);

                    cv::putText(
                        image,text,
                        cv::Point(20,50),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.8,
                        cv::Scalar(0,255,0),
                        2
                    );

                    publish_tf(rvecs[i],tvecs[i],ids[i]);
                }
            }
            cv::imshow("Gazebo Camera",image);
            cv::waitKey(1);
        }

        void publish_tf(cv::Vec3d rvec,cv::Vec3d tvec,int id)
        {

            cv::Mat R;
            cv::Rodrigues(rvec,R);

            geometry_msgs::msg::TransformStamped tf;

            tf.header.stamp =this->get_clock()->now();

            tf.header.frame_id ="camera_link";

            tf.child_frame_id ="aruco_marker_" + std::to_string(id);

            tf.transform.translation.x =tvec[0];

            tf.transform.translation.y =tvec[1];

            tf.transform.translation.z =tvec[2];

            
            // rotation matrix -> quaternion
            tf2::Matrix3x3 m(
                R.at<double>(0,0),
                R.at<double>(0,1),
                R.at<double>(0,2),
                R.at<double>(1,0),
                R.at<double>(1,1),
                R.at<double>(1,2),
                R.at<double>(2,0),
                R.at<double>(2,1),
                R.at<double>(2,2)
            );

            tf2::Quaternion q;

            m.getRotation(q);

            tf.transform.rotation.x=q.x();
            tf.transform.rotation.y=q.y();
            tf.transform.rotation.z=q.z();
            tf.transform.rotation.w=q.w();

            tf_broadcaster_->sendTransform(tf);
        }
};



int main(int argc,char **argv)
{

    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<CameraNode>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}