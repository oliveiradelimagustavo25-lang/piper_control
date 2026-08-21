#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>


int main()
{

    // 选择ArUco字典
    auto package_share_directory = ament_index_cpp::get_package_share_directory("my_aruco_pkg");
    cv::Ptr<cv::aruco::Dictionary> dictionary =cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    int marker_id = 0;

    int size = 500;

    cv::Mat marker;

    cv::aruco::drawMarker(dictionary, marker_id, size, marker, 1);

    cv::imwrite(package_share_directory + "/aruco_model/materials/textures/marker_0.png", marker);

    cv::imwrite("src/my_aruco_pkg/aruco_model/materials/textures/marker_0.png", marker);



    std::cout << "ArUco marker generated!"<< std::endl;

    return 0;
}