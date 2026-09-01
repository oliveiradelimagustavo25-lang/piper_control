#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>


int main(int argc,char** argv)
{

    // 选择ArUco字典
    auto package_share_directory = ament_index_cpp::get_package_share_directory("my_aruco_pkg");
    cv::Ptr<cv::aruco::Dictionary> dictionary =cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    int marker_id = 0;
    if (argc > 1)
    {
        try
        {
            std::string arg = argv[argc - 1];
            size_t pos = 0;
            int id = std::stoi(arg, &pos);
            // 必须保证整个字符串都是数字
            if (pos == arg.size())
            {
                marker_id = id;
                std::cout << " marker ID: " << arg << std::endl;
            }
            else
            {
                std::cerr << "Invalid marker ID: " << arg << std::endl;
            }
        }
        catch (const std::invalid_argument&)
        {
            std::cerr << "Invalid marker ID: "
                    << argv[argc - 1]
                    << " (not a number)"
                    << std::endl;
        }
        catch (const std::out_of_range&)
        {
            std::cerr << "Marker ID out of range: "
                    << argv[argc - 1]
                    << std::endl;
        }
    }
    constexpr int marker_pixels = 500;
    constexpr int quiet_zone_pixels = 50;
    constexpr int texture_pixels = marker_pixels + 2 * quiet_zone_pixels;

    cv::Mat marker;

    cv::aruco::drawMarker(dictionary, marker_id, marker_pixels, marker, 1);

    // ArUco detection needs a light quiet zone outside the black marker border.
    cv::Mat texture(texture_pixels, texture_pixels, CV_8UC1, cv::Scalar(255));
    marker.copyTo(texture(cv::Rect(
        quiet_zone_pixels, quiet_zone_pixels, marker_pixels, marker_pixels)));

    cv::imwrite(package_share_directory + "/aruco_model/materials/textures/marker_0.png", texture);

    cv::imwrite("src/my_aruco_pkg/aruco_model/materials/textures/marker_0.png", texture);



    std::cout << "ArUco marker generated!"<< std::endl;

    return 0;
}
