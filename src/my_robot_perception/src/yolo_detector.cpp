#include <algorithm>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <my_robot_perception/object_catalog.hpp>
#include <yolo_tools/segmenter.hpp>

class YoloDetector : public rclcpp::Node
{
public:
  YoloDetector() : Node("yolo_detector")
  {
    const auto classes = my_robot_perception::loadCatalog(*this);
    const auto configured = declare_parameter<std::string>("model_path", "");
    const auto override = declare_parameter<std::string>("model_path_override", "");
    std::filesystem::path path(override.empty() ? configured : override);
    if (path.empty()) {
      throw std::invalid_argument("Set yolo_detector.model_path in perception.yaml or yolo_model:=/path/model.onnx");
    }
    const auto config = get_parameter("perception_config").as_string();
    if (path.is_relative()) {
      path = (override.empty() && !config.empty() ?
        std::filesystem::absolute(config).parent_path() : std::filesystem::current_path()) / path;
    }
    path = std::filesystem::canonical(path);
    set_parameter(rclcpp::Parameter("model_path", path.string()));
    yolo_tools::Options options;
    options.confidence = declare_parameter<double>("confidence", 0.25);
    options.iou_threshold = declare_parameter<double>("iou_threshold", 0.45);
    options.mask_threshold = declare_parameter<double>("mask_threshold", 0.5);
    options.threads = declare_parameter<int>("threads", 1);
    min_area_ = declare_parameter<int>("min_area", 12);
    if (min_area_ < 1) {throw std::invalid_argument("min_area must be positive pixels");}
    show_window_ = declare_parameter<bool>("show_window", false);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    model_ = std::make_unique<yolo_tools::Segmenter>(path.string(), options);
    for (const auto & name : model_->names()) {
      const auto found = std::find_if(classes.begin(), classes.end(),
        [&](const auto & entry) {return entry.name == name;});
      class_ids_.push_back(found == classes.end() ? 0 : found->id);
      if (found == classes.end()) {
        RCLCPP_WARN(get_logger(), "Model class '%s' is not in the catalog; excluded", name.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Model class '%s' -> class_id %u", name.c_str(), found->id);
      }
    }
    if (std::none_of(class_ids_.begin(), class_ids_.end(), [](int id) {return id != 0;})) {
      throw std::invalid_argument("No model class names match the shared catalog; use matching trained weights");
    }
    const auto labels = declare_parameter<std::string>("labels_topic", "/perception/color_labels");
    labels_pub_ = create_publisher<sensor_msgs::msg::Image>(labels, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("debug_topic", "/perception/color_debug"), 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("image_topic", "/camera_sensor/image_raw"),
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&YoloDetector::detect, this, std::placeholders::_1));
    if (show_window_) {cv::namedWindow("YOLO detector", cv::WINDOW_NORMAL);}
    RCLCPP_INFO(get_logger(), "YOLO CPU model %s; publishing mono8 class labels on %s", path.c_str(), labels.c_str());
  }

  ~YoloDetector() override
  {
    if (show_window_) {cv::destroyWindow("YOLO detector");}
  }

private:
  void detect(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      const auto input = cv_bridge::toCvShare(msg, "bgr8");
      const auto detections = model_->predict(input->image);
      cv::Mat labels = cv::Mat::zeros(input->image.size(), CV_8UC1);
      cv::Mat debug;
      if (publish_debug_ || show_window_) {debug = input->image.clone();}
      // SDK returns confidence order, so higher-confidence masks own overlaps.
      for (const auto & detection : detections) {
        const auto id = class_ids_.at(detection.class_index);
        if (id == 0 || cv::countNonZero(detection.mask) < min_area_) {continue;}
        labels.setTo(id, (detection.mask != 0) & (labels == 0));
        if (!debug.empty()) {
          cv::rectangle(debug, detection.box, {0, 255, 255}, 1);
          cv::putText(debug, model_->names()[detection.class_index] + " id=" + std::to_string(id),
            detection.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 255}, 1);
        }
      }
      // Empty inference is a real empty frame. Errors below publish no healthy frame.
      labels_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", labels).toImageMsg());
      if (publish_debug_) {
        debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
      }
      if (show_window_) {cv::imshow("YOLO detector", debug); cv::waitKey(1);}
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "YOLO frame failed; waiting for the next frame: %s", error.what());
    }
  }

  std::unique_ptr<yolo_tools::Segmenter> model_;
  std::vector<uint16_t> class_ids_;
  int min_area_;
  bool show_window_, publish_debug_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr labels_pub_, debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YoloDetector>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("yolo_detector"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
