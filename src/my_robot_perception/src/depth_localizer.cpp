#include <algorithm>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <functional>
#include <memory>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <my_robot_perception/object_catalog.hpp>

class DepthLocalizer : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

public:
  DepthLocalizer() : Node("depth_localizer")
  {
    const auto labels_topic = declare_parameter<std::string>(
      "labels_topic", "/perception/color_labels");
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera_sensor/depth/image_raw");
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera_sensor/camera_info");
    const auto output_topic = declare_parameter<std::string>(
      "points_topic", "/perception/object_points");
    target_frame_ = declare_parameter<std::string>("target_frame", "");
    objects_frame_ = declare_parameter<std::string>("objects_frame", "base_link");
    classes_ = my_robot_perception::loadCatalog(*this);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    if (objects_frame_.empty() ||
      !std::isfinite(tf_timeout_) || tf_timeout_ < 0)
    {
      throw std::invalid_argument("Invalid object frame or TF timeout");
    }
    match_distance_ = declare_parameter<double>("track_match_distance", 0.025);
    ambiguity_margin_ = declare_parameter<double>("track_ambiguity_margin", 0.003);
    if (!std::isfinite(match_distance_) || match_distance_ <= 0 ||
      !std::isfinite(ambiguity_margin_) || ambiguity_margin_ < 0)
    {
      throw std::invalid_argument("Invalid tracking distances");
    }
    min_depth_ = declare_parameter<double>("min_depth", 0.05);
    max_depth_ = declare_parameter<double>("max_depth", 3.0);
    depth_tolerance_ = declare_parameter<double>("depth_tolerance", 0.02);
    min_pixels_ = declare_parameter<int>("min_valid_pixels", 5);
    erosion_size_ = declare_parameter<int>("erosion_kernel", 1);
    const double sync_slop = declare_parameter<double>("sync_slop", 0.03);
    const int sync_queue_size = declare_parameter<int>("sync_queue_size", 30);
    if (!std::isfinite(min_depth_) || !std::isfinite(max_depth_) ||
      !std::isfinite(depth_tolerance_) || !std::isfinite(sync_slop) ||
      min_depth_ <= 0 || max_depth_ <= min_depth_ || depth_tolerance_ <= 0 ||
      min_pixels_ < 1 || erosion_size_ < 1 || erosion_size_ % 2 == 0 || sync_slop <= 0 || sync_queue_size < 1)
    {
      throw std::invalid_argument("Invalid depth, pixel count, erosion or sync parameters");
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    objects_pub_ = create_publisher<my_robot_interface::msg::DetectedObjectArray>(
      declare_parameter<std::string>("objects_topic", "/perception/objects"), 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { info_ = info; });

    // 按采集时间匹配；检测器保留原 RGB 时间戳，队列容纳推理延迟。
    labels_sub_.subscribe(this, labels_topic, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size), labels_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop));
    sync_->registerCallback(std::bind(
      &DepthLocalizer::localize, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Depth localization ready; publishing surface points and object centers");
  }

private:
  static float median(std::vector<float> values)
  {
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  using Object = my_robot_interface::msg::DetectedObject;

  void associate(std::vector<Object> & objects)
  {
    // ID 只在首次出现时分配。相机离开观察位或漏检不会注销历史轨迹。
    std::sort(objects.begin(), objects.end(), [](const Object & a, const Object & b) {
      return std::tie(a.class_id, a.pose.position.x, a.pose.position.y) <
             std::tie(b.class_id, b.pose.position.x, b.pose.position.y);
    });
    struct Match {double distance; size_t detection, track;};
    std::vector<Match> matches;
    for (size_t i = 0; i < objects.size(); ++i) {
      for (size_t j = 0; j < tracks_.size(); ++j) {
        if (objects[i].class_id != tracks_[j].class_id) {continue;}
        const auto & a = objects[i].pose.position;
        const auto & b = tracks_[j].pose.position;
        const double d = std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
        if (d <= match_distance_) {matches.push_back({d, i, j});}
      }
    }
    std::sort(matches.begin(), matches.end(), [](const Match & a, const Match & b) {
      return std::tie(a.distance, a.detection, a.track) <
             std::tie(b.distance, b.detection, b.track);
    });
    std::vector<bool> assigned(objects.size()), used(tracks_.size());
    for (const auto & match : matches) {
      if (assigned[match.detection] || used[match.track]) {continue;}
      auto & object = objects[match.detection];
      object.object_id = tracks_[match.track].object_id;
      object.tracking_uncertain = std::any_of(matches.begin(), matches.end(),
        [&](const Match & other) {
          return ((other.detection == match.detection && other.track != match.track) ||
                 (other.track == match.track && other.detection != match.detection)) &&
                 std::abs(other.distance - match.distance) <= ambiguity_margin_;
        });
      assigned[match.detection] = used[match.track] = true;
      // 歧义时不把猜测位置写回身份记录，等待分离后的新观测。
      if (!object.tracking_uncertain) {tracks_[match.track] = object;}
    }
    for (size_t i = 0; i < objects.size(); ++i) {
      if (assigned[i]) {continue;}
      auto & object = objects[i];
      object.tracking_uncertain = std::any_of(matches.begin(), matches.end(),
        [i](const Match & match) {return match.detection == i;});
      const auto definition = std::find_if(classes_.begin(), classes_.end(),
        [&](const auto & entry) {return entry.id == object.class_id;});
      object.object_id = definition->name + "_" + std::to_string(++next_id_[object.class_id]);
      if (!object.tracking_uncertain) {tracks_.push_back(object);}
    }
  }

  void localize(const Image::ConstSharedPtr & labels_msg, const Image::ConstSharedPtr & depth_msg)
  {
    // 必须提供与 RGB 对齐的深度和对应 CameraInfo，不能仅凭相同尺寸认定对齐。
    if (!info_ || labels_msg->header.frame_id.empty() ||
      labels_msg->header.frame_id != depth_msg->header.frame_id ||
      info_->header.frame_id != labels_msg->header.frame_id ||
      labels_msg->width != depth_msg->width || labels_msg->height != depth_msg->height ||
      labels_msg->width != info_->width || labels_msg->height != info_->height)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for matching CameraInfo and RGB-aligned depth (same optical frame and dimensions)");
      return;
    }
    if (labels_msg->encoding != "mono8" ||
      (depth_msg->encoding != "32FC1" && depth_msg->encoding != "16UC1"))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Expected mono8 labels and 32FC1 (m) or 16UC1 (mm) depth");
      return;
    }
    const double fx = info_->k[0], fy = info_->k[4];
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
      !std::isfinite(info_->k[2]) || !std::isfinite(info_->k[5]) ||
      !std::all_of(info_->d.begin(), info_->d.end(), [](double d) { return std::isfinite(d); }))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid camera intrinsics");
      return;
    }
    // 本节点使用原始彩色图的 K/D；整流图需提供对应的等效内参。
    const bool distorted = std::any_of(info_->d.begin(), info_->d.end(),
      [](double d) { return d != 0.0; });
    if (distorted && info_->distortion_model != "plumb_bob" &&
      info_->distortion_model != "rational_polynomial")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported distortion model");
      return;
    }

    try {
      const auto labels = cv_bridge::toCvShare(labels_msg, "mono8");
      const auto depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
      std::vector<cv::Point3f> points;
      std::vector<uint8_t> class_ids;
      my_robot_interface::msg::DetectedObjectArray objects;
      objects.header = labels_msg->header;
      objects.header.frame_id = objects_frame_;
      const auto issue = [&](const std::string & reason) {
        if (std::find(objects.issues.begin(), objects.issues.end(), reason) == objects.issues.end()) {
          objects.issues.push_back(reason);
        }
      };
      std::set<int> present;
      for (int row = 0; row < labels->image.rows; ++row) {
        const auto * pixels = labels->image.ptr<uint8_t>(row);
        for (int col = 0; col < labels->image.cols; ++col) {if (pixels[col]) {present.insert(pixels[col]);}}
      }
      for (const auto id : present) {
        if (std::none_of(classes_.begin(), classes_.end(), [id](const auto & c) {return c.id == id;})) {
          issue("undefined class " + std::to_string(id));
        }
      }
      if (classes_.empty()) {issue("no valid class definitions");}
      tf2::Transform camera_to_objects;
      camera_to_objects.setIdentity();
      bool objects_tf_ready = true;
      try {
        if (labels_msg->header.frame_id != objects_frame_) {
          const auto transform = tf_buffer_->lookupTransform(
            objects_frame_, labels_msg->header.frame_id, rclcpp::Time(labels_msg->header.stamp),
            rclcpp::Duration::from_seconds(tf_timeout_));
          tf2::fromMsg(transform.transform, camera_to_objects);
        }
      } catch (const tf2::TransformException & error) {
        objects_tf_ready = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Object center TF unavailable: %s", error.what());
      }
      for (const auto & definition : classes_) {
        if (!present.count(definition.id)) {continue;}
        const auto class_name = "class " + std::to_string(definition.id);
        cv::Mat mask = labels->image == definition.id;
        if (erosion_size_ > 1) {
          cv::erode(mask, mask, cv::getStructuringElement(
            cv::MORPH_ELLIPSE, {erosion_size_, erosion_size_}));
        }
        cv::Mat components, stats, centroids;
        const int count = cv::connectedComponentsWithStats(mask, components, stats, centroids);
        for (int id = 1; id < count; ++id) {
          if (stats.at<int>(id, cv::CC_STAT_AREA) < min_pixels_) {
            continue;
          }
          std::vector<cv::Point> pixels;
          cv::findNonZero(components == id, pixels);
          std::vector<float> depths;
          std::vector<cv::Point> valid_pixels;
          for (const auto & pixel : pixels) {
            const float z = depth_msg->encoding == "32FC1" ?
              depth->image.at<float>(pixel) : depth->image.at<uint16_t>(pixel) * 0.001f;
            if (std::isfinite(z) && z >= min_depth_ && z <= max_depth_) {
              depths.push_back(z);
              valid_pixels.push_back(pixel);
            }
          }
          if (depths.size() < static_cast<size_t>(min_pixels_)) {
            issue(class_name + ": insufficient valid depth");
            continue;
          }
          const float center_depth = median(depths);
          std::vector<float> inliers;
          cv::Point2d uv(0, 0);
          for (size_t i = 0; i < depths.size(); ++i) {
            if (std::abs(depths[i] - center_depth) <= depth_tolerance_) {
              inliers.push_back(depths[i]);
              uv += cv::Point2d(valid_pixels[i].x, valid_pixels[i].y);
            }
          }
          if (inliers.size() < static_cast<size_t>(min_pixels_)) {
            issue(class_name + ": insufficient depth inliers");
            continue;
          }
          uv *= 1.0 / inliers.size();
          std::vector<cv::Point2d> ray;
          const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            fx, 0, info_->k[2], 0, fy, info_->k[5], 0, 0, 1);
          cv::undistortPoints(std::vector<cv::Point2d>{uv}, ray, camera_matrix,
            distorted ? cv::Mat(info_->d) : cv::Mat());
          const float z = median(inliers);
          points.emplace_back(ray[0].x * z, ray[0].y * z, z);
          class_ids.push_back(static_cast<uint8_t>(definition.id));

          // 统一掩码定位：有效像素中心 + 深度中值。标称半高将表面点换为近似中心。
          // 假设物体直立、顶部可见；遮挡和侧面占比高时中心可能偏移。
          if (objects_tf_ready) {
            const auto center = camera_to_objects * tf2::Vector3(
              points.back().x, points.back().y, points.back().z);
            my_robot_interface::msg::DetectedObject object;
            object.class_id = definition.id;
            object.color = definition.color;
            object.dimensions.x = definition.dimensions[0];
            object.dimensions.y = definition.dimensions[1];
            object.dimensions.z = definition.dimensions[2];
            object.pose.position.x = center.x();
            object.pose.position.y = center.y();
            object.pose.position.z = center.z() - object.dimensions.z / 2;
            object.pose.orientation.w = 1.0;
            objects.objects.push_back(object);
          }
        }
      }

      // TF 故障不是空场景；不更新身份，也不发布可被误判为清场的空列表。
      if (objects_tf_ready) {
        associate(objects.objects);
        if (!objects.issues.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Incomplete perception: %s", objects.issues.front().c_str());
        }
        objects_pub_->publish(objects);
      }

      auto header = labels_msg->header;
      if (!points.empty() && !target_frame_.empty() && target_frame_ != header.frame_id) {
        // 使用采集时间查询 TF；暂不可用时丢弃本帧，避免使用错误的最新姿态。
        const auto transform = tf_buffer_->lookupTransform(
          target_frame_, header.frame_id, rclcpp::Time(header.stamp),
          rclcpp::Duration::from_seconds(tf_timeout_));
        for (auto & point : points) {
          geometry_msgs::msg::PointStamped source, result;
          source.header = header;
          source.point.x = point.x;
          source.point.y = point.y;
          source.point.z = point.z;
          tf2::doTransform(source, result, transform);
          point = cv::Point3f(result.point.x, result.point.y, result.point.z);
        }
      }
      if (!target_frame_.empty()) {
        header.frame_id = target_frame_;
      }
      sensor_msgs::msg::PointCloud2 output;
      output.header = header;
      sensor_msgs::PointCloud2Modifier modifier(output);
      modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "label", 1, sensor_msgs::msg::PointField::UINT8);
      modifier.resize(points.size());
      if (!points.empty()) {
        sensor_msgs::PointCloud2Iterator<float> x(output, "x"), y(output, "y"), z(output, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> label(output, "label");
        for (size_t i = 0; i < points.size(); ++i, ++x, ++y, ++z, ++label) {
          *x = points[i].x;
          *y = points[i].y;
          *z = points[i].z;
          *label = class_ids[i];
        }
      }
      // 无有效物块时发布空点云，表示本帧无检测结果。
      points_pub_->publish(output);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF unavailable: %s", error.what());
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Image conversion: %s", error.what());
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCV: %s", error.what());
    }
  }

  std::vector<Object> tracks_;
  std::map<uint16_t, uint64_t> next_id_;
  std::vector<my_robot_perception::ObjectClass> classes_;
  double match_distance_, ambiguity_margin_;
  std::string target_frame_, objects_frame_;
  double min_depth_, max_depth_, depth_tolerance_, tf_timeout_;
  int min_pixels_, erosion_size_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  message_filters::Subscriber<Image> labels_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<my_robot_interface::msg::DetectedObjectArray>::SharedPtr objects_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DepthLocalizer>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_localizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
