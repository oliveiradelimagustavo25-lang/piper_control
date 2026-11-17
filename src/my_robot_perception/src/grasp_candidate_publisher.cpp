// GraspNet 候选发布节点：消费现有观测链（objects/depth/labels），节流推理，
// 发布 /grasp/candidates。数据驱动降级：本节点死亡只影响候选链；
// /perception/objects 由 depth_localizer 独立发布，planner 行为自动退化为传统链路。
// 推理通过子进程调用 tools/graspnet/scripts/predict.py（conda 环境，进程边界）。

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <my_robot_interface/msg/grasp_candidate.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fs = std::filesystem;
using DetectedObjectArray = my_robot_interface::msg::DetectedObjectArray;
using GraspCandidate = my_robot_interface::msg::GraspCandidate;

class GraspCandidatePublisher : public rclcpp::Node
{
public:
  GraspCandidatePublisher() : Node("grasp_candidate_publisher")
  {
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera_sensor/depth/image_raw");
    const auto labels_topic = declare_parameter<std::string>(
      "labels_topic", "/perception/color_labels");
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera_sensor/camera_info");
    const auto objects_topic = declare_parameter<std::string>(
      "objects_topic", "/perception/objects");
    const auto candidates_topic = declare_parameter<std::string>(
      "candidates_topic", "/grasp/candidates");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    min_interval_ = declare_parameter<double>("min_interval", 5.0);
    conda_python_ = declare_parameter<std::string>("conda_python", "");
    predict_script_ = declare_parameter<std::string>("predict_script", "");
    weights_ = declare_parameter<std::string>("weights", "");
    work_root_ = declare_parameter<std::string>("work_root", "/tmp/graspnet_candidates");
    min_pixels_ = declare_parameter<int>("min_pixels", 50);
    keep_work_dirs_ = declare_parameter<int>("keep_work_dirs", 2);
    if (min_interval_ < 1.0) {
      throw std::invalid_argument("min_interval must be >= 1 s");
    }
    if (min_pixels_ < 1) {
      throw std::invalid_argument("min_pixels must be positive");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    candidates_pub_ = create_publisher<GraspCandidate>(candidates_topic, 10);

    if (conda_python_.empty() || predict_script_.empty()) {
      RCLCPP_WARN(get_logger(),
        "conda_python/predict_script not set; candidate publishing disabled "
        "(observation-only mode, data-driven degrade applies)");
      enabled_ = false;
    } else if (!fs::exists(predict_script_)) {
      throw std::invalid_argument("predict_script does not exist: " + predict_script_);
    } else {
      RCLCPP_INFO(get_logger(), "GraspNet candidates enabled via %s", predict_script_.c_str());
    }

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(depth_topic, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
          depth_ = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } catch (const cv_bridge::Exception & error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "depth convert failed: %s", error.what());
          return;
        }
        depth_stamp_ = msg->header.stamp;
        depth_frame_ = msg->header.frame_id;
        have_depth_ = true;
      });
    labels_sub_ = create_subscription<sensor_msgs::msg::Image>(labels_topic, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
          labels_ = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8)->image;
        } catch (const cv_bridge::Exception & error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "labels convert failed: %s", error.what());
          return;
        }
        have_labels_ = true;
      });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        info_ = *msg;
        have_info_ = true;
      });
    objects_sub_ = create_subscription<DetectedObjectArray>(objects_topic, 10,
      [this](const DetectedObjectArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        objects_ = *msg;
        have_objects_ = true;
        pending_ = true;   // 新观测到达 → 请求一轮推理（节流由 worker 把关）
      });

    worker_ = std::thread([this]() { loop(); });
  }

  ~GraspCandidatePublisher() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void loop()
  {
    while (running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (!enabled_ || !running_) {
        continue;
      }
      cv::Mat depth, labels;
      sensor_msgs::msg::CameraInfo info;
      DetectedObjectArray objects;
      rclcpp::Time stamp;
      std::string frame;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pending_ || !have_depth_ || !have_labels_ || !have_info_ || !have_objects_) {
          continue;
        }
        const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_attempt_).count();
        if (last_attempt_.time_since_epoch().count() != 0 && elapsed < min_interval_) {
          continue;
        }
        depth = depth_.clone();
        labels = labels_.clone();
        info = info_;
        objects = objects_;
        stamp = depth_stamp_;
        frame = depth_frame_;
        pending_ = false;
        last_attempt_ = std::chrono::steady_clock::now();
      }
      process(depth, labels, info, objects, stamp, frame);
    }
  }

  void process(const cv::Mat & depth, const cv::Mat & labels,
               const sensor_msgs::msg::CameraInfo & info,
               const DetectedObjectArray & objects,
               const rclcpp::Time & stamp, const std::string & frame)
  {
    // base ← 相机光学系，取观测时刻 TF，不用最新位姿冒充。
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(target_frame_, frame, stamp,
        rclcpp::Duration::from_seconds(0.3));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "candidate TF unavailable: %s", error.what());
      return;
    }
    double m[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    {
      tf2::Transform tf;
      tf2::fromMsg(transform.transform, tf);
      for (int row = 0; row < 3; ++row) {
        m[row][0] = tf.getBasis()[row][0];
        m[row][1] = tf.getBasis()[row][1];
        m[row][2] = tf.getBasis()[row][2];
        m[row][3] = tf.getOrigin()[row];
      }
    }
    // 类别 → 目标列表；掩码像素不足的类视为该帧目标不可见。
    std::map<int, std::vector<std::pair<std::string, geometry_msgs::msg::Point>>> classes;
    for (const auto & object : objects.objects) {
      classes[object.class_id].push_back({object.object_id, object.pose.position});
    }
    std::vector<std::pair<int, const std::vector<std::pair<std::string,
      geometry_msgs::msg::Point>> *>> visible;
    for (const auto & entry : classes) {
      if (cv::countNonZero(labels == entry.first) >= min_pixels_) {
        visible.push_back({entry.first, &entry.second});
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "class %d has < %d mask pixels; skipped this tick",
          entry.first, min_pixels_);
      }
    }
    if (visible.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "no visible target class; candidates skipped this tick");
      return;
    }

    const auto seq = ++sequence_;
    const fs::path dir = fs::path(work_root_) / std::to_string(seq);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "cannot create work dir %s: %s",
        dir.string().c_str(), ec.message().c_str());
      return;
    }
    const auto depth_path = (dir / "depth.f32").string();
    const auto labels_path = (dir / "labels.png").string();
    const auto manifest_path = (dir / "manifest.json").string();
    const auto results_path = (dir / "results.json").string();
    const auto proposals_path = (dir / "proposals.txt").string();
    const auto log_path = (dir / "predict.log").string();

    // 深度原始 float32（行优先）；labels 走 PNG；点云反投影由 predict.py 完成。
    std::ofstream raw(depth_path, std::ios::binary);
    raw.write(reinterpret_cast<const char *>(depth.data),
      static_cast<std::streamsize>(depth.total() * depth.elemSize()));
    raw.close();
    if (!cv::imwrite(labels_path, labels)) {
      RCLCPP_ERROR(get_logger(), "failed to write %s", labels_path.c_str());
      return;
    }

    const double stamp_seconds = stamp.seconds();
    std::ostringstream manifest;
    manifest << std::setprecision(9);
    manifest << "{\"params\": {\"sample_mode\": \"masked_voxel\", \"n_points\": 20000, "
             << "\"seed\": " << seq << ", \"voxel_m\": 0.001, "
             << "\"collision_cloud\": \"scene\", \"collision_points_max\": 80000, "
             << "\"collision_voxel\": 0.01, \"approach_dist\": 0.05, "
             << "\"collision_thresh\": 0.01, \"width_max\": 0.07, "
             << "\"attrib_tol_m\": 0.02}, \"items\": [";
    bool first_item = true;
    for (const auto & entry : visible) {
      if (!first_item) {
        manifest << ", ";
      }
      first_item = false;
      manifest << "{\"id\": \"" << seq << "/class" << entry.first << "\", "
               << "\"depth_raw\": \"" << depth_path << "\", "
               << "\"shape\": [" << depth.rows << ", " << depth.cols << "], "
               << "\"labels\": \"" << labels_path << "\", "
               << "\"class\": " << entry.first << ", "
               << "\"K\": {\"fx\": " << info.k[0] << ", \"fy\": " << info.k[4]
               << ", \"cx\": " << info.k[2] << ", \"cy\": " << info.k[5] << "}, "
               << "\"M\": [";
      for (int row = 0; row < 4; ++row) {
        manifest << (row ? ", " : "") << "[";
        for (int col = 0; col < 4; ++col) {
          manifest << (col ? ", " : "") << m[row][col];
        }
        manifest << "]";
      }
      manifest << "], \"targets\": [";
      bool first_target = true;
      for (const auto & target : *entry.second) {
        if (!first_target) {
          manifest << ", ";
        }
        first_target = false;
        manifest << "{\"body\": \"" << target.first << "\", \"center\": ["
                 << target.second.x << ", " << target.second.y << ", "
                 << target.second.z << "]}";
      }
      manifest << "], \"stamp\": " << stamp_seconds << "}";
    }
    manifest << "]}";
    std::ofstream manifest_file(manifest_path);
    manifest_file << manifest.str();
    manifest_file.close();

    std::ostringstream command;
    command << '"' << conda_python_ << "\" \"" << predict_script_ << "\""
            << " --input \"" << manifest_path << "\" --out \"" << results_path << "\""
            << " --proposals \"" << proposals_path << "\"";
    if (!weights_.empty()) {
      command << " --weights \"" << weights_ << "\"";
    }
    command << " > \"" << log_path << "\" 2>&1";
    RCLCPP_INFO(get_logger(), "inference request %llu: %d class item(s)",
      static_cast<unsigned long long>(seq), static_cast<int>(visible.size()));
    const int code = std::system(command.str().c_str());
    if (code != 0) {
      RCLCPP_WARN(get_logger(), "predict.py failed (exit %d); see %s",
        code, log_path.c_str());
      return;
    }

    // 解析行式候选：BEST <item> <object_id> <rank> <x y z> <qx qy qz qw> <width> <score>
    std::ifstream proposals(proposals_path);
    std::string line;
    int published = 0;
    while (std::getline(proposals, line)) {
      std::istringstream stream(line);
      std::string tag;
      stream >> tag;
      if (tag != "BEST") {
        continue;
      }
      int index = -1, rank = -1;
      std::string body;
      double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 0, width = 0, score = 0;
      stream >> index >> body >> rank >> x >> y >> z >> qx >> qy >> qz >> qw >> width >> score;
      if (rank < 0) {
        continue;   // 该目标本轮无归属候选（目标归属判定，不等于可执行）
      }
      GraspCandidate candidate;
      candidate.header.stamp = stamp;
      candidate.object_id = body;
      candidate.tcp_pose.header.stamp = stamp;
      candidate.tcp_pose.header.frame_id = target_frame_;
      candidate.tcp_pose.pose.position.x = x;
      candidate.tcp_pose.pose.position.y = y;
      candidate.tcp_pose.pose.position.z = z;
      candidate.tcp_pose.pose.orientation.x = qx;
      candidate.tcp_pose.pose.orientation.y = qy;
      candidate.tcp_pose.pose.orientation.z = qz;
      candidate.tcp_pose.pose.orientation.w = qw;
      candidate.width = width;
      candidate.score = score;
      candidates_pub_->publish(candidate);
      ++published;
      RCLCPP_INFO(get_logger(), "candidate %s: tcp (%.4f, %.4f, %.4f) width %.1f mm score %.2f",
        body.c_str(), x, y, z, width * 1000, score);
    }
    RCLCPP_INFO(get_logger(), "request %llu published %d candidate(s)",
      static_cast<unsigned long long>(seq), published);

    prune_work_dirs();
  }

  void prune_work_dirs()
  {
    try {
      std::vector<fs::path> dirs;
      for (const auto & entry : fs::directory_iterator(fs::path(work_root_))) {
        if (entry.is_directory()) {
          dirs.push_back(entry.path());
        }
      }
      std::sort(dirs.begin(), dirs.end(), [](const fs::path & a, const fs::path & b) {
        // 目录名是递增序号，按数值排序；字典序会把 "15" 排在 "8" 前面，误删最新目录。
        try {
          return std::stoll(a.stem().string()) < std::stoll(b.stem().string());
        } catch (const std::invalid_argument &) {
          return a < b;
        }
      });
      std::error_code ec;
      while (dirs.size() > static_cast<size_t>(keep_work_dirs_)) {
        fs::remove_all(dirs.front(), ec);
        dirs.erase(dirs.begin());
      }
    } catch (const fs::filesystem_error &) {
      // 清理失败不影响主流程
    }
  }

  std::mutex mutex_;
  cv::Mat depth_, labels_;
  sensor_msgs::msg::CameraInfo info_;
  DetectedObjectArray objects_;
  rclcpp::Time depth_stamp_{0, 0, RCL_ROS_TIME};
  std::string depth_frame_;
  bool have_depth_ = false, have_labels_ = false, have_info_ = false;
  bool have_objects_ = false, pending_ = false, enabled_ = true, running_ = true;
  std::chrono::steady_clock::time_point last_attempt_{};
  uint64_t sequence_ = 0;

  std::string target_frame_, conda_python_, predict_script_, weights_, work_root_;
  double min_interval_ = 5.0;
  int min_pixels_ = 50, keep_work_dirs_ = 2;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<GraspCandidate>::SharedPtr candidates_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, labels_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<DetectedObjectArray>::SharedPtr objects_sub_;
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspCandidatePublisher>());
  rclcpp::shutdown();
  return 0;
}