#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace yolo_tools
{
struct Options
{
  float confidence = 0.25F;
  float iou_threshold = 0.45F;  // Only used for raw, one-to-many exports.
  float mask_threshold = 0.5F;
  int threads = 1;
};

struct Detection
{
  int class_index;
  float confidence;
  cv::Rect2f box;
  cv::Mat mask;  // CV_8UC1, 0/255, in original image coordinates.
};

// Batch-one FP32 Ultralytics segmentation ONNX; no ROS or project class IDs.
class Segmenter
{
public:
  explicit Segmenter(const std::string & model, const Options & options = {});
  ~Segmenter();
  const std::vector<std::string> & names() const;
  std::vector<Detection> predict(const cv::Mat & bgr);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace yolo_tools
