#include <yolo_tools/segmenter.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <set>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace yolo_tools
{
struct Segmenter::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo_tools"};
  Ort::Session session{nullptr};
  std::string input_name;
  std::vector<std::string> output_names;
  std::vector<std::string> names;
  cv::Size input_size;
  Options options;

  Impl(const std::string & model, const Options & value) : options(value)
  {
    for (float threshold : {options.confidence, options.iou_threshold, options.mask_threshold}) {
      if (!std::isfinite(threshold) || threshold <= 0 || threshold >= 1) {
        throw std::invalid_argument("YOLO thresholds must be finite and between 0 and 1");
      }
    }
    if (options.threads < 1) {throw std::invalid_argument("YOLO threads must be positive");}
    Ort::SessionOptions settings;
    settings.SetIntraOpNumThreads(options.threads);
    settings.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session = Ort::Session(env, model.c_str(), settings);
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 2) {
      throw std::invalid_argument("Expected segmentation ONNX with one input and two outputs");
    }
    // Keep the owning TypeInfo alive while using its tensor view.
    const auto input = session.GetInputTypeInfo(0);
    const auto tensor = input.GetTensorTypeAndShapeInfo();
    const auto shape = tensor.GetShape();
    if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || shape.size() != 4 ||
      shape[0] != 1 || shape[1] != 3 || shape[2] <= 0 || shape[3] <= 0)
    {
      throw std::invalid_argument("Export a static batch=1, FP32 NCHW segmentation ONNX model");
    }
    input_size = {static_cast<int>(shape[3]), static_cast<int>(shape[2])};
    Ort::AllocatorWithDefaultOptions allocator;
    input_name = session.GetInputNameAllocated(0, allocator).get();
    for (size_t i = 0; i < 2; ++i) {
      output_names.emplace_back(session.GetOutputNameAllocated(i, allocator).get());
    }
    const auto metadata = session.GetModelMetadata();
    const auto task = metadata.LookupCustomMetadataMapAllocated("task", allocator);
    if (task && std::string(task.get()) != "segment") {
      throw std::invalid_argument("YOLO model task must be segment");
    }
    const auto labels = metadata.LookupCustomMetadataMapAllocated("names", allocator);
    if (!labels) {throw std::invalid_argument("ONNX names metadata is missing; use Ultralytics export");}
    const auto mapping = YAML::Load(labels.get());
    if ((!mapping.IsMap() && !mapping.IsSequence()) || mapping.size() == 0) {
      throw std::invalid_argument("Invalid ONNX names metadata");
    }
    std::set<std::string> unique;
    for (size_t i = 0; i < mapping.size(); ++i) {
      auto name = mapping[static_cast<int>(i)].as<std::string>();
      if (name.empty() || !unique.insert(name).second) {
        throw std::invalid_argument("ONNX class names must be nonempty and unique");
      }
      names.push_back(std::move(name));
    }
  }

  std::vector<Detection> predict(const cv::Mat & bgr)
  {
    if (bgr.empty() || bgr.type() != CV_8UC3) {
      throw std::invalid_argument("YOLO input must be a nonempty BGR8 image");
    }
    const float gain = std::min(float(input_size.width) / bgr.cols, float(input_size.height) / bgr.rows);
    const cv::Size resized(std::max(1, int(std::round(bgr.cols * gain))),
                          std::max(1, int(std::round(bgr.rows * gain))));
    const int left = (input_size.width - resized.width) / 2;
    const int top = (input_size.height - resized.height) / 2;
    cv::Mat image(input_size, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::resize(bgr, image(cv::Rect(left, top, resized.width, resized.height)), resized);
    auto blob = cv::dnn::blobFromImage(image, 1.0 / 255.0, input_size, {}, true, false);
    const std::array<int64_t, 4> shape{1, 3, input_size.height, input_size.width};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(memory, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
    const char * input_names[] = {input_name.c_str()};
    const char * outputs[] = {output_names[0].c_str(), output_names[1].c_str()};
    auto values = session.Run(Ort::RunOptions{nullptr}, input_names, &input, 1, outputs, 2);
    auto prediction_shape = values[0].GetTensorTypeAndShapeInfo().GetShape();
    auto proto_shape = values[1].GetTensorTypeAndShapeInfo().GetShape();
    if (values[0].GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      values[1].GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      prediction_shape.size() != 3 || prediction_shape[0] != 1 ||
      proto_shape.size() != 4 || proto_shape[0] != 1 ||
      proto_shape[1] <= 0 || proto_shape[2] <= 0 || proto_shape[3] <= 0)
    {
      throw std::runtime_error("Unexpected segmentation output types or shapes");
    }
    const int channels = static_cast<int>(proto_shape[1]);
    const bool end_to_end = prediction_shape[2] == 6 + channels;
    if (!end_to_end && prediction_shape[1] != 4 + int(names.size()) + channels) {
      throw std::runtime_error("Unsupported segmentation output layout; check export settings");
    }
    cv::Mat raw(static_cast<int>(prediction_shape[1]), static_cast<int>(prediction_shape[2]),
                CV_32F, values[0].GetTensorMutableData<float>());
    cv::Mat rows;
    if (end_to_end) {rows = raw;} else {cv::transpose(raw, rows);}
    std::vector<Detection> candidates;
    std::vector<cv::Mat> coefficients;
    for (int i = 0; i < rows.rows; ++i) {
      const auto * row = rows.ptr<float>(i);
      if (end_to_end && (!std::isfinite(row[5]) || row[5] != std::floor(row[5]))) {
        throw std::runtime_error("Invalid model class index");
      }
      const int index = end_to_end ? int(row[5]) :
        int(std::max_element(row + 4, row + 4 + names.size()) - (row + 4));
      const float score = end_to_end ? row[4] : row[4 + index];
      if (!std::isfinite(score) || score < options.confidence) {continue;}
      if (index < 0 || index >= int(names.size())) {throw std::runtime_error("Invalid model class index");}
      if (!std::all_of(row, row + 4, [](float x) {return std::isfinite(x);})) {continue;}
      const float x1 = end_to_end ? row[0] : row[0] - row[2] / 2;
      const float y1 = end_to_end ? row[1] : row[1] - row[3] / 2;
      const float x2 = end_to_end ? row[2] : row[0] + row[2] / 2;
      const float y2 = end_to_end ? row[3] : row[1] + row[3] / 2;
      const float x = std::clamp((x1 - left) / gain, 0.0F, float(bgr.cols));
      const float y = std::clamp((y1 - top) / gain, 0.0F, float(bgr.rows));
      const float right = std::clamp((x2 - left) / gain, 0.0F, float(bgr.cols));
      const float bottom = std::clamp((y2 - top) / gain, 0.0F, float(bgr.rows));
      if (right <= x || bottom <= y) {continue;}
      candidates.push_back({index, score, {x, y, right - x, bottom - y}, {}});
      coefficients.push_back(rows.row(i).colRange(end_to_end ? 6 : 4 + int(names.size()), rows.cols));
    }
    std::vector<int> keep;
    if (end_to_end) {
      keep.resize(candidates.size());
      std::iota(keep.begin(), keep.end(), 0);
    } else {
      for (size_t index = 0; index < names.size(); ++index) {
        std::vector<cv::Rect2d> boxes;
        std::vector<float> scores;
        std::vector<int> indices, selected;
        for (size_t i = 0; i < candidates.size(); ++i) {
          if (candidates[i].class_index == int(index)) {
            indices.push_back(int(i)); boxes.push_back(candidates[i].box);
            scores.push_back(candidates[i].confidence);
          }
        }
        cv::dnn::NMSBoxes(boxes, scores, options.confidence, options.iou_threshold, selected);
        for (int i : selected) {keep.push_back(indices[i]);}
      }
    }
    std::stable_sort(keep.begin(), keep.end(), [&](int a, int b) {
      return candidates[a].confidence > candidates[b].confidence;
    });
    const int height = static_cast<int>(proto_shape[2]), width = static_cast<int>(proto_shape[3]);
    cv::Mat prototypes(channels, height * width, CV_32F, values[1].GetTensorMutableData<float>());
    std::vector<Detection> result;
    const double threshold = std::log(options.mask_threshold / (1 - options.mask_threshold));
    for (int i : keep) {
      cv::Mat logits = coefficients[i] * prototypes;
      cv::Mat scaled, original;
      cv::resize(logits.reshape(1, height), scaled, input_size, 0, 0, cv::INTER_LINEAR);
      cv::resize(scaled(cv::Rect(left, top, resized.width, resized.height)), original,
                 bgr.size(), 0, 0, cv::INTER_LINEAR);
      auto & detection = candidates[i];
      detection.mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
      const auto & box = detection.box;
      const cv::Rect region = cv::Rect(int(std::ceil(box.x)), int(std::ceil(box.y)),
        int(std::ceil(box.x + box.width)) - int(std::ceil(box.x)),
        int(std::ceil(box.y + box.height)) - int(std::ceil(box.y))) & cv::Rect(0, 0, bgr.cols, bgr.rows);
      if (!region.empty()) {
        cv::compare(original(region), threshold, detection.mask(region), cv::CMP_GT);
        result.push_back(std::move(detection));
      }
    }
    return result;
  }
};

Segmenter::Segmenter(const std::string & model, const Options & options)
: impl_(std::make_unique<Impl>(model, options)) {}
Segmenter::~Segmenter() = default;
const std::vector<std::string> & Segmenter::names() const {return impl_->names;}
std::vector<Detection> Segmenter::predict(const cv::Mat & bgr) {return impl_->predict(bgr);}
}  // namespace yolo_tools
