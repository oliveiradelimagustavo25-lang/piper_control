#include <iostream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <yolo_tools/segmenter.hpp>

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "Usage: yolo_predict MODEL.onnx IMAGE LABELS.png\n"
              << "Labels: 0=background, model class index+1; these are not project class IDs.\n";
    return 2;
  }
  try {
    yolo_tools::Segmenter model(argv[1]);
    const auto image = cv::imread(argv[2]);
    cv::Mat labels = cv::Mat::zeros(image.size(), CV_16UC1);
    for (const auto & detection : model.predict(image)) {
      labels.setTo(detection.class_index + 1, (detection.mask != 0) & (labels == 0));
      std::cout << model.names()[detection.class_index] << " " << detection.confidence << " "
                << detection.box << "\n";
    }
    if (!cv::imwrite(argv[3], labels)) {throw std::runtime_error("Could not write label image");}
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; return 1;
  }
  return 0;
}
