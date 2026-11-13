#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace my_robot_perception
{
struct HsvRange
{
  std::array<int, 3> lower, upper;
};

struct ObjectClass
{
  uint16_t id;
  std::string name;
  uint8_t color;
  std::string geometry;
  std::array<double, 3> dimensions;
  std::optional<HsvRange> cv;
};

// 节点共用路径解析与类别读取；launch 只传参，算法参数由各节点管理。
inline std::vector<ObjectClass> loadCatalog(rclcpp::Node & node)
{
  const auto config_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("my_robot_perception")) / "config";
  const auto perception_config = node.declare_parameter<std::string>("perception_config", "");
  auto configured = node.declare_parameter<std::string>("class_config", "");
  // 独立参数保留来源：显式 launch 覆盖相对于工作目录，YAML 值相对于 YAML。
  const auto override = node.declare_parameter<std::string>("class_config_override", "");
  if (override.empty() && configured.empty() && !perception_config.empty()) {
    configured = YAML::LoadFile(perception_config)["/**"]["ros__parameters"]["class_config"].as<std::string>("");
  }
  auto path = std::filesystem::path(override.empty() ? configured : override);
  if (path.empty()) {
    path = config_dir / "object_classes.yaml";
  } else if (path.is_relative()) {
    const auto base = override.empty() && !perception_config.empty() ?
      std::filesystem::absolute(perception_config).parent_path() : std::filesystem::current_path();
    path = base / path;
  }
  path = std::filesystem::canonical(path);
  node.set_parameter(rclcpp::Parameter("class_config", path.string()));
  const auto classes = YAML::LoadFile(path.string())["classes"];
  if (!classes.IsSequence()) {
    throw std::invalid_argument("class_config requires a classes list");
  }
  const std::array<std::string, 4> colors{"", "red", "green", "blue"};
  std::map<int, size_t> ids;
  std::map<std::string, size_t> names;
  for (const auto & entry : classes) {
    try {++ids[entry["id"].as<int>()];} catch (const YAML::Exception &) {}
    try {++names[entry["name"].as<std::string>()];} catch (const YAML::Exception &) {}
  }
  std::vector<ObjectClass> result;
  for (const auto & entry : classes) {
    try {
      const int id = entry["id"].as<int>();
      const auto name = entry["name"].as<std::string>();
      const auto color = std::find(colors.begin() + 1, colors.end(), entry["color"].as<std::string>(""));
      const auto geometry = entry["geometry"];
      if (id < 1 || id > 255 || ids[id] != 1 || name.empty() ||
        names[name] != 1 || geometry["type"].as<std::string>().empty())
      {
        throw std::invalid_argument("Invalid/duplicate class id or name, or missing geometry: " + name);
      }
      ObjectClass object{static_cast<uint16_t>(id), name,
        static_cast<uint8_t>(color == colors.end() ? 0 : color - colors.begin()),
        geometry["type"].as<std::string>(),
        geometry["dimensions"].as<std::array<double, 3>>(), std::nullopt};
      if (!std::all_of(object.dimensions.begin(), object.dimensions.end(),
        [](double value) {return std::isfinite(value) && value > 0;}))
      {
        throw std::invalid_argument(name + ": dimensions must be positive finite lengths in metres");
      }
      if (entry["cv"]) {
        try {
          HsvRange hsv{entry["cv"]["hsv_lower"].as<std::array<int, 3>>(),
            entry["cv"]["hsv_upper"].as<std::array<int, 3>>()};
          for (size_t i = 0; i < 3; ++i) {
            const int limit = i == 0 ? 179 : 255;
            if (hsv.lower[i] < 0 || hsv.upper[i] < 0 || hsv.lower[i] > limit ||
              hsv.upper[i] > limit || (i > 0 && hsv.lower[i] > hsv.upper[i]))
            {
              throw std::invalid_argument(name + ": invalid HSV bounds");
            }
          }
          object.cv = hsv;
        } catch (const std::exception & error) {
          RCLCPP_WARN(node.get_logger(), "Class %d CV disabled: %s", id, error.what());
        }
      }
      result.push_back(object);
      RCLCPP_INFO(node.get_logger(), "Class %u=%s color=%u dimensions=[%.3f,%.3f,%.3f] CV=%s",
        object.id, object.name.c_str(), object.color,
        object.dimensions[0], object.dimensions[1], object.dimensions[2], object.cv ? "yes" : "no");
    } catch (const std::exception & error) {
      RCLCPP_WARN(node.get_logger(), "Skipping invalid class: %s", error.what());
    }
  }
  RCLCPP_INFO(node.get_logger(), "Loaded %zu classes from %s", result.size(), path.c_str());
  return result;
}
}  // namespace my_robot_perception
