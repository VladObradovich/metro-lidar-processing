#pragma once
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "metro_perception_core/config.hpp"
#include "metro_perception_core/types.hpp"
#include "tf2_ros/buffer.h"
namespace metro_perception_ros {
struct PreprocessingConfig {
  metro_perception_core::AlgorithmConfig algorithm;
  std::string source_frame, target_frame{"base_link"};
  bool calibration_verified{false}, has_static_transform{false};
  bool allow_unverified_calibration{false};
  geometry_msgs::msg::TransformStamped static_transform;
};
// The same YAML and resolver are used by the node and offline evaluator.
PreprocessingConfig load_preprocessing(const std::string& path);
// Wildcard profiles bind once; a later frame change is rejected by resolve_context.
void bind_source_frame(PreprocessingConfig& config, const std::string& frame);
metro_perception_core::FrameContext resolve_context(const std_msgs::msg::Header& header,
                                                    const PreprocessingConfig& config,
                                                    tf2_ros::Buffer& buffer);
}  // namespace metro_perception_ros
