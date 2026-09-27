#pragma once
#include <chrono>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "metro_perception_core/config.hpp"
#include "metro_perception_core/types.hpp"
#include "tf2_ros/buffer.h"

namespace metro_perception_ros {

enum class SourceFrameMode { EXACT, BIND_FIRST };

struct PreprocessingConfig {
  metro_perception_core::AlgorithmConfig algorithm;
  SourceFrameMode source_frame_mode{SourceFrameMode::EXACT};
  std::string source_frame, target_frame{"base_link"};
  bool calibration_verified{false}, has_static_transform{false};
  bool allow_unverified_calibration{false};
  geometry_msgs::msg::TransformStamped static_transform;
};

struct SourceFrameBinding {
  std::string frame_id;
  void reset() { frame_id.clear(); }
};

// The same immutable YAML config and resolver are used by the node and offline evaluator.
PreprocessingConfig load_preprocessing(const std::string& path);
// Temporal confirmation (G4) from the `temporal:` section of the same sensor profile, so the
// online monitor and evaluate_bag always use one rule. An empty path is the default profile.
metro_perception_core::TemporalConfig load_temporal_config(const std::string& path);

// EXACT validates the configured frame. BIND_FIRST captures the first valid frame per session and
// rejects a different frame until SourceFrameBinding::reset() is called.
bool bind_source_frame(const PreprocessingConfig& config, SourceFrameBinding& binding,
                       const std::string& frame);

std::optional<geometry_msgs::msg::TransformStamped> resolved_static_transform(
    const PreprocessingConfig& config, const SourceFrameBinding& binding);

metro_perception_core::FrameContext resolve_context(
    const std_msgs::msg::Header& header, const PreprocessingConfig& config,
    const SourceFrameBinding& binding, tf2_ros::Buffer& buffer,
    std::chrono::nanoseconds tf_wait_timeout = std::chrono::nanoseconds::zero());

}  // namespace metro_perception_ros
