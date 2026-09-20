#pragma once
#include <cstddef>

#include "metro_perception_core/pipeline.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
namespace metro_perception_ros {
metro_perception_core::FrameInput decode_cloud(const sensor_msgs::msg::PointCloud2& message,
                                               std::size_t max_points = 2000000);
// Shared online/offline decoding and preprocessing path; no latest-TF fallback.
metro_perception_core::FrameResult process_cloud(
    const sensor_msgs::msg::PointCloud2& message,
    const metro_perception_core::PerceptionPipeline& pipeline, std::size_t max_points = 2000000,
    const metro_perception_core::FrameContext& context = {});
}  // namespace metro_perception_ros
