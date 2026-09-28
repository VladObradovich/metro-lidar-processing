#pragma once
#include <cstddef>
#include <functional>

#include "metro_perception_core/pipeline.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace metro_perception_ros {

inline constexpr std::size_t kDefaultMaxPoints = 2000000;
inline constexpr std::size_t kMaxPointLimit = 10000000;
inline constexpr std::size_t kDefaultMaxCloudBytes = 256 * 1024 * 1024;
inline constexpr std::size_t kMaxCloudByteLimit = 1024 * 1024 * 1024;

void validate_pointcloud_limits(std::size_t max_points, std::size_t max_cloud_bytes);

// Resolve/bind only after a nonempty cloud passes decoding and resource limits.
// Input/resource errors from decoding or the resolver become fail-closed frame results.
metro_perception_core::FrameResult process_cloud_with_context(
    const sensor_msgs::msg::PointCloud2& message,
    metro_perception_core::PerceptionPipeline& pipeline, std::size_t max_points,
    const std::function<metro_perception_core::FrameContext()>& resolve,
    std::size_t max_cloud_bytes = kDefaultMaxCloudBytes);

metro_perception_core::FrameInput decode_cloud(const sensor_msgs::msg::PointCloud2& message,
                                               std::size_t max_points = kDefaultMaxPoints,
                                               std::size_t max_cloud_bytes = kDefaultMaxCloudBytes);

// Shared online/offline decoding and preprocessing path; no latest-TF fallback.
metro_perception_core::FrameResult process_cloud(
    const sensor_msgs::msg::PointCloud2& message,
    metro_perception_core::PerceptionPipeline& pipeline, std::size_t max_points = kDefaultMaxPoints,
    const metro_perception_core::FrameContext& context = {},
    std::size_t max_cloud_bytes = kDefaultMaxCloudBytes);

}  // namespace metro_perception_ros
