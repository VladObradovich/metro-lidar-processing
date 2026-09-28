#include "metro_perception_ros/labelled_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "metro_perception_core/labels.hpp"

namespace metro_perception_ros {
namespace {
constexpr double kShadeRangeM = 100.0;  // Grey level falls from near to this range.
sensor_msgs::msg::PointField field(const char* name, std::uint32_t offset, std::uint8_t type) {
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}
}  // namespace

sensor_msgs::msg::PointCloud2 make_labelled_cloud(const std_msgs::msg::Header& header,
                                                  const metro_perception_core::FrameResult& frame,
                                                  const std::vector<std::uint8_t>& labels) {
  using sensor_msgs::msg::PointField;
  const auto& points = frame.preprocessed.geometry_points;
  const auto& origin = frame.preprocessed.sensor_origin;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.fields = {field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32),
                  field("z", 8, PointField::FLOAT32), field("rgb", 12, PointField::FLOAT32),
                  field("label", 16, PointField::UINT8)};
  cloud.is_bigendian = false;
  cloud.point_step = 20;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = true;
  cloud.data.assign(static_cast<std::size_t>(cloud.row_step), 0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i].point;
    const double range = std::hypot(p.x - origin.x, p.y - origin.y, p.z - origin.z);
    const double shade = 1.0 - std::clamp(range / kShadeRangeM, 0.0, 1.0);
    const auto level = static_cast<std::uint8_t>(45 + 150 * shade);
    const std::uint8_t label = i < labels.size() ? labels[i] : 0;
    std::uint8_t r = level, g = level, b = level;
    if (label == metro_perception_core::kCorridorPoint) {
      r = 20, g = static_cast<std::uint8_t>(110 + 145 * shade), b = 50;
    } else if (label == metro_perception_core::kObstaclePoint) {
      r = 255, g = 40, b = 40;
    }
    const float xyz[3] = {static_cast<float>(p.x), static_cast<float>(p.y),
                          static_cast<float>(p.z)};
    const std::uint32_t rgb = (std::uint32_t(r) << 16) | (std::uint32_t(g) << 8) | b;
    auto* out = cloud.data.data() + i * cloud.point_step;
    std::memcpy(out, xyz, sizeof(xyz));
    std::memcpy(out + 12, &rgb, sizeof(rgb));
    out[16] = label;
  }
  return cloud;
}
}  // namespace metro_perception_ros
