#include "metro_perception_ros/pointcloud_adapter.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
namespace metro_perception_ros {
namespace {
using Field = sensor_msgs::msg::PointField;
Field coordinate_field(const sensor_msgs::msg::PointCloud2& msg, const char* name) {
  const auto it = std::find_if(msg.fields.begin(), msg.fields.end(),
                               [name](const Field& f) { return f.name == name; });
  if (it == msg.fields.end() || it->count != 1 ||
      (it->datatype != Field::FLOAT32 && it->datatype != Field::FLOAT64)) {
    throw std::invalid_argument("XYZ must be scalar FLOAT32/FLOAT64");
  }
  const std::uint64_t bytes = it->datatype == Field::FLOAT32 ? 4 : 8;
  if (std::uint64_t(it->offset) + bytes > msg.point_step) {
    throw std::invalid_argument("XYZ field exceeds point_step");
  }
  return *it;
}
template <typename T>
double read_number(const std::uint8_t* ptr, bool big_endian) {
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::copy_n(ptr, sizeof(T), bytes.begin());
  const std::uint16_t one = 1;
  const bool host_big_endian = *reinterpret_cast<const std::uint8_t*>(&one) == 0;
  if (big_endian != host_big_endian) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return static_cast<double>(value);
}
}  // namespace
metro_perception_core::FrameInput decode_cloud(const sensor_msgs::msg::PointCloud2& msg,
                                               std::size_t max_points) {
  metro_perception_core::FrameInput frame;
  frame.context.measurement_time_ns =
      std::int64_t(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
  const std::uint64_t count = std::uint64_t(msg.width) * msg.height;
  if (count > max_points) {
    throw std::invalid_argument("POINT_LIMIT_EXCEEDED");
  }
  if (count == 0) {
    return frame;
  }
  if (msg.point_step == 0 || std::uint64_t(msg.width) * msg.point_step > msg.row_step ||
      std::uint64_t(msg.row_step) * msg.height > msg.data.size()) {
    throw std::invalid_argument("Invalid PointCloud2 strides or data length");
  }
  const std::array<Field, 3> fields{coordinate_field(msg, "x"), coordinate_field(msg, "y"),
                                    coordinate_field(msg, "z")};
  frame.points.reserve(static_cast<std::size_t>(count));
  for (std::uint32_t row = 0; row < msg.height; ++row) {
    for (std::uint32_t col = 0; col < msg.width; ++col) {
      const auto* ptr =
          msg.data.data() + std::size_t(row) * msg.row_step + std::size_t(col) * msg.point_step;
      std::array<double, 3> xyz;
      for (std::size_t i = 0; i < fields.size(); ++i) {
        xyz[i] = fields[i].datatype == Field::FLOAT32
                     ? read_number<float>(ptr + fields[i].offset, msg.is_bigendian)
                     : read_number<double>(ptr + fields[i].offset, msg.is_bigendian);
      }
      frame.points.push_back({xyz[0], xyz[1], xyz[2]});
    }
  }
  // Raw XYZ: preserve zeros/NaNs for preprocessing; optional fields are not used yet.
  return frame;
}
metro_perception_core::FrameResult process_cloud(
    const sensor_msgs::msg::PointCloud2& message,
    const metro_perception_core::PerceptionPipeline& pipeline, std::size_t max_points,
    const metro_perception_core::FrameContext& context) {
  try {
    auto frame = decode_cloud(message, max_points);
    frame.context = context;
    frame.context.measurement_time_ns =
        std::int64_t(message.header.stamp.sec) * 1000000000LL + message.header.stamp.nanosec;
    return pipeline.process(frame);
  } catch (const std::invalid_argument& e) {
    metro_perception_core::FrameResult result;
    result.status = metro_perception_core::AnalysisStatus::BAD_INPUT;
    result.reason = e.what();
    return result;
  }
}
}  // namespace metro_perception_ros
