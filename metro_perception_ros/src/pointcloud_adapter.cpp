#include "metro_perception_ros/pointcloud_adapter.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <new>
#include <stdexcept>

#include "metro_perception_ros/measurement_time.hpp"
namespace metro_perception_ros {
namespace {
using Field = sensor_msgs::msg::PointField;

std::uint64_t scalar_bytes(std::uint8_t datatype) {
  if (datatype == Field::FLOAT32) return 4;
  if (datatype == Field::FLOAT64) return 8;
  return 0;
}

Field coordinate_field(const sensor_msgs::msg::PointCloud2& msg, const char* name) {
  const Field* match = nullptr;
  for (const auto& field : msg.fields) {
    if (field.name != name) continue;
    if (match) throw std::invalid_argument("Duplicate XYZ field");
    match = &field;
  }
  if (!match || match->count != 1 || scalar_bytes(match->datatype) == 0) {
    throw std::invalid_argument("XYZ must be scalar FLOAT32/FLOAT64");
  }
  const auto bytes = scalar_bytes(match->datatype);
  if (std::uint64_t(match->offset) + bytes > msg.point_step) {
    throw std::invalid_argument("XYZ field exceeds point_step");
  }
  return *match;
}

bool fields_overlap(const Field& a, const Field& b) {
  const auto a_begin = std::uint64_t(a.offset);
  const auto a_end = a_begin + scalar_bytes(a.datatype);
  const auto b_begin = std::uint64_t(b.offset);
  const auto b_end = b_begin + scalar_bytes(b.datatype);
  return a_begin < b_end && b_begin < a_end;
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

void validate_pointcloud_limits(std::size_t max_points, std::size_t max_cloud_bytes) {
  if (max_points == 0 || max_points > kMaxPointLimit) {
    throw std::invalid_argument("max_points must be in [1, 10000000]");
  }
  if (max_cloud_bytes == 0 || max_cloud_bytes > kMaxCloudByteLimit) {
    throw std::invalid_argument("max_cloud_bytes must be in [1, 1073741824]");
  }
}

metro_perception_core::FrameInput decode_cloud(const sensor_msgs::msg::PointCloud2& msg,
                                               std::size_t max_points,
                                               std::size_t max_cloud_bytes) {
  validate_pointcloud_limits(max_points, max_cloud_bytes);
  if (msg.data.size() > max_cloud_bytes) {
    throw std::invalid_argument("POINTCLOUD_BYTE_LIMIT_EXCEEDED");
  }

  metro_perception_core::FrameInput frame;
  const auto measurement_time = decode_measurement_time_ns(msg.header.stamp);
  if (!measurement_time) throw std::invalid_argument("INVALID_TIMESTAMP");
  frame.context.measurement_time_ns = *measurement_time;
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
  for (std::size_t i = 0; i < fields.size(); ++i) {
    for (std::size_t j = i + 1; j < fields.size(); ++j) {
      if (fields_overlap(fields[i], fields[j])) {
        throw std::invalid_argument("XYZ fields overlap");
      }
    }
  }
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
  // Raw XYZ: preserve zeros/NaNs for centralized preprocessing filtering; optional fields are
  // not used yet. Structural corruption has already been rejected above.
  return frame;
}
metro_perception_core::FrameResult process_cloud(
    const sensor_msgs::msg::PointCloud2& message,
    const metro_perception_core::PerceptionPipeline& pipeline, std::size_t max_points,
    const metro_perception_core::FrameContext& context, std::size_t max_cloud_bytes) {
  return process_cloud_with_context(
      message, pipeline, max_points, [&context] { return context; }, max_cloud_bytes);
}

metro_perception_core::FrameResult process_cloud_with_context(
    const sensor_msgs::msg::PointCloud2& message,
    const metro_perception_core::PerceptionPipeline& pipeline, std::size_t max_points,
    const std::function<metro_perception_core::FrameContext()>& resolve,
    std::size_t max_cloud_bytes) {
  try {
    auto frame = decode_cloud(message, max_points, max_cloud_bytes);
    const auto measurement_time_ns = frame.context.measurement_time_ns;
    if (!frame.points.empty()) frame.context = resolve();
    frame.context.measurement_time_ns = measurement_time_ns;
    return pipeline.process(frame);
  } catch (const std::invalid_argument& e) {
    metro_perception_core::FrameResult result;
    result.status = metro_perception_core::AnalysisStatus::BAD_INPUT;
    result.reason = e.what();
    return result;
  } catch (const std::length_error&) {
    metro_perception_core::FrameResult result;
    result.status = metro_perception_core::AnalysisStatus::BAD_INPUT;
    result.reason = "RESOURCE_EXHAUSTED";
    return result;
  } catch (const std::bad_alloc&) {
    metro_perception_core::FrameResult result;
    result.status = metro_perception_core::AnalysisStatus::BAD_INPUT;
    result.reason = "RESOURCE_EXHAUSTED";
    return result;
  }
}
}  // namespace metro_perception_ros
