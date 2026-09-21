#include "metro_perception_ros/preprocessing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"
namespace metro_perception_ros {
namespace {
std::array<double, 3> triple(const YAML::Node& n) {
  if (!n.IsSequence() || n.size() != 3)
    throw std::invalid_argument("Expected three finite numbers");
  std::array<double, 3> v;
  for (int i = 0; i < 3; ++i) {
    v[i] = n[i].as<double>();
    if (!std::isfinite(v[i])) throw std::invalid_argument("Non-finite configuration");
  }
  return v;
}
}  // namespace
PreprocessingConfig load_preprocessing(const std::string& path) {
  PreprocessingConfig c;
  const auto file = path.empty()
                        ? ament_index_cpp::get_package_share_directory("metro_perception_ros") +
                              "/config/forward_sector_assumed.yaml"
                        : path;
  const auto n = YAML::LoadFile(file);
  c.source_frame = n["source_frame"].as<std::string>();
  c.target_frame = n["target_frame"].as<std::string>();
  if (c.source_frame.empty() || c.target_frame.empty() || c.source_frame[0] == '/' ||
      c.target_frame[0] == '/')
    throw std::invalid_argument("Invalid source/target frame");
  c.calibration_verified = n["calibration_verified"].as<bool>();
  c.allow_unverified_calibration =
      n["allow_unverified_calibration"] ? n["allow_unverified_calibration"].as<bool>() : false;
  if (n["translation_m"] && !n["translation_m"].IsNull()) {
    if (c.source_frame == c.target_frame)
      throw std::invalid_argument("Static TF needs distinct frames");
    auto xyz = triple(n["translation_m"]);
    auto rpy = triple(n["rotation_rpy_rad"]);
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    auto& t = c.static_transform;
    t.header.frame_id = c.target_frame;
    t.child_frame_id = c.source_frame;
    t.transform.translation.x = xyz[0];
    t.transform.translation.y = xyz[1];
    t.transform.translation.z = xyz[2];
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    c.has_static_transform = true;
  } else if (n["rotation_rpy_rad"] && !n["rotation_rpy_rad"].IsNull()) {
    throw std::invalid_argument("rotation requires translation");
  }
  if (c.calibration_verified && (!n["calibration_source"] || n["calibration_source"].IsNull() ||
                                 n["calibration_source"].as<std::string>().empty()))
    throw std::invalid_argument("Verified calibration needs calibration_source");
  if (n["blind_radius_m"]) c.algorithm.blind_radius_m = n["blind_radius_m"].as<double>();
  for (auto item : {std::make_pair("geometry_roi", &c.algorithm.geometry_roi),
                    std::make_pair("detection_roi", &c.algorithm.detection_roi)}) {
    if (n[item.first]) {
      item.second->min = triple(n[item.first]["min"]);
      item.second->max = triple(n[item.first]["max"]);
    }
  }
  if (c.source_frame == "*" && !c.has_static_transform)
    throw std::invalid_argument("Wildcard source requires an explicit static transform");
  c.algorithm.validate();
  return c;
}
void bind_source_frame(PreprocessingConfig& c, const std::string& frame) {
  if (c.source_frame != "*" || frame.empty() || frame[0] == '/' || frame == "*" ||
      frame == c.target_frame)
    return;
  c.source_frame = frame;
  c.static_transform.child_frame_id = frame;
}
metro_perception_core::FrameContext resolve_context(
    const std_msgs::msg::Header& h, const PreprocessingConfig& c, tf2_ros::Buffer& buffer,
    std::chrono::nanoseconds tf_wait_timeout) {
  metro_perception_core::FrameContext context;
  context.measurement_time_ns = std::int64_t(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;
  context.calibration_verified = c.calibration_verified;
  context.allow_unverified_calibration = c.allow_unverified_calibration;
  // ROS zero time requests latest TF. Never silently use it for a measurement.
  if (context.measurement_time_ns <= 0 || h.stamp.nanosec >= 1000000000u || h.frame_id.empty() ||
      (!c.source_frame.empty() && h.frame_id != c.source_frame))
    return context;
  if (h.frame_id == c.target_frame) {
    context.sensor_to_target = {};
    context.sensor_origin = {};
    context.transform_available = true;
    return context;
  }
  try {
    auto& core = static_cast<tf2::BufferCore&>(buffer);
    const auto lookup_time =
        tf2::TimePoint(std::chrono::nanoseconds(context.measurement_time_ns));
    if (tf_wait_timeout > std::chrono::nanoseconds::zero()) {
      const auto deadline = std::chrono::steady_clock::now() + tf_wait_timeout;
      while (!core.canTransform(c.target_frame, h.frame_id, lookup_time) &&
             std::chrono::steady_clock::now() < deadline) {
        const auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::steady_clock::duration::zero()) break;
        std::this_thread::sleep_for(
            std::min(remaining, std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::milliseconds(2))));
      }
    }
    const auto t = core.lookupTransform(c.target_frame, h.frame_id, lookup_time);
    const auto& q = t.transform.rotation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    if (!std::isfinite(quaternion.length2()) || std::abs(quaternion.length2() - 1) > 1e-6)
      return context;
    tf2::Matrix3x3 r(quaternion);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) context.sensor_to_target.rotation[3 * i + j] = r[i][j];
    context.sensor_to_target.translation = {t.transform.translation.x, t.transform.translation.y,
                                            t.transform.translation.z};
    context.sensor_origin = context.sensor_to_target.translation;
    context.transform_available = true;
  } catch (const tf2::TransformException&) {
    // Missing, stale, disconnected or future TF after the bounded wait is rejected.
  }
  return context;
}
}  // namespace metro_perception_ros
