#include "metro_perception_ros/preprocessing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "metro_perception_ros/measurement_time.hpp"
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

bool valid_frame_name(const std::string& frame) {
  return !frame.empty() && frame[0] != '/' && frame != "*";
}

}  // namespace

PreprocessingConfig load_preprocessing(const std::string& path) {
  PreprocessingConfig c;
  const auto file = path.empty()
                        ? ament_index_cpp::get_package_share_directory("metro_perception_ros") +
                              "/config/forward_sector_assumed.yaml"
                        : path;
  const auto n = YAML::LoadFile(file);

  const auto mode = n["source_frame_mode"] ? n["source_frame_mode"].as<std::string>() : "exact";
  if (mode == "exact") {
    c.source_frame_mode = SourceFrameMode::EXACT;
  } else if (mode == "bind_first") {
    c.source_frame_mode = SourceFrameMode::BIND_FIRST;
  } else {
    throw std::invalid_argument("source_frame_mode must be exact or bind_first");
  }

  if (n["source_frame"] && !n["source_frame"].IsNull()) {
    c.source_frame = n["source_frame"].as<std::string>();
  }
  // Backward-compatible reading of the old wildcard syntax. New profiles should use bind_first.
  if (c.source_frame == "*") {
    c.source_frame_mode = SourceFrameMode::BIND_FIRST;
    c.source_frame.clear();
  }

  c.target_frame = n["target_frame"].as<std::string>();
  if (!valid_frame_name(c.target_frame)) throw std::invalid_argument("Invalid target frame");
  if (c.source_frame_mode == SourceFrameMode::EXACT) {
    if (!valid_frame_name(c.source_frame)) throw std::invalid_argument("Invalid source frame");
  } else if (!c.source_frame.empty()) {
    throw std::invalid_argument("bind_first profile must not hard-code source_frame");
  }

  c.calibration_verified = n["calibration_verified"].as<bool>();
  c.allow_unverified_calibration =
      n["allow_unverified_calibration"] ? n["allow_unverified_calibration"].as<bool>() : false;
  if (c.calibration_verified && c.source_frame_mode == SourceFrameMode::BIND_FIRST) {
    throw std::invalid_argument("Verified calibration requires exact source_frame");
  }

  if (n["translation_m"] && !n["translation_m"].IsNull()) {
    if (c.source_frame_mode == SourceFrameMode::EXACT && c.source_frame == c.target_frame)
      throw std::invalid_argument("Static TF needs distinct frames");
    const auto xyz = triple(n["translation_m"]);
    const auto rpy = triple(n["rotation_rpy_rad"]);
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    auto& t = c.static_transform;
    t.header.frame_id = c.target_frame;
    if (c.source_frame_mode == SourceFrameMode::EXACT) t.child_frame_id = c.source_frame;
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
  if (const auto detector = n["detector"]) {
    if (!detector.IsMap()) throw std::invalid_argument("detector must be a mapping");
    auto read_double = [&](const char* key, double& value) {
      if (detector[key]) value = detector[key].as<double>();
    };
    auto read_count = [&](const char* key, std::size_t& value) {
      if (detector[key]) value = detector[key].as<std::size_t>();
    };
    read_double("corridor_half_width_m", c.algorithm.corridor_half_width_m);
    read_double("corridor_height_m", c.algorithm.corridor_height_m);
    read_double("ground_max_slope", c.algorithm.ground_max_slope);
    read_double("ground_inlier_tolerance_m", c.algorithm.ground_inlier_tolerance_m);
    read_double("ground_max_gap_m", c.algorithm.ground_max_gap_m);
    read_count("ground_min_bin_points", c.algorithm.ground_min_bin_points);
    read_double("obstacle_min_height_m", c.algorithm.obstacle_min_height_m);
    read_double("angular_cell_deg", c.algorithm.angular_cell_deg);
    read_count("min_ground_inliers", c.algorithm.min_ground_inliers);
    read_count("min_candidate_cells", c.algorithm.min_candidate_cells);
    read_count("min_candidate_points", c.algorithm.min_candidate_points);
    read_count("background_history_frames", c.algorithm.background_history_frames);
    read_count("background_lag_frames", c.algorithm.background_lag_frames);
    read_double("background_margin_m", c.algorithm.background_margin_m);
    read_double("background_relative_margin", c.algorithm.background_relative_margin);
  }
  if (c.source_frame_mode == SourceFrameMode::BIND_FIRST && !c.has_static_transform)
    throw std::invalid_argument("bind_first requires an explicit static transform");

  c.algorithm.validate();
  return c;
}

bool bind_source_frame(const PreprocessingConfig& c, SourceFrameBinding& binding,
                       const std::string& frame) {
  if (!valid_frame_name(frame)) return false;
  if (c.source_frame_mode == SourceFrameMode::EXACT) return frame == c.source_frame;
  if (frame == c.target_frame) return false;
  if (binding.frame_id.empty()) binding.frame_id = frame;
  return binding.frame_id == frame;
}

std::optional<geometry_msgs::msg::TransformStamped> resolved_static_transform(
    const PreprocessingConfig& c, const SourceFrameBinding& binding) {
  if (!c.has_static_transform) return std::nullopt;
  auto transform = c.static_transform;
  if (c.source_frame_mode == SourceFrameMode::BIND_FIRST) {
    if (binding.frame_id.empty()) return std::nullopt;
    transform.child_frame_id = binding.frame_id;
  }
  return transform;
}

metro_perception_core::FrameContext resolve_context(const std_msgs::msg::Header& h,
                                                    const PreprocessingConfig& c,
                                                    const SourceFrameBinding& binding,
                                                    tf2_ros::Buffer& buffer,
                                                    std::chrono::nanoseconds tf_wait_timeout) {
  metro_perception_core::FrameContext context;
  const auto measurement_time = decode_measurement_time_ns(h.stamp);
  if (measurement_time) context.measurement_time_ns = *measurement_time;
  context.calibration_verified = c.calibration_verified;
  context.allow_unverified_calibration = c.allow_unverified_calibration;

  const auto& expected_source =
      c.source_frame_mode == SourceFrameMode::EXACT ? c.source_frame : binding.frame_id;

  // ROS zero time requests latest TF. Never silently use it for a measurement.
  if (!measurement_time || !valid_frame_name(h.frame_id) || expected_source.empty() ||
      h.frame_id != expected_source)
    return context;

  if (h.frame_id == c.target_frame) {
    context.sensor_to_target = {};
    context.sensor_origin = {};
    context.transform_available = true;
    return context;
  }

  try {
    auto& core = static_cast<tf2::BufferCore&>(buffer);
    const auto lookup_time = tf2::TimePoint(std::chrono::nanoseconds(*measurement_time));
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
