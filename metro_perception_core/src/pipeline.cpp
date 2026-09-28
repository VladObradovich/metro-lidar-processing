#include "metro_perception_core/pipeline.hpp"

#include <cmath>

#include "metro_perception_core/detector.hpp"
namespace metro_perception_core {
namespace {
bool finite(const PointXYZ& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
bool inside(const PointXYZ& p, const Bounds& b) {
  return p.x >= b.min[0] && p.x <= b.max[0] && p.y >= b.min[1] && p.y <= b.max[1] &&
         p.z >= b.min[2] && p.z <= b.max[2];
}
bool valid(const RigidTransform& t) {
  if (!finite(t.translation)) return false;
  const auto& r = t.rotation;
  for (double v : r)
    if (!std::isfinite(v)) return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double dot = 0;
      for (int k = 0; k < 3; ++k) dot += r[3 * i + k] * r[3 * j + k];
      if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-6) return false;
    }
  const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                     r[2] * (r[3] * r[7] - r[4] * r[6]);
  return std::abs(det - 1.0) <= 1e-6;
}
}  // namespace
PerceptionPipeline::PerceptionPipeline(AlgorithmConfig config) : config_(config) {
  config_.validate();
}
FrameResult PerceptionPipeline::process(const FrameInput& frame) {
  FrameResult result;
  auto fail = [&](AnalysisStatus status, const char* reason) {
    detector_.reset();
    result.status = status;
    result.reason = reason;
    return result;
  };
  if (frame.points.size() > config_.max_points)
    return fail(AnalysisStatus::BAD_INPUT, "POINT_LIMIT_EXCEEDED");
  if (frame.points.empty()) return fail(AnalysisStatus::BAD_INPUT, "EMPTY_INPUT");
  if (!frame.context.transform_available)
    return fail(AnalysisStatus::TF_UNAVAILABLE, "TF_UNAVAILABLE");
  const auto& t = frame.context.sensor_to_target;
  if (!valid(t)) return fail(AnalysisStatus::INVALID_GEOMETRY, "INVALID_TRANSFORM");
  if (frame.context.calibration_verified) {
    result.calibration_trust = CalibrationTrust::VERIFIED;
  } else if (frame.context.allow_unverified_calibration) {
    result.calibration_trust = CalibrationTrust::ASSUMED;
  }
  auto& out = result.preprocessed;
  out.sensor_origin = t.translation;
  out.transform_applied = true;
  out.geometry_points.reserve(frame.points.size());
  const auto& r = t.rotation;
  for (std::size_t index = 0; index < frame.points.size(); ++index) {
    const auto& p = frame.points[index];
    if (!finite(p) || (p.x == 0 && p.y == 0 && p.z == 0)) {
      ++out.invalid_points;
      continue;
    }
    if (std::hypot(p.x, p.y, p.z) < config_.blind_radius_m) {
      ++out.blind_points;
      continue;
    }
    const PointXYZ q{r[0] * p.x + r[1] * p.y + r[2] * p.z + t.translation.x,
                     r[3] * p.x + r[4] * p.y + r[5] * p.z + t.translation.y,
                     r[6] * p.x + r[7] * p.y + r[8] * p.z + t.translation.z};
    if (!finite(q)) return fail(AnalysisStatus::INVALID_GEOMETRY, "TRANSFORM_OVERFLOW");
    if (!inside(q, config_.geometry_roi)) {
      ++out.outside_roi_points;
      continue;
    }
    if (inside(q, config_.detection_roi))
      out.detection_indices.push_back(out.geometry_points.size());
    out.geometry_points.push_back({q, index});
  }
  if (!frame.context.calibration_verified && !frame.context.allow_unverified_calibration)
    return fail(AnalysisStatus::INVALID_GEOMETRY, "CALIBRATION_UNVERIFIED");
  if (out.geometry_points.empty())
    return fail(AnalysisStatus::INVALID_GEOMETRY, "EMPTY_GEOMETRY_ROI");
  detector_.process(config_, result, frame.context.measurement_time_ns);
  return result;
}
}  // namespace metro_perception_core
