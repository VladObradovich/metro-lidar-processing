#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace metro_perception_core {
struct PointXYZ {
  double x{0};
  double y{0};
  double z{0};
};
struct RigidTransform {
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  PointXYZ translation;
};
struct FrameContext {
  RigidTransform sensor_to_target;
  bool transform_available{false};
  bool calibration_verified{false};
  bool allow_unverified_calibration{false};
  std::int64_t measurement_time_ns{0};
  std::uint64_t frame_sequence{0};
  PointXYZ sensor_origin;
};
struct FrameInput {
  std::vector<PointXYZ> points;
  FrameContext context;
};
enum class AnalysisStatus : std::uint8_t {
  OK = 0,
  NOT_IMPLEMENTED = 1,
  BAD_INPUT = 2,
  TF_UNAVAILABLE = 3,
  INVALID_GEOMETRY = 4
};
enum class State : std::uint8_t { UNKNOWN = 0, OBSTACLE = 1, NO_OBSTACLE_DETECTED = 2 };
enum class CalibrationTrust : std::uint8_t { UNKNOWN = 0, ASSUMED = 1, VERIFIED = 2 };
struct ObstacleCandidate {
  // Evidence that produced the candidate: MOTION (new relative to the rolling baseline)
  // and/or GAUGE (anything inside the narrow route gauge, whatever the history says).
  static constexpr std::uint8_t kMotion = 1, kGauge = 2;
  std::uint64_t id{0};
  PointXYZ center, size, nearest_point;
  double distance_m{0};
  bool distance_valid{false};
  std::uint32_t support_points{0};
  std::uint8_t channels{0};
};
// Route centre ahead, y = c1 * x + c2 * x^2, estimated from the tunnel walls. Invalid means
// the straight corridor y = 0 is used and the gauge channel is off.
struct RouteEstimate {
  double c1{0}, c2{0};
  bool valid{false};
  double max_x{0};  // Farthest wall support; the gauge channel stops here.
  double center(double x) const { return c1 * x + c2 * x * x; }
  double offset(const PointXYZ& p) const { return p.y - center(p.x); }
};
struct CorridorSegment {
  PointXYZ start, end;
  double width_m{0}, height_m{0};
  std::array<double, 4> ground_plane{0, 0, 1, 0};
  std::uint32_t ground_inliers{0};  // Floor returns within the supported extent.
  bool geometry_valid{false}, coverage_valid{false};
};
struct IndexedPoint {
  PointXYZ point;
  std::size_t raw_index;
};
struct PreprocessedFrame {
  std::vector<IndexedPoint> geometry_points;
  // Indices into geometry_points, whose raw_index refers to row-major input XYZ.
  std::vector<std::size_t> detection_indices;
  PointXYZ sensor_origin;
  std::size_t invalid_points{0}, blind_points{0}, outside_roi_points{0};
  bool transform_applied{false};
};
struct FrameResult {
  PreprocessedFrame preprocessed;
  AnalysisStatus status{AnalysisStatus::NOT_IMPLEMENTED};
  std::string reason{"NOT_IMPLEMENTED"};
  std::vector<ObstacleCandidate> candidates;
  std::vector<CorridorSegment> corridor;
  bool evaluation_region_valid{false};
  double evaluated_range_m{0};
  CalibrationTrust calibration_trust{CalibrationTrust::UNKNOWN};
  RouteEstimate route;
  // Lidar-only forward speed used to move the range baseline; diagnostic only.
  bool ego_motion_valid{false};
  double ego_speed_mps{0};
};
struct Assessment {
  State state{State::UNKNOWN};
  std::string reason{"NOT_IMPLEMENTED"};
  bool distance_valid{false};
  // NaN unless distance_valid: a missing distance is never reported as zero.
  double distance_m{std::numeric_limits<double>::quiet_NaN()};
  CalibrationTrust calibration_trust{CalibrationTrust::UNKNOWN};
};
}  // namespace metro_perception_core
