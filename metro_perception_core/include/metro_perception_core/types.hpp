#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace metro_perception_core {
struct PointXYZ {
  double x{0};
  double y{0};
  double z{0};
};
struct FrameContext {
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
struct ObstacleCandidate {
  std::uint64_t id{0};
  PointXYZ center, size, nearest_point;
  double distance_m{0};
  bool distance_valid{false};
  std::uint32_t support_points{0};
};
struct CorridorSegment {
  PointXYZ start, end;
  double width_m{0}, height_m{0};
  bool geometry_valid{false}, coverage_valid{false};
};
struct FrameResult {
  AnalysisStatus status{AnalysisStatus::NOT_IMPLEMENTED};
  std::string reason{"NOT_IMPLEMENTED"};
  std::vector<ObstacleCandidate> candidates;
  std::vector<CorridorSegment> corridor;
  bool evaluation_region_valid{false};
};
struct Assessment {
  State state{State::UNKNOWN};
  std::string reason{"NOT_IMPLEMENTED"};
  bool distance_valid{false};
  double distance_m{0};
};
}  // namespace metro_perception_core
