#include "metro_perception_core/labels.hpp"

#include <algorithm>
#include <cmath>

namespace metro_perception_core {
std::vector<std::uint8_t> label_points(const AlgorithmConfig& config, const FrameResult& result) {
  constexpr double kBoxMarginM = 0.1, kBelowFloorM = 0.3;
  const auto& points = result.preprocessed.geometry_points;
  std::vector<std::uint8_t> labels(points.size(), kBackgroundPoint);
  double corridor_end = -INFINITY;
  for (const auto& segment : result.corridor)
    corridor_end = std::max({corridor_end, segment.start.x, segment.end.x});
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i].point;
    const bool obstacle = std::any_of(
        result.candidates.begin(), result.candidates.end(), [&](const ObstacleCandidate& c) {
          return std::abs(p.x - c.center.x) <= c.size.x / 2 + kBoxMarginM &&
                 std::abs(p.y - c.center.y) <= c.size.y / 2 + kBoxMarginM &&
                 std::abs(p.z - c.center.z) <= c.size.z / 2 + kBoxMarginM;
        });
    if (obstacle) {
      labels[i] = kObstaclePoint;
      continue;
    }
    if (!result.ground_valid || p.x < config.detection_roi.min[0] || p.x > corridor_end) continue;
    const double height = p.z - (result.ground_a * p.x + result.ground_b * p.y + result.ground_c);
    if (std::abs(result.route.offset(p)) <= config.corridor_half_width_m &&
        height >= -kBelowFloorM && height <= config.corridor_height_m)
      labels[i] = kCorridorPoint;
  }
  return labels;
}
}  // namespace metro_perception_core
