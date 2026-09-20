#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
namespace metro_perception_core {
struct Bounds {
  std::array<double, 3> min{-10, -20, -10}, max{150, 20, 15};
  void validate() const {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(min[i]) || !std::isfinite(max[i]) || min[i] >= max[i])
        throw std::invalid_argument("Invalid ROI bounds");
    }
  }
};
struct AlgorithmConfig {
  std::size_t max_points{2000000};
  double blind_radius_m{0.5};  // Sensor coordinates, before translation.
  Bounds geometry_roi;
  Bounds detection_roi{{0, -5, -3}, {120, 5, 5}};
  void validate() const {
    if (max_points == 0 || max_points > 10000000)
      throw std::invalid_argument("max_points must be in [1, 10000000]");
    if (!std::isfinite(blind_radius_m) || blind_radius_m < 0)
      throw std::invalid_argument("Invalid blind_radius_m");
    geometry_roi.validate();
    detection_roi.validate();
    for (int i = 0; i < 3; ++i)
      if (detection_roi.min[i] < geometry_roi.min[i] || detection_roi.max[i] > geometry_roi.max[i])
        throw std::invalid_argument("Detection ROI must be inside geometry ROI");
  }
};
}  // namespace metro_perception_core
