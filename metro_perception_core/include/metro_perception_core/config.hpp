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
  // Initial straight-path hypothesis, measured from the lidar origin.
  double corridor_half_width_m{2.0};
  double corridor_height_m{3.5};
  double ground_max_slope{0.15};
  double ground_inlier_tolerance_m{0.15};
  double obstacle_min_height_m{0.25};
  double angular_cell_deg{0.25};
  std::size_t min_ground_inliers{30};
  std::size_t min_candidate_cells{3};
  std::size_t min_candidate_points{5};
  // Zero disables the rolling baseline for controlled B0 comparisons.
  std::size_t background_history_frames{10};
  std::size_t background_lag_frames{5};
  double background_margin_m{0.5};
  double background_relative_margin{0.02};
  void validate() const {
    if (max_points == 0 || max_points > 10000000)
      throw std::invalid_argument("max_points must be in [1, 10000000]");
    if (!std::isfinite(blind_radius_m) || blind_radius_m < 0)
      throw std::invalid_argument("Invalid blind_radius_m");
    if (!std::isfinite(corridor_half_width_m) || corridor_half_width_m <= 0 ||
        corridor_half_width_m > 5 || !std::isfinite(corridor_height_m) || corridor_height_m <= 0 ||
        corridor_height_m > 8 || !std::isfinite(ground_max_slope) || ground_max_slope <= 0 ||
        ground_max_slope > 0.5 || !std::isfinite(ground_inlier_tolerance_m) ||
        ground_inlier_tolerance_m <= 0 || ground_inlier_tolerance_m > 0.5 ||
        !std::isfinite(obstacle_min_height_m) || obstacle_min_height_m <= 0 ||
        obstacle_min_height_m > corridor_height_m || !std::isfinite(angular_cell_deg) ||
        angular_cell_deg < 0.05 || angular_cell_deg > 1.0 || min_ground_inliers < 3 ||
        min_candidate_cells == 0 || min_candidate_points == 0 || background_history_frames > 60 ||
        (background_history_frames &&
         (background_lag_frames == 0 || background_lag_frames >= background_history_frames)) ||
        !std::isfinite(background_margin_m) || background_margin_m < 0 ||
        !std::isfinite(background_relative_margin) || background_relative_margin < 0 ||
        background_relative_margin > 0.5)
      throw std::invalid_argument("Invalid detector configuration");
    geometry_roi.validate();
    detection_roi.validate();
    for (int i = 0; i < 3; ++i)
      if (detection_roi.min[i] < geometry_roi.min[i] || detection_roi.max[i] > geometry_roi.max[i])
        throw std::invalid_argument("Detection ROI must be inside geometry ROI");
  }
};
}  // namespace metro_perception_core
