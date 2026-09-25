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
  // Diagnostics only: keep rejected components and their reasons in FrameResult.
  bool record_rejected{false};
  // Diagnostics only: keep the odometry shift errors and tracker state in FrameResult.
  bool record_motion{false};
  double blind_radius_m{0.5};  // Sensor coordinates, before translation.
  Bounds geometry_roi;
  Bounds detection_roi{{0, -5, -3}, {120, 5, 5}};
  // Initial straight-path hypothesis, measured from the lidar origin.
  double corridor_half_width_m{2.0};
  double corridor_height_m{3.5};
  double ground_max_slope{0.15};
  double ground_inlier_tolerance_m{0.15};
  // Longest unobserved floor stretch before the usable range ends. At least the
  // 5 m support bin, so gaps inside one bin never exceed it.
  double ground_max_gap_m{10.0};
  // Floor returns needed for a 5 m stretch to count as observed.
  std::size_t ground_min_bin_points{30};
  double obstacle_min_height_m{0.25};
  // Candidate extent is measured this far past the corridor edge.
  double candidate_margin_m{0.5};
  double angular_cell_deg{0.25};
  std::size_t min_ground_inliers{30};
  std::size_t min_candidate_cells{3};
  std::size_t min_candidate_points{5};
  // Zero disables the rolling baseline for controlled B0 comparisons.
  std::size_t background_history_frames{10};
  std::size_t background_lag_frames{5};
  double background_margin_m{0.5};
  double background_relative_margin{0.02};
  // Route centre from the tunnel walls; the corridor follows curves instead of a line.
  bool route_estimation{true};
  double route_min_radius_m{150.0};
  // Gauge channel: returns inside a narrow route gauge are candidates without any history,
  // so obstacles fixed in the world are found while the train approaches them. Long
  // gauge-only structures (walls, platform edges) are rejected by length.
  bool static_channel{true};
  // Chosen on development data: rails and track-side equipment start around 0.76 m.
  double static_half_width_m{0.9};
  double static_min_height_m{0.3};
  double static_max_height_m{2.5};
  double static_max_length_m{3.0};
  // Returns below low_object_height_m above the bed count as obstacle evidence only between
  // the rails: rails (+-0.76 m), the contact rail (~1.5 m), cable ducts and walkways are low
  // and lie outside low_object_half_width_m. Tall objects count anywhere in the corridor.
  double low_object_height_m{1.0};
  double low_object_half_width_m{0.5};
  // Vehicle half-width with margin (a metro car is ~1.35-1.4 m). While the train moves at
  // envelope_min_speed_mps or more, evidence without the gauge channel that stays farther
  // from the route is not a candidate; standing still, the whole corridor counts.
  double envelope_half_width_m{1.5};
  double envelope_min_speed_mps{1.0};
  // Move the baseline by lidar-only forward odometry before differencing.
  bool ego_motion_compensation{true};
  double ego_max_speed_mps{25.0};
  // Required lead, in metres of mean profile error, of the best shift over the median.
  double ego_min_contrast{0.005};
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
        !std::isfinite(ground_max_gap_m) || ground_max_gap_m < 5 || ground_max_gap_m > 50 ||
        ground_min_bin_points == 0 || !std::isfinite(obstacle_min_height_m) ||
        obstacle_min_height_m <= 0 || obstacle_min_height_m > corridor_height_m ||
        !std::isfinite(angular_cell_deg) || angular_cell_deg < 0.05 || angular_cell_deg > 1.0 ||
        min_ground_inliers < 3 || min_candidate_cells == 0 || min_candidate_points == 0 ||
        background_history_frames > 60 ||
        (background_history_frames &&
         (background_lag_frames == 0 || background_lag_frames >= background_history_frames)) ||
        !std::isfinite(candidate_margin_m) || candidate_margin_m < 0 || candidate_margin_m > 2 ||
        !std::isfinite(ego_max_speed_mps) || ego_max_speed_mps <= 0 || ego_max_speed_mps > 60 ||
        !std::isfinite(ego_min_contrast) || ego_min_contrast <= 0 || ego_min_contrast > 0.3 ||
        !std::isfinite(route_min_radius_m) || route_min_radius_m < 20 ||
        !std::isfinite(static_half_width_m) || static_half_width_m <= 0 ||
        !std::isfinite(static_min_height_m) || static_min_height_m <= 0 ||
        !std::isfinite(static_max_height_m) || static_max_height_m <= static_min_height_m ||
        !std::isfinite(static_max_length_m) || static_max_length_m <= 0 ||
        !std::isfinite(low_object_height_m) || low_object_height_m < 0 ||
        !std::isfinite(low_object_half_width_m) || low_object_half_width_m < 0 ||
        !std::isfinite(envelope_half_width_m) || envelope_half_width_m < 0 ||
        !std::isfinite(envelope_min_speed_mps) || envelope_min_speed_mps < 0 ||
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
// Confirmation over frames (G4). The defaults reproduce the per-frame decision exactly.
struct TemporalConfig {
  // Hits a track needs within its last `window` frames to be confirmed.
  std::size_t confirm_hits{1}, confirm_window{1};
  // Stricter rule for tracks seen only by the gauge channel, which has no history evidence.
  std::size_t gauge_confirm_hits{1}, gauge_confirm_window{1};
  // A confirmed track survives this many missed frames minus one, coasting on ego motion.
  std::size_t release_misses{1};
  // Association gate: base + fraction of range + object and unknown-ego motion over dt.
  double gate_base_m{1.0};
  double gate_range_fraction{0.03};
  double object_max_speed_mps{4.0};
  double unknown_ego_speed_mps{20.0};
  // Below this valid ego speed a track with MOTION evidence is confirmed on its first hit
  // (standing still, differencing needs no odometry); 0 disables.
  double still_speed_mps{0.0};
  // Returns inside the corridor such a track needs in that frame (not a sparse flicker).
  std::uint32_t still_min_points{50};
  // A longer pause between analysed frames restarts all tracks.
  double max_gap_s{0.5};
  std::size_t max_tracks{64};
  void validate() const {
    const auto rule_ok = [](std::size_t hits, std::size_t window) {
      return hits >= 1 && window >= hits && window <= 32;
    };
    if (!rule_ok(confirm_hits, confirm_window) ||
        !rule_ok(gauge_confirm_hits, gauge_confirm_window) || release_misses < 1 ||
        release_misses > 32 || !std::isfinite(gate_base_m) || gate_base_m <= 0 ||
        !std::isfinite(gate_range_fraction) || gate_range_fraction < 0 ||
        !std::isfinite(object_max_speed_mps) || object_max_speed_mps < 0 ||
        !std::isfinite(unknown_ego_speed_mps) || unknown_ego_speed_mps < 0 ||
        !std::isfinite(still_speed_mps) || still_speed_mps < 0 || !std::isfinite(max_gap_s) ||
        max_gap_s <= 0 || max_tracks == 0 || max_tracks > 1024)
      throw std::invalid_argument("Invalid temporal configuration");
  }
};
}  // namespace metro_perception_core
