#include "metro_perception_core/detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace metro_perception_core {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGroundMinX = 2, kGroundMaxX = 90, kGroundBinM = 5;
// Brief ground dropouts keep the range history; longer ones restart it.
constexpr std::size_t kMaxGeometryFailures = 5;
struct Plane {
  double a{0}, b{0}, c{0};  // z = a*x + b*y + c
  std::size_t inliers{0};
  double max_supported_x{0};
  bool valid{false};
  double height(const PointXYZ& p) const { return p.z - (a * p.x + b * p.y + c); }
};
struct Cell {
  int az{0}, el{0};
  double range{std::numeric_limits<double>::infinity()};
  std::vector<std::size_t> points;  // Indices into geometry_points.
};

bool in_ground_region(const PointXYZ& p) {
  return p.x >= kGroundMinX && p.x <= kGroundMaxX && std::abs(p.y) <= 3 && p.z >= -3 && p.z <= 0.5;
}
bool plausible_ground(const AlgorithmConfig& config, double a, double b, double c) {
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) &&
         std::hypot(a, b) <= config.ground_max_slope && c >= -3 && c <= 0.5;
}

// Least-squares z = a*x + b*y + c over the sample plane's inliers. The RANSAC
// hypothesis is kept when the refit is degenerate or leaves the plausible range.
void refine_ground(const AlgorithmConfig& config, const std::vector<const PointXYZ*>& seeds,
                   Plane& plane) {
  double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, n = 0, sxz = 0, syz = 0, sz = 0;
  for (const auto* p : seeds) {
    if (std::abs(plane.height(*p)) > config.ground_inlier_tolerance_m) continue;
    sxx += p->x * p->x, sxy += p->x * p->y, sx += p->x, syy += p->y * p->y, sy += p->y;
    sxz += p->x * p->z, syz += p->y * p->z, sz += p->z, n += 1;
  }
  const auto det3 = [](double a1, double b1, double c1, double a2, double b2, double c2, double a3,
                       double b3, double c3) {
    return a1 * (b2 * c3 - c2 * b3) - b1 * (a2 * c3 - c2 * a3) + c1 * (a2 * b3 - b2 * a3);
  };
  const double det = det3(sxx, sxy, sx, sxy, syy, sy, sx, sy, n);
  if (std::abs(det) < 1e-9) return;
  const double a = det3(sxz, sxy, sx, syz, syy, sy, sz, sy, n) / det;
  const double b = det3(sxx, sxz, sx, sxy, syz, sy, sx, sz, n) / det;
  const double c = det3(sxx, sxy, sxz, sxy, syy, syz, sx, sy, sz) / det;
  if (plausible_ground(config, a, b, c)) plane.a = a, plane.b = b, plane.c = c;
}

// The usable range ends at the first floor gap longer than ground_max_gap_m,
// counted from the sensor to the nearest floor return of the next observed bin.
// Gaps inside a bin are at most kGroundBinM, which the config keeps <= the limit.
// Isolated far returns do not extend it.
void measure_support(const AlgorithmConfig& config, const std::vector<IndexedPoint>& points,
                     Plane& plane) {
  const auto bins = static_cast<std::size_t>(std::ceil((kGroundMaxX - kGroundMinX) / kGroundBinM));
  std::vector<std::size_t> count(bins, 0);
  std::vector<double> near(bins, kGroundMaxX), far(bins, 0);
  for (const auto& indexed : points) {
    const auto& p = indexed.point;
    if (!in_ground_region(p) || std::abs(plane.height(p)) > config.ground_inlier_tolerance_m)
      continue;
    const auto bin =
        std::min(bins - 1, static_cast<std::size_t>((p.x - kGroundMinX) / kGroundBinM));
    ++count[bin];
    near[bin] = std::min(near[bin], p.x);
    far[bin] = std::max(far[bin], p.x);
  }
  plane.inliers = 0;
  plane.max_supported_x = kGroundMinX;
  for (std::size_t bin = 0; bin < bins; ++bin) {
    if (count[bin] < config.ground_min_bin_points) continue;
    if (near[bin] - plane.max_supported_x > config.ground_max_gap_m) break;
    plane.inliers += count[bin];
    plane.max_supported_x = far[bin];
  }
}

Plane estimate_ground(const AlgorithmConfig& config, const PreprocessedFrame& frame) {
  std::vector<const PointXYZ*> seeds;
  const auto& points = frame.geometry_points;
  const std::size_t stride = std::max<std::size_t>(1, points.size() / 6000);
  for (std::size_t i = 0; i < points.size(); i += stride) {
    const auto& p = points[i].point;
    if (in_ground_region(p)) seeds.push_back(&p);
  }
  if (seeds.size() < config.min_ground_inliers) return {};
  Plane best;
  std::uint32_t random = 0x9e3779b9u;
  auto pick = [&] {
    random = random * 1664525u + 1013904223u;
    return std::size_t(random) % seeds.size();
  };
  for (int attempt = 0; attempt < 256; ++attempt) {
    const auto& p = *seeds[pick()];
    const auto& q = *seeds[pick()];
    const auto& r = *seeds[pick()];
    const double dx1 = q.x - p.x, dy1 = q.y - p.y;
    const double dx2 = r.x - p.x, dy2 = r.y - p.y;
    const double det = dx1 * dy2 - dx2 * dy1;
    if (std::abs(det) < 0.5) continue;
    const double dz1 = q.z - p.z, dz2 = r.z - p.z;
    const double a = (dz1 * dy2 - dz2 * dy1) / det;
    const double b = (dx1 * dz2 - dx2 * dz1) / det;
    const double c = p.z - a * p.x - b * p.y;
    // The slope limit rejects walls; the offset limit keeps the plane below the sensor.
    if (!plausible_ground(config, a, b, c)) continue;
    std::size_t inliers = 0;
    std::uint8_t along = 0, across = 0;
    for (const auto* point : seeds) {
      if (std::abs(point->z - a * point->x - b * point->y - c) > config.ground_inlier_tolerance_m)
        continue;
      ++inliers;
      if (point->x < 25) along |= 1;
      if (point->x >= 25) along |= 2;
      if (point->y < -0.25) across |= 1;
      if (point->y > 0.25) across |= 2;
    }
    // A small patch, rail or platform edge cannot establish the route surface.
    if (inliers < config.min_ground_inliers || along != 3 || across != 3) continue;
    if (!best.valid || inliers > best.inliers || (inliers == best.inliers && c < best.c)) {
      best = {a, b, c, inliers, 0, true};
    }
  }
  if (!best.valid) return best;
  refine_ground(config, seeds, best);
  measure_support(config, points, best);
  return best;
}

std::int64_t key(int az, int el) { return std::int64_t(az) * 2048 + el; }

struct DisjointSet {
  std::vector<std::size_t> parent;
  explicit DisjointSet(std::size_t count) : parent(count) {
    std::iota(parent.begin(), parent.end(), 0);
  }
  std::size_t root(std::size_t i) {
    while (parent[i] != i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  }
  void join(std::size_t a, std::size_t b) { parent[root(a)] = root(b); }
};

// Forward displacement between frames from the tunnel's lateral profile: in
// 5 cm steps along X, per side and height band, how far the nearest structure
// protrudes from the frame's mean cross-section (median lateral distance per
// 10 cm of height). Comparing departures rather than raw distances keeps a
// curved wall, sampled at sensor-fixed heights, from favouring zero shift. A
// uniform tunnel yields no clear shift and falls back to the last speed.
// Periodic lining produces aliases, so the speed is tracked near its last value
// and changes to a distant shift only after consistent frames.
constexpr double kProfileMinX = 3, kProfileMaxX = 35, kProfileStepM = 0.05;
constexpr std::size_t kProfileBins = 640, kProfileRows = 6;
constexpr std::size_t kProfileMinPairs = 200;
constexpr double kProfileMaxError = 0.3;
constexpr double kMaxAccelerationMps2 = 3.0;
// An unconfirmed speed drives the odometry for kMotionHoldS and centres the
// reacquisition window until kMotionForgetS.
constexpr double kMotionHoldS = 5.0, kMotionForgetS = 10.0;
constexpr std::size_t kMotionConfirmFrames = 3;
constexpr double kSpeedGain = 0.5;
}  // namespace

void GeometricDetector::estimate_motion(const AlgorithmConfig& config,
                                        const PreprocessedFrame& frame,
                                        const std::array<double, 3>& ground,
                                        std::int64_t measurement_time_ns, FrameResult& result) {
  struct Sample {
    std::size_t section, bin, row;
    float lateral;
  };
  constexpr std::size_t kSectionBins = 27;  // 10 cm of height from 0.3 m, per side.
  std::vector<Sample> samples;
  std::array<std::vector<float>, 2 * kSectionBins> section;
  for (const auto& indexed : frame.geometry_points) {
    const auto& p = indexed.point;
    const double lateral = std::abs(p.y);
    const double height = p.z - (ground[0] * p.x + ground[1] * p.y + ground[2]);
    if (p.x < kProfileMinX || p.x >= kProfileMaxX || lateral < 1 || lateral > 4 || height < 0.3 ||
        height >= 3)
      continue;
    const std::size_t side = p.y > 0 ? 0 : 1;
    const std::size_t level =
        std::min(kSectionBins - 1, static_cast<std::size_t>((height - 0.3) / 0.1));
    const std::size_t row = side * 3 + (height < 1 ? 0 : height < 2 ? 1 : 2);
    const auto bin =
        std::min(kProfileBins - 1, static_cast<std::size_t>((p.x - kProfileMinX) / kProfileStepM));
    samples.push_back({side * kSectionBins + level, bin, row, static_cast<float>(lateral)});
    section[samples.back().section].push_back(samples.back().lateral);
  }
  std::array<float, 2 * kSectionBins> mean_section{};
  for (std::size_t i = 0; i < section.size(); ++i) {
    auto& values = section[i];
    if (values.empty()) continue;
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    mean_section[i] = values[values.size() / 2];
  }
  std::vector<float> profile(kProfileRows * kProfileBins, INFINITY);
  for (const auto& sample : samples) {
    auto& value = profile[sample.row * kProfileBins + sample.bin];
    value = std::min(value, sample.lateral - mean_section[sample.section]);
  }
  const double dt = motion_stamp_ns_ && measurement_time_ns > motion_stamp_ns_
                        ? (measurement_time_ns - motion_stamp_ns_) * 1e-9
                        : 0.0;
  bool valid = false;
  double displacement = 0;
  const auto max_shift = std::min<std::size_t>(
      kProfileBins / 2,
      static_cast<std::size_t>(std::ceil(config.ego_max_speed_mps * dt / kProfileStepM)));
  if (dt > 0 && max_shift >= 4 && !motion_profile_.empty()) {
    // Mean profile difference for each forward shift; a static structure at x in
    // the previous frame is now at x - shift.
    std::vector<double> errors;
    for (std::size_t shift = 0; shift <= max_shift; ++shift) {
      double sum = 0;
      std::size_t pairs = 0;
      for (std::size_t row = 0; row < kProfileRows; ++row)
        for (std::size_t bin = 0; bin + shift < kProfileBins; ++bin) {
          const float now = profile[row * kProfileBins + bin];
          const float then = motion_profile_[row * kProfileBins + bin + shift];
          if (!std::isfinite(now) || !std::isfinite(then)) continue;
          sum += std::min<double>(std::abs(now - then), kProfileMaxError);
          ++pairs;
        }
      errors.push_back(pairs >= kProfileMinPairs ? sum / double(pairs) : INFINITY);
    }
    std::vector<double> finite;
    for (const double error : errors)
      if (std::isfinite(error)) finite.push_back(error);
    double median = INFINITY;
    if (finite.size() >= 4) {
      std::nth_element(finite.begin(), finite.begin() + finite.size() / 2, finite.end());
      median = finite[finite.size() / 2];
    }
    // Best clear shift within [predicted - window, predicted + window] bins.
    auto search = [&](double predicted, double window) -> std::optional<std::size_t> {
      const auto low = static_cast<std::size_t>(std::max(0.0, std::floor(predicted - window)));
      const auto high =
          std::min(max_shift, static_cast<std::size_t>(std::ceil(predicted + window)));
      if (low > high) return std::nullopt;
      std::size_t best = low;
      for (std::size_t shift = low; shift <= high; ++shift)
        if (errors[shift] < errors[best]) best = shift;
      // Without enough comparable shifts there is no median to stand out from.
      if (!std::isfinite(median) || !std::isfinite(errors[best]) ||
          median - errors[best] < config.ego_min_contrast)
        return std::nullopt;
      return best;
    };
    // A known speed is tracked within two 5 cm steps per frame. Periodic lining
    // makes aliases, so leaving the track needs persistent evidence.
    std::optional<std::size_t> tracked;
    const double predicted = speed_mps_ * dt / kProfileStepM;
    if (speed_known_) tracked = search(std::round(predicted), 2);
    // A different shift is adopted only as the best overall, clearly better
    // than the tracked one and consistent for kMotionConfirmFrames frames.
    const auto global =
        static_cast<std::size_t>(std::min_element(errors.begin(), errors.end()) - errors.begin());
    std::optional<std::size_t> candidate;
    bool adopted = false;
    if (!tracked) {
      candidate = speed_known_
                      ? search(predicted, 1 + std::ceil(kMaxAccelerationMps2 *
                                                        (unconfirmed_s_ + dt) * dt / kProfileStepM))
                      : search(0, double(max_shift));
      if (candidate && (*candidate > global + 1 || *candidate + 1 < global)) candidate.reset();
    } else if ((global > *tracked + 1 || global + 1 < *tracked) &&
               errors[*tracked] - errors[global] > config.ego_min_contrast) {
      candidate = search(double(global), 0);
    }
    if (candidate) {
      const double speed = *candidate * kProfileStepM / dt;
      pending_frames_ = std::abs(speed - pending_speed_mps_) <= 1.0 ? pending_frames_ + 1 : 1;
      pending_speed_mps_ = speed;
      if (pending_frames_ >= kMotionConfirmFrames) {
        tracked = candidate;
        adopted = true;
        pending_frames_ = 0;
      }
    } else {
      pending_frames_ = 0;
    }
    if (tracked) {
      // Shifts are quantised to 5 cm; a tracked speed follows them gradually.
      const double measured = *tracked * kProfileStepM / dt;
      speed_mps_ = !adopted && unconfirmed_s_ == 0 && speed_known_
                       ? speed_mps_ + kSpeedGain * (measured - speed_mps_)
                       : measured;
      displacement = speed_mps_ * dt;
      unconfirmed_s_ = 0;
      speed_known_ = valid = true;
    } else if (speed_known_) {
      unconfirmed_s_ += dt;
      if (unconfirmed_s_ <= kMotionHoldS) {
        displacement = speed_mps_ * dt;
        valid = true;
      } else if (unconfirmed_s_ > kMotionForgetS) {
        speed_known_ = false;
      }
    }
  }
  if (valid) {
    odometry_m_ += displacement;
  } else {
    // History recorded before a break cannot be moved into this frame.
    ++motion_epoch_;
  }
  motion_profile_ = std::move(profile);
  if (measurement_time_ns) motion_stamp_ns_ = measurement_time_ns;
  result.ego_motion_valid = valid;
  result.ego_speed_mps = valid && dt > 0 ? displacement / dt : 0;
}

void GeometricDetector::process(const AlgorithmConfig& config, FrameResult& result,
                                std::int64_t measurement_time_ns) {
  if (measurement_time_ns && last_stamp_ns_ &&
      (measurement_time_ns <= last_stamp_ns_ ||
       measurement_time_ns - last_stamp_ns_ > 2000000000LL))
    reset();
  if (measurement_time_ns) last_stamp_ns_ = measurement_time_ns;
  const auto& frame = result.preprocessed;
  const auto ground = estimate_ground(config, frame);
  const double end_x = std::min(config.detection_roi.max[0], ground.max_supported_x + 3.0);
  if (!ground.valid || end_x < 25.0 || frame.detection_indices.empty()) {
    // Skipped frames do not enter the history; it stays usable after a short dropout.
    if (++geometry_failures_ > kMaxGeometryFailures) reset();
    result.status = AnalysisStatus::INVALID_GEOMETRY;
    result.reason = ground.valid ? "CORRIDOR_UNOBSERVABLE" : "GROUND_UNSUPPORTED";
    return;
  }
  geometry_failures_ = 0;
  if (config.background_history_frames && config.ego_motion_compensation)
    estimate_motion(config, frame, {ground.a, ground.b, ground.c}, measurement_time_ns, result);
  CorridorSegment segment;
  segment.start = {config.detection_roi.min[0], 0, ground.c};
  segment.end = {end_x, 0, ground.a * end_x + ground.c};
  segment.width_m = 2.0 * config.corridor_half_width_m;
  segment.height_m = config.corridor_height_m;
  const double norm = std::sqrt(1 + ground.a * ground.a + ground.b * ground.b);
  segment.ground_plane = {-ground.a / norm, -ground.b / norm, 1 / norm, -ground.c / norm};
  segment.ground_inliers = static_cast<std::uint32_t>(ground.inliers);
  segment.geometry_valid = true;
  segment.coverage_valid = true;
  result.corridor.push_back(segment);
  result.evaluated_range_m = end_x;
  result.evaluation_region_valid = true;

  // From analyze_bag.py: map each return to an angular cell, then connect adjacent
  // occupied cells at compatible ranges. Elevation is measured directly here because
  // the current PointCloud2 adapter does not expose the optional ring field.
  const int n_az = static_cast<int>(std::ceil(360.0 / config.angular_cell_deg));
  const int n_el = static_cast<int>(std::ceil(180.0 / config.angular_cell_deg));
  const auto& origin = frame.sensor_origin;
  auto angular_cell = [&](const PointXYZ& p) {
    const double sx = p.x - origin.x, sy = p.y - origin.y, sz = p.z - origin.z;
    const int az = std::clamp(static_cast<int>(std::floor((std::atan2(sy, sx) * 180 / kPi + 180) /
                                                          config.angular_cell_deg)),
                              0, n_az - 1);
    const int el = std::clamp(
        static_cast<int>(std::floor((std::atan2(sz, std::hypot(sx, sy)) * 180 / kPi + 90) /
                                    config.angular_cell_deg)),
        0, n_el - 1);
    return std::pair<int, int>{az, el};
  };
  auto sensor_range = [&](const PointXYZ& p) {
    return std::hypot(std::hypot(p.x - origin.x, p.y - origin.y), p.z - origin.z);
  };
  // Candidates may extend this far past the corridor edge; only returns inside
  // the corridor decide acceptance and distance.
  const double band_half_width = config.corridor_half_width_m + config.candidate_margin_m;
  std::unordered_map<std::int64_t, PointXYZ> current;  // Nearest return per cell.
  if (config.background_history_frames) {
    // The full range image includes floor and tunnel returns. Foreground is a
    // nearer surface in the same angular cell, as in analyze_bag.py. The history
    // keeps returns beyond the usable range so it still covers it after moving.
    for (const auto index : frame.detection_indices) {
      const auto& p = frame.geometry_points[index].point;
      const double height = ground.height(p);
      if (p.x < config.detection_roi.min[0] || std::abs(p.y) > band_half_width || height < -0.5 ||
          height > config.corridor_height_m)
        continue;
      const auto [az, el] = angular_cell(p);
      const auto found = current.try_emplace(key(az, el), p);
      if (!found.second && sensor_range(p) < sensor_range(found.first->second))
        found.first->second = p;
    }
    if (history_.size() < config.background_history_frames) {
      history_.push_back(make_history(current, origin));
      result.status = AnalysisStatus::INVALID_GEOMETRY;
      result.reason = "BASELINE_WARMUP";
      result.evaluation_region_valid = false;
      result.evaluated_range_m = 0;
      result.corridor.front().coverage_valid = false;
      return;
    }
  }
  // Each current return is moved back into the baseline frames by the odometry
  // since they were recorded and checked against what their own rays saw there.
  // Without valid odometry the shift is zero and this is plain differencing.
  std::vector<double> shifts;
  bool moved = false;
  if (config.background_history_frames) {
    const auto end = history_.size() - config.background_lag_frames;
    for (std::size_t i = 0; i < end; ++i) {
      shifts.push_back(config.ego_motion_compensation && history_[i].epoch == motion_epoch_
                           ? odometry_m_ - history_[i].odometry_m
                           : 0.0);
      moved = moved || shifts.back() > 0;
    }
  }
  std::unordered_map<std::int64_t, bool> foreground_cache;
  auto foreground = [&](std::int64_t cell_key) {
    if (!config.background_history_frames) return true;
    const auto known = foreground_cache.find(cell_key);
    if (known != foreground_cache.end()) return known->second;
    const auto current_cell = current.find(cell_key);
    if (current_cell == current.end()) return false;
    // Positive: the baseline ray passed beyond this return (free space then).
    // Near zero: it hit the same surface. Negative: it was occluded. The least
    // clearance over neighbouring cells absorbs grazing surfaces such as walls
    // parallel to the track, whose range changes steeply within one cell.
    std::vector<double> clearances;
    double reference = 0;
    for (std::size_t i = 0; i < shifts.size(); ++i) {
      const auto& p = current_cell->second;
      const PointXYZ past{p.x + shifts[i], p.y, p.z};
      if (past.x > config.detection_roi.max[0]) continue;  // Never inside that baseline.
      const auto [az, el] = angular_cell(past);
      const double range = sensor_range(past);
      double clearance = INFINITY, seen = 0;
      for (int da = -1; da <= 1; ++da)
        for (int de = -1; de <= 1; ++de) {
          const auto found = history_[i].ranges.find(key(az + da, el + de));
          if (found == history_[i].ranges.end() || found->second - range >= clearance) continue;
          clearance = found->second - range;
          seen = found->second;
        }
      if (!std::isfinite(clearance)) continue;
      clearances.push_back(clearance);
      reference += seen;
    }
    // Newly occupied cells follow the script's "appeared" path when the rays are
    // unchanged. After a shift the same direction may fall between scan lines,
    // so missing returns are no evidence. A sporadic single baseline return is
    // insufficient evidence either way.
    bool is_foreground = clearances.empty() && !moved;
    if (clearances.size() >= 2) {
      std::sort(clearances.begin(), clearances.end());
      reference /= double(clearances.size());
      is_foreground = clearances[clearances.size() / 2] >
                      config.background_margin_m + config.background_relative_margin * reference;
    }
    foreground_cache.emplace(cell_key, is_foreground);
    return is_foreground;
  };
  std::unordered_map<std::int64_t, std::size_t> lookup;
  std::vector<Cell> cells;
  for (const auto index : frame.detection_indices) {
    const auto& p = frame.geometry_points[index].point;
    const double height = ground.height(p);
    if (p.x > end_x || p.x < config.detection_roi.min[0] || std::abs(p.y) > band_half_width ||
        height < config.obstacle_min_height_m || height > config.corridor_height_m)
      continue;
    const double range = sensor_range(p);
    if (!(range > 0)) continue;
    const auto [az, el] = angular_cell(p);
    const auto cell_key = key(az, el);
    if (!foreground(cell_key)) continue;
    const auto found = lookup.find(cell_key);
    if (found == lookup.end()) {
      lookup.emplace(cell_key, cells.size());
      cells.push_back({az, el, range, {index}});
    } else {
      auto& cell = cells[found->second];
      cell.range = std::min(cell.range, range);
      cell.points.push_back(index);
    }
  }
  DisjointSet groups(cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i) {
    // One missing angular cell may separate adjacent returns, as in
    // analyze_bag.py's one-cell dilation before connected components.
    for (int da = -2; da <= 2; ++da) {
      for (int de = -2; de <= 2; ++de) {
        if (da == 0 && de == 0) continue;
        const auto near = lookup.find(key(cells[i].az + da, cells[i].el + de));
        if (near == lookup.end()) continue;
        const auto& other = cells[near->second];
        const double depth_tolerance = 0.5 + 0.02 * std::min(cells[i].range, other.range);
        if (std::abs(cells[i].range - other.range) <= depth_tolerance) groups.join(i, near->second);
      }
    }
  }
  std::unordered_map<std::size_t, std::vector<std::size_t>> components;
  for (std::size_t i = 0; i < cells.size(); ++i) components[groups.root(i)].push_back(i);
  for (const auto& entry : components) {
    PointXYZ lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    PointXYZ nearest;
    double distance = INFINITY, inside_low = INFINITY, inside_high = -INFINITY;
    std::size_t support = 0, inside_cells = 0;
    for (const auto cell_index : entry.second) {
      const auto& cell = cells[cell_index];
      bool cell_inside = false;
      for (const auto point_index : cell.points) {
        const auto& p = frame.geometry_points[point_index].point;
        if (sensor_range(p) > cell.range + 0.5) continue;
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
        hi.z = std::max(hi.z, p.z);
        if (std::abs(p.y) > config.corridor_half_width_m) continue;
        cell_inside = true;
        inside_low = std::min(inside_low, p.z);
        inside_high = std::max(inside_high, p.z);
        if (p.x - origin.x < distance) {
          distance = p.x - origin.x;
          nearest = p;
        }
        ++support;
      }
      inside_cells += cell_inside;
    }
    // The bbox spans the margin band; acceptance uses only returns inside.
    if (inside_cells < config.min_candidate_cells || support < config.min_candidate_points ||
        inside_high - inside_low < config.obstacle_min_height_m)
      continue;
    ObstacleCandidate candidate;
    candidate.center = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, (lo.z + hi.z) / 2};
    candidate.size = {hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
    candidate.nearest_point = nearest;
    candidate.distance_m = distance;
    candidate.distance_valid = std::isfinite(distance) && distance >= 0;
    candidate.support_points = static_cast<std::uint32_t>(support);
    if (candidate.distance_valid) result.candidates.push_back(candidate);
  }
  std::sort(result.candidates.begin(), result.candidates.end(),
            [](const auto& a, const auto& b) { return a.distance_m < b.distance_m; });
  for (std::size_t i = 0; i < result.candidates.size(); ++i) result.candidates[i].id = i + 1;
  result.status = AnalysisStatus::OK;
  result.reason = result.candidates.empty()
                      ? (config.background_history_frames ? "BACKGROUND_CANNOT_CONFIRM_CLEAR"
                                                          : "NO_CANDIDATE_IN_EVALUATED_REGION")
                      : "CANDIDATES_FOUND";
  if (config.background_history_frames) {
    // A stationary object already present in history can disappear from the
    // foreground. Differencing alone cannot certify a clear path.
    result.evaluation_region_valid = false;
    result.corridor.front().coverage_valid = false;
    history_.pop_front();
    history_.push_back(make_history(current, origin));
  }
}

GeometricDetector::HistoryFrame GeometricDetector::make_history(
    const std::unordered_map<std::int64_t, PointXYZ>& nearest, const PointXYZ& origin) const {
  HistoryFrame frame{{}, odometry_m_, motion_epoch_};
  frame.ranges.reserve(nearest.size());
  for (const auto& [cell, p] : nearest)
    frame.ranges.emplace(cell,
                         std::hypot(std::hypot(p.x - origin.x, p.y - origin.y), p.z - origin.z));
  return frame;
}
}  // namespace metro_perception_core
