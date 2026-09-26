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
  std::uint8_t channels{0};         // ObstacleCandidate::kMotion / kGauge.
};

// The route floor is searched in the same lateral band as candidates. Wider, a platform,
// a parked train or a bench beside the track can outvote the floor and lift the plane,
// which cuts the lower body of an obstacle off below "ground".
bool in_ground_region(const AlgorithmConfig& config, const PointXYZ& p) {
  return p.x >= kGroundMinX && p.x <= kGroundMaxX &&
         std::abs(p.y) <= config.corridor_half_width_m + config.candidate_margin_m && p.z >= -3 &&
         p.z <= 0.5;
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
    if (!in_ground_region(config, p) ||
        std::abs(plane.height(p)) > config.ground_inlier_tolerance_m)
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
  // The route floor is the lowest surface. Seeds are the lowest return of each 1 m x 0.2 m
  // cell, so walls, walkways and platforms, which a forward-looking lidar sees far more
  // densely than the grazing floor, cannot outvote it by point count.
  const double half_width = config.corridor_half_width_m + config.candidate_margin_m;
  const auto columns = static_cast<std::size_t>(std::ceil(2 * half_width / 0.2)) + 1;
  const auto rows = static_cast<std::size_t>(std::ceil(kGroundMaxX - kGroundMinX)) + 1;
  std::vector<const PointXYZ*> lowest(rows * columns, nullptr);
  for (const auto& indexed : frame.geometry_points) {
    const auto& p = indexed.point;
    if (!in_ground_region(config, p)) continue;
    const auto row = static_cast<std::size_t>(p.x - kGroundMinX);
    const auto column = static_cast<std::size_t>((p.y + half_width) / 0.2);
    auto& slot = lowest[std::min(row, rows - 1) * columns + std::min(column, columns - 1)];
    if (!slot || p.z < slot->z) slot = &p;
  }
  std::vector<const PointXYZ*> seeds;
  for (const auto* point : lowest)
    if (point) seeds.push_back(point);
  if (seeds.size() < config.min_ground_inliers) return {};
  std::vector<Plane> hypotheses;
  std::size_t most = 0;
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
    hypotheses.push_back({a, b, c, inliers, 0, true});
    most = std::max(most, inliers);
  }
  if (hypotheses.empty()) return {};
  // The best-supported plane is the floor unless a clearly distinct, still well-supported
  // surface lies below it: a platform or walkway can outnumber the track bed. Planes within
  // the inlier tolerance of each other are the same surface.
  const auto height = [](const Plane& plane) { return plane.a * 10 + plane.c; };
  Plane best;
  for (const auto& plane : hypotheses)
    if (plane.inliers == most && (!best.valid || height(plane) < height(best))) best = plane;
  const double separation = 2 * config.ground_inlier_tolerance_m;
  Plane lower = best;
  for (const auto& plane : hypotheses)
    if (plane.inliers * 10 >= most * 6 && height(plane) < height(best) - separation &&
        (lower.inliers == best.inliers || plane.inliers > lower.inliers ||
         (plane.inliers == lower.inliers && height(plane) < height(lower))))
      lower = plane;
  best = lower;
  refine_ground(config, seeds, best);
  measure_support(config, frame.geometry_points, best);
  return best;
}

std::int64_t key(int az, int el) { return std::int64_t(az) * 2048 + el; }

// Route centre from the tunnel walls. Per 1 m slice ahead, the nearest structure on each
// side of the predicted centre (at least kWallMinM away, 0.5-2.5 m above the floor) is a
// wall sample; its shift from the near-range wall offset is the centre shift. Slices are
// followed outwards and gated around the running fit, so niches, columns and obstacles
// (short along the route) are outliers. A least-squares y = c1 x + c2 x^2 through the
// accepted slices, refitted without the worst residuals, is the route.
RouteEstimate estimate_route(const AlgorithmConfig& config, const PreprocessedFrame& frame,
                             const Plane& ground) {
  RouteEstimate route;
  if (!config.route_estimation) return route;
  constexpr double kMinX = 3, kNearMaxX = 8, kBinM = 1, kWallMinM = 1.0, kWallMaxM = 6.0;
  constexpr std::size_t kMinSlices = 10;
  const double max_x = std::min(config.detection_roi.max[0], 150.0);
  if (max_x <= kNearMaxX) return route;
  const auto bins = static_cast<std::size_t>(std::ceil((max_x - kMinX) / kBinM));
  std::vector<std::vector<float>> lateral(bins);
  for (const auto& indexed : frame.geometry_points) {
    const auto& p = indexed.point;
    if (p.x < kMinX || p.x >= max_x || std::abs(p.y) > 12) continue;
    const double height = ground.height(p);
    if (height < 0.5 || height > 2.5) continue;
    lateral[static_cast<std::size_t>((p.x - kMinX) / kBinM)].push_back(static_cast<float>(p.y));
  }
  auto nearest_walls = [&](std::size_t bin, double centre) {
    double left = INFINITY, right = -INFINITY;
    for (const float y : lateral[bin]) {
      if (y >= centre + kWallMinM && y <= centre + kWallMaxM) left = std::min<double>(left, y);
      if (y <= centre - kWallMinM && y >= centre - kWallMaxM) right = std::max<double>(right, y);
    }
    return std::pair<double, double>{left, right};
  };
  auto median = [](std::vector<double> values) {
    if (values.size() < 3) return double(NAN);
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return values[values.size() / 2];
  };
  std::vector<double> near_left, near_right;
  for (std::size_t bin = 0; kMinX + (bin + 1) * kBinM <= kNearMaxX; ++bin) {
    const auto [left, right] = nearest_walls(bin, 0.0);
    if (std::isfinite(left)) near_left.push_back(left);
    if (std::isfinite(right)) near_right.push_back(right);
  }
  const double left0 = median(near_left), right0 = median(near_right);
  if (!std::isfinite(left0) && !std::isfinite(right0)) return route;

  const double c2_limit = 1.0 / (2.0 * config.route_min_radius_m);
  struct Slice {
    double x, shift;
  };
  std::vector<Slice> slices;
  double c1 = 0, c2 = 0;
  // Least squares without intercept: the route passes the lidar (profile assumption).
  auto fit = [&](const std::vector<Slice>& use, double& a, double& b) {
    double s2 = 0, s3 = 0, s4 = 0, sy1 = 0, sy2 = 0;
    for (const auto& s : use) {
      const double x2 = s.x * s.x;
      s2 += x2, s3 += x2 * s.x, s4 += x2 * x2, sy1 += s.x * s.shift, sy2 += x2 * s.shift;
    }
    const double det = s2 * s4 - s3 * s3;
    if (use.size() < 3 || std::abs(det) < 1e-9) return false;
    a = (sy1 * s4 - sy2 * s3) / det;
    b = (s2 * sy2 - s3 * sy1) / det;
    return true;
  };
  for (std::size_t bin = 0; bin < bins; ++bin) {
    const double x = kMinX + (bin + 0.5) * kBinM;
    const double predicted = c1 * x + c2 * x * x;
    const auto [left, right] = nearest_walls(bin, predicted);
    const double gate = 0.3 + 0.01 * x;
    double sum = 0;
    int count = 0;
    if (std::isfinite(left0) && std::isfinite(left) && std::abs(left - left0 - predicted) <= gate)
      sum += left - left0, ++count;
    if (std::isfinite(right0) && std::isfinite(right) &&
        std::abs(right - right0 - predicted) <= gate)
      sum += right - right0, ++count;
    if (!count) continue;
    slices.push_back({x, sum / count});
    if (slices.size() >= 5) fit(slices, c1, c2);
  }
  if (slices.size() < kMinSlices || slices.back().x < 20) return route;
  // Refit without the worst tenth of residuals: a gated outlier can still bend the curve.
  fit(slices, c1, c2);
  std::vector<double> residuals;
  for (const auto& s : slices) residuals.push_back(std::abs(s.shift - (c1 * s.x + c2 * s.x * s.x)));
  auto sorted = residuals;
  std::sort(sorted.begin(), sorted.end());
  const double cutoff = sorted[sorted.size() * 9 / 10];
  std::vector<Slice> kept;
  for (std::size_t i = 0; i < slices.size(); ++i)
    if (residuals[i] <= cutoff) kept.push_back(slices[i]);
  if (!fit(kept, c1, c2) || std::abs(c1) > 0.05 || std::abs(c2) > c2_limit) return route;
  route.c1 = c1;
  route.c2 = c2;
  route.valid = true;
  route.max_x = kept.back().x + kBinM / 2;
  return route;
}

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
// curved wall, sampled at sensor-fixed heights, from favouring zero shift. The
// error of each forward shift goes to SpeedTracker, which keeps the speed.
constexpr double kProfileMinX = 3, kProfileMaxX = 35, kProfileStepM = SpeedTracker::kStepM;
constexpr std::size_t kProfileBins = 640, kProfileRows = 6;
constexpr std::size_t kProfileMinPairs = 200;
constexpr double kProfileMaxError = 0.3;
// Below one profile step per frame the train counts as standing still.
constexpr double kStandstillSpeedMps = 0.5;
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
  MotionDiagnostics diagnostics;
  diagnostics.recorded = config.record_motion;
  diagnostics.has_previous = !motion_profile_.empty();
  diagnostics.dt_s = dt;
  const auto max_shift = std::min<std::size_t>(
      kProfileBins / 2,
      static_cast<std::size_t>(std::ceil(config.ego_max_speed_mps * dt / kProfileStepM)));
  std::vector<double> errors;
  if (dt > 0 && max_shift >= 4 && !motion_profile_.empty()) {
    // Mean profile difference for each forward shift; a static structure at x in
    // the previous frame is now at x - shift.
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
  }
  const auto step = speed_.update(errors, dt, config.ego_min_contrast,
                                  config.record_motion ? &diagnostics : nullptr);
  const bool valid = step.valid;
  const double displacement = step.displacement_m;
  if (valid) {
    odometry_m_ += displacement;
    last_speed_mps_ = speed_.speed_mps();
  } else {
    // History recorded before a break cannot be moved into this frame.
    ++motion_epoch_;
  }
  motion_profile_ = std::move(profile);
  if (measurement_time_ns) motion_stamp_ns_ = measurement_time_ns;
  result.ego_motion_valid = valid;
  result.ego_speed_mps = valid && dt > 0 ? displacement / dt : 0;
  if (config.record_motion) result.motion = std::move(diagnostics);
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
  // Without a route estimate the corridor is the straight line y = 0, as before.
  const auto route = estimate_route(config, frame, ground);
  result.route = route;
  const double norm = std::sqrt(1 + ground.a * ground.a + ground.b * ground.b);
  auto route_point = [&](double x) {
    const double y = route.center(x);
    return PointXYZ{x, y, ground.a * x + ground.b * y + ground.c};
  };
  // A curved route is published as a chain of short straight segments.
  const double segment_m = route.valid ? 5.0 : end_x - config.detection_roi.min[0];
  for (double x0 = config.detection_roi.min[0]; x0 < end_x - 1e-6; x0 += segment_m) {
    CorridorSegment segment;
    segment.start = route_point(x0);
    segment.end = route_point(std::min(end_x, x0 + segment_m));
    segment.width_m = 2.0 * config.corridor_half_width_m;
    segment.height_m = config.corridor_height_m;
    segment.ground_plane = {-ground.a / norm, -ground.b / norm, 1 / norm, -ground.c / norm};
    segment.ground_inliers = static_cast<std::uint32_t>(ground.inliers);
    segment.geometry_valid = true;
    segment.coverage_valid = true;
    result.corridor.push_back(segment);
  }
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
  // The background map, the cells and the component loops all need the range and the angular
  // cell of the same returns; each is computed once per return and frame.
  std::vector<double> range_cache(frame.geometry_points.size(), -1.0);
  std::vector<std::int64_t> cell_cache(frame.geometry_points.size(), -1);
  auto range_of = [&](std::size_t index) {
    double& range = range_cache[index];
    if (range < 0) range = sensor_range(frame.geometry_points[index].point);
    return range;
  };
  auto cell_of = [&](std::size_t index) {
    std::int64_t& cell = cell_cache[index];
    if (cell < 0) {
      const auto [az, el] = angular_cell(frame.geometry_points[index].point);
      cell = key(az, el);
    }
    return cell;
  };
  // Candidates may extend this far past the corridor edge; only returns inside
  // the corridor decide acceptance and distance.
  const double band_half_width = config.corridor_half_width_m + config.candidate_margin_m;
  std::unordered_map<std::int64_t, std::size_t> current;  // Nearest return per cell.
  if (config.background_history_frames) {
    // The full range image includes floor and tunnel returns. Foreground is a
    // nearer surface in the same angular cell, as in analyze_bag.py. The history
    // keeps returns beyond the usable range so it still covers it after moving.
    for (const auto index : frame.detection_indices) {
      const auto& p = frame.geometry_points[index].point;
      const double height = ground.height(p);
      if (p.x < config.detection_roi.min[0] || std::abs(route.offset(p)) > band_half_width ||
          height < -0.5 || height > config.corridor_height_m)
        continue;
      const double range = range_of(index);
      const auto found = current.try_emplace(cell_of(index), index);
      if (!found.second && range < range_of(found.first->second)) found.first->second = index;
    }
    if (history_.size() < config.background_history_frames) {
      history_.push_back(make_history(current, range_cache));
      result.status = AnalysisStatus::INVALID_GEOMETRY;
      result.reason = "BASELINE_WARMUP";
      result.evaluation_region_valid = false;
      result.evaluated_range_m = 0;
      for (auto& segment : result.corridor) segment.coverage_valid = false;
      return;
    }
  }
  // Each current return is moved back into the baseline frames by the odometry
  // since they were recorded and checked against what their own rays saw there.
  // Frames recorded before an odometry break cannot be moved into this one:
  // compared without a shift they are right only for a train that last stood
  // still (or whose speed was never known), and are skipped after it moved.
  // Without ego-motion compensation (B0) the shift is always zero.
  std::vector<std::pair<std::size_t, double>> baselines;  // History index, shift.
  bool moved = false;
  if (config.background_history_frames) {
    const bool stood_still = last_speed_mps_ < kStandstillSpeedMps;
    const auto end = history_.size() - config.background_lag_frames;
    for (std::size_t i = 0; i < end; ++i) {
      const bool same_chain = history_[i].epoch == motion_epoch_;
      if (config.ego_motion_compensation && !same_chain && !stood_still) continue;
      const double shift =
          config.ego_motion_compensation && same_chain ? odometry_m_ - history_[i].odometry_m : 0.0;
      baselines.emplace_back(i, shift);
      moved = moved || shift > 0;
    }
  }
  const bool moving =
      result.ego_motion_valid && result.ego_speed_mps >= config.envelope_min_speed_mps;
  std::unordered_map<std::int64_t, bool> foreground_cache;
  auto foreground = [&](std::int64_t cell_key) {
    if (!config.background_history_frames) return true;
    if (baselines.empty()) return false;  // After an odometry break: no evidence yet.
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
    for (const auto& [i, shift] : baselines) {
      const auto& p = frame.geometry_points[current_cell->second].point;
      const PointXYZ past{p.x + shift, p.y, p.z};
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
  // Gauge channel: anything inside the narrow route gauge, independent of the history.
  const bool gauge_on = config.static_channel && route.valid;
  const double gauge_half_width =
      std::min(config.static_half_width_m, config.corridor_half_width_m);
  const double gauge_max_height = std::min(config.static_max_height_m, config.corridor_height_m);
  auto gauge_point = [&](double offset, double height) {
    return offset <= gauge_half_width && height >= config.static_min_height_m &&
           height <= gauge_max_height;
  };
  // Static low returns off the space between the rails are track structure (rails, the
  // contact rail, fastenings, ducts). They link a structure along the route so that its full
  // length is judged, but never place, size or range a candidate: an obstacle beside them
  // keeps its own position, and a structure alone has no support.
  auto structure_point = [&](double offset, double height) {
    return offset > config.low_object_half_width_m && height < config.low_object_height_m;
  };
  for (const auto index : frame.detection_indices) {
    const auto& p = frame.geometry_points[index].point;
    if (p.x > end_x || p.x < config.detection_roi.min[0]) continue;
    const double height = ground.height(p);
    const double offset = std::abs(route.offset(p));
    const bool in_band = offset <= band_half_width && height >= config.obstacle_min_height_m &&
                         height <= config.corridor_height_m;
    const bool in_gauge = gauge_on && p.x <= route.max_x && gauge_point(offset, height);
    if (!in_band && !in_gauge) continue;
    const double range = range_of(index);
    if (!(range > 0)) continue;
    const auto cell_key = cell_of(index);
    const int az = static_cast<int>(cell_key / 2048), el = static_cast<int>(cell_key % 2048);
    std::uint8_t channels = in_gauge ? ObstacleCandidate::kGauge : 0;
    if (in_band && foreground(cell_key)) channels |= ObstacleCandidate::kMotion;
    if (!channels) continue;
    const auto found = lookup.find(cell_key);
    if (found == lookup.end()) {
      lookup.emplace(cell_key, cells.size());
      cells.push_back({az, el, range, {index}, channels});
    } else {
      auto& cell = cells[found->second];
      cell.range = std::min(cell.range, range);
      cell.points.push_back(index);
      cell.channels |= channels;
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
  // Index nearby low returns once. An isolated bump over the rail bed can then be
  // compared with the same lateral strip immediately before and after it without
  // rescanning the entire cloud for every component.
  constexpr double kLowBinM = 0.5, kContextInnerM = 0.75, kContextOuterM = 2.5;
  std::vector<std::vector<std::size_t>> low_bins(
      static_cast<std::size_t>(std::ceil(end_x / kLowBinM)) + 1);
  if (config.static_channel) {
    for (std::size_t i = 0; i < frame.geometry_points.size(); ++i) {
      const auto& p = frame.geometry_points[i].point;
      if (p.x < 0 || p.x > end_x || std::abs(route.offset(p)) > config.static_half_width_m + 0.25)
        continue;
      const double height = ground.height(p);
      if (height < 0 || height > config.low_object_height_m) continue;
      low_bins[static_cast<std::size_t>(p.x / kLowBinM)].push_back(i);
    }
  }
  auto low_bump = [&](const PointXYZ& lo, const PointXYZ& hi, double top) {
    if (hi.x - lo.x > config.low_bump_max_length_m || hi.y - lo.y < config.low_bump_min_width_m)
      return false;
    const double x = (lo.x + hi.x) / 2, y = (lo.y + hi.y) / 2;
    std::vector<double> before, after;
    const auto first = static_cast<std::size_t>(std::max(0.0, x - kContextOuterM) / kLowBinM);
    const auto last =
        std::min(low_bins.size() - 1, static_cast<std::size_t>((x + kContextOuterM) / kLowBinM));
    for (std::size_t bin = first; bin <= last; ++bin) {
      for (const auto index : low_bins[bin]) {
        const auto& p = frame.geometry_points[index].point;
        if (std::abs(p.y - y) > 0.25) continue;
        const double dx = p.x - x;
        if (dx <= -kContextInnerM && dx >= -kContextOuterM)
          before.push_back(ground.height(p));
        else if (dx >= kContextInnerM && dx <= kContextOuterM)
          after.push_back(ground.height(p));
      }
    }
    if (before.size() < config.low_bump_min_context_points ||
        after.size() < config.low_bump_min_context_points)
      return false;
    auto upper = [](std::vector<double>& values) {
      const auto rank = 9 * (values.size() - 1) / 10;
      std::nth_element(values.begin(), values.begin() + rank, values.end());
      return values[rank];
    };
    return top >= std::max(upper(before), upper(after)) + config.low_bump_min_prominence_m;
  };
  for (const auto& entry : components) {
    PointXYZ possible_lo{INFINITY, INFINITY, INFINITY};
    PointXYZ possible_hi{-INFINITY, -INFINITY, -INFINITY};
    double possible_top = -INFINITY, possible_offset = 0;
    std::size_t possible_points = 0;
    for (const auto cell_index : entry.second) {
      const auto& cell = cells[cell_index];
      for (const auto point_index : cell.points) {
        const auto& p = frame.geometry_points[point_index].point;
        if (range_of(point_index) > cell.range + 0.5 ||
            std::abs(route.offset(p)) > config.corridor_half_width_m)
          continue;
        possible_lo.x = std::min(possible_lo.x, p.x);
        possible_lo.y = std::min(possible_lo.y, p.y);
        possible_lo.z = std::min(possible_lo.z, p.z);
        possible_hi.x = std::max(possible_hi.x, p.x);
        possible_hi.y = std::max(possible_hi.y, p.y);
        possible_hi.z = std::max(possible_hi.z, p.z);
        possible_top = std::max(possible_top, ground.height(p));
        possible_offset += route.offset(p);
        ++possible_points;
      }
    }
    const bool raised_low =
        possible_points && possible_top < config.low_object_height_m &&
        std::abs(possible_offset / double(possible_points)) > config.low_object_half_width_m &&
        low_bump(possible_lo, possible_hi, possible_top);
    PointXYZ lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    PointXYZ nearest;
    double distance = INFINITY, inside_low = INFINITY, inside_high = -INFINITY;
    std::size_t support = 0, inside_cells = 0;
    std::uint8_t channels = 0;
    // Extent along the route of all gauge returns, including those behind the nearest
    // surface of each cell: a structure parallel to the route is seen at a grazing angle
    // and falls into few angular cells, so the per-cell nearest layer would look short.
    // Returns of the inner gauge are kept apart: rail-side equipment in the outer strip
    // next to a compact object on the route must not make the object long.
    double gauge_lo_x = INFINITY, gauge_hi_x = -INFINITY;
    double inner_lo_x = INFINITY, inner_hi_x = -INFINITY;
    double closest_offset = INFINITY;  // Of the evidence, from the route centre.
    std::vector<PointXYZ> structure;
    double inside_top = -INFINITY, inside_offset = 0;
    std::size_t inside_points = 0;
    for (const auto cell_index : entry.second) {
      const auto& cell = cells[cell_index];
      channels |= cell.channels;
      bool cell_inside = false;
      for (const auto point_index : cell.points) {
        const auto& p = frame.geometry_points[point_index].point;
        const double offset = std::abs(route.offset(p)), height = ground.height(p);
        if (gauge_on && gauge_point(offset, height)) {
          gauge_lo_x = std::min(gauge_lo_x, p.x);
          gauge_hi_x = std::max(gauge_hi_x, p.x);
          if (offset <= config.static_inner_half_width_m) {
            inner_lo_x = std::min(inner_lo_x, p.x);
            inner_hi_x = std::max(inner_hi_x, p.x);
          }
        }
        if (range_of(point_index) > cell.range + 0.5) continue;
        const bool inside = offset <= config.corridor_half_width_m;
        if (inside) {
          inside_top = std::max(inside_top, height);
          inside_offset += route.offset(p);
          ++inside_points;
        }
        if (!(cell.channels & ObstacleCandidate::kMotion) && structure_point(offset, height) &&
            !raised_low) {
          structure.push_back(p);
          continue;
        }
        closest_offset = std::min(closest_offset, offset);
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
        hi.z = std::max(hi.z, p.z);
        if (!inside) continue;
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
    // The bbox spans the margin band; acceptance uses only returns inside. Gauge-only
    // evidence has no history to tell a wall from an obstacle: long structures along the
    // route are rejected, and its returns already start above static_min_height_m.
    const bool gauge_only = channels == ObstacleCandidate::kGauge;
    const double min_extent = gauge_only ? 0.1 : config.obstacle_min_height_m;
    // A component seen only in the outer strip is judged by its full gauge extent.
    const double gauge_length =
        std::isfinite(inner_lo_x) ? inner_hi_x - inner_lo_x : gauge_hi_x - gauge_lo_x;
    const char* rejected = nullptr;
    if (inside_cells < config.min_candidate_cells) {
      rejected = "FEW_CELLS";
    } else if (support < config.min_candidate_points) {
      rejected = "FEW_POINTS";
    } else if (inside_high - inside_low < min_extent) {
      rejected = "LOW_EXTENT";
    } else if (gauge_only && gauge_length > config.static_max_length_m) {
      rejected = "LONG_GAUGE_STRUCTURE";
    } else if (closest_offset > gauge_half_width && hi.x - lo.x > config.static_max_length_m) {
      // Beside the gauge, a surface long along the route is a wall, a platform edge or a
      // cable seen as new through odometry error or disocclusion; an obstacle is compact.
      rejected = "LONG_EDGE_STRUCTURE";
    } else if (config.static_inner_half_width_m < gauge_half_width &&
               (channels & ObstacleCandidate::kGauge) &&
               std::abs(route.offset({(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, 0})) >
                   config.static_inner_half_width_m &&
               (hi.x - lo.x > config.static_outer_max_length_m ||
                hi.y - lo.y < config.static_outer_min_width_m ||
                support < config.static_outer_min_points)) {
      // The wider static channel sees rail and platform fragments. A compact
      // object there needs a resolved lateral face and enough current returns.
      rejected = "WEAK_OUTER_STATIC";
    } else if (moving && !(channels & ObstacleCandidate::kGauge) &&
               std::abs(route.offset({(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, 0})) >
                   config.envelope_half_width_m) {
      // A grazing point at the edge of a wall is not enough to put its whole component
      // in the vehicle envelope. The component centre must intrude while moving.
      rejected = "OUTSIDE_ENVELOPE";
    } else if (inside_top < config.low_object_height_m &&
               std::abs(inside_offset / double(inside_points)) > config.low_object_half_width_m &&
               !raised_low) {
      // A low off-centre component must rise above the same rail-side strip on
      // both sides. A continuous rail or walkway has no local protrusion.
      rejected = "LOW_OFF_CENTRE";
    }
    if (rejected) {
      if (config.record_rejected && std::isfinite(lo.x)) {
        RejectedComponent item;
        item.reason = rejected;
        item.center = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, (lo.z + hi.z) / 2};
        item.size = {hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
        item.cells = static_cast<std::uint32_t>(entry.second.size());
        item.points = static_cast<std::uint32_t>(support);
        item.channels = channels;
        result.rejected.push_back(item);
      }
      continue;
    }
    // Structure returns complete the box only alongside the evidence (the low part of a tall
    // object at the corridor edge), never extend it along the route.
    for (const auto& p : structure) {
      if (p.x < lo.x || p.x > hi.x) continue;
      lo.y = std::min(lo.y, p.y);
      lo.z = std::min(lo.z, p.z);
      hi.y = std::max(hi.y, p.y);
      hi.z = std::max(hi.z, p.z);
    }
    ObstacleCandidate candidate;
    candidate.center = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, (lo.z + hi.z) / 2};
    candidate.size = {hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
    candidate.nearest_point = nearest;
    candidate.distance_m = distance;
    candidate.distance_valid = std::isfinite(distance) && distance >= 0;
    candidate.support_points = static_cast<std::uint32_t>(support);
    candidate.channels = channels;
    if (candidate.distance_valid) result.candidates.push_back(candidate);
  }
  if (config.hanging_channel && config.static_channel && route.valid) {
    // Thin suspended obstacles are easily absorbed by the rolling background and
    // have too few points below the corridor roof for the ordinary angular cluster.
    // Group a narrow vertical column in metric XY, then require its lower tip to
    // enter the vehicle height. A broad overhead structure is not such a column.
    constexpr double kUpperCellM = 0.25;
    struct UpperCell {
      int x, y;
      std::vector<std::size_t> points;
    };
    std::vector<UpperCell> upper_cells;
    std::unordered_map<std::int64_t, std::size_t> upper_lookup;
    for (const auto index : frame.detection_indices) {
      const auto& p = frame.geometry_points[index].point;
      const double height = ground.height(p);
      if (p.x < config.detection_roi.min[0] || p.x > end_x ||
          std::abs(route.offset(p)) > config.low_object_half_width_m ||
          height < config.hanging_tip_max_height_m - 0.5 || height > config.corridor_height_m + 1.5)
        continue;
      const int ix = static_cast<int>(std::floor(p.x / kUpperCellM));
      const int iy = static_cast<int>(std::floor(p.y / kUpperCellM));
      const auto cell_key = key(ix, iy + 1024);
      const auto found = upper_lookup.find(cell_key);
      if (found == upper_lookup.end()) {
        upper_lookup.emplace(cell_key, upper_cells.size());
        upper_cells.push_back({ix, iy, {index}});
      } else {
        upper_cells[found->second].points.push_back(index);
      }
    }
    DisjointSet upper_groups(upper_cells.size());
    for (std::size_t i = 0; i < upper_cells.size(); ++i)
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
          const auto near =
              upper_lookup.find(key(upper_cells[i].x + dx, upper_cells[i].y + dy + 1024));
          if (near != upper_lookup.end()) upper_groups.join(i, near->second);
        }
    std::unordered_map<std::size_t, std::vector<std::size_t>> upper_components;
    for (std::size_t i = 0; i < upper_cells.size(); ++i)
      upper_components[upper_groups.root(i)].push_back(i);
    for (const auto& component : upper_components) {
      PointXYZ lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
      double tip = INFINITY, top = -INFINITY;
      std::size_t count = 0;
      for (const auto cell_index : component.second)
        for (const auto point_index : upper_cells[cell_index].points) {
          const auto& p = frame.geometry_points[point_index].point;
          lo.x = std::min(lo.x, p.x), lo.y = std::min(lo.y, p.y), lo.z = std::min(lo.z, p.z);
          hi.x = std::max(hi.x, p.x), hi.y = std::max(hi.y, p.y), hi.z = std::max(hi.z, p.z);
          tip = std::min(tip, ground.height(p));
          top = std::max(top, ground.height(p));
          ++count;
        }
      if (count < config.hanging_min_points || tip > config.hanging_tip_max_height_m ||
          top - tip < config.hanging_min_vertical_span_m ||
          hi.x - lo.x > config.hanging_max_footprint_m ||
          hi.y - lo.y > config.hanging_max_footprint_m)
        continue;
      const PointXYZ centre{(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, (lo.z + hi.z) / 2};
      if (std::any_of(
              result.candidates.begin(), result.candidates.end(), [&](const auto& candidate) {
                return std::hypot(candidate.center.x - centre.x, candidate.center.y - centre.y) <
                       0.5;
              }))
        continue;
      ObstacleCandidate candidate;
      candidate.center = centre;
      candidate.size = {hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
      candidate.nearest_point = lo;
      candidate.distance_m = lo.x - origin.x;
      candidate.distance_valid = std::isfinite(candidate.distance_m) && candidate.distance_m > 0;
      candidate.support_points = static_cast<std::uint32_t>(count);
      candidate.channels = ObstacleCandidate::kGauge;
      if (candidate.distance_valid) result.candidates.push_back(candidate);
    }
  }
  std::sort(result.candidates.begin(), result.candidates.end(),
            [](const auto& a, const auto& b) { return a.distance_m < b.distance_m; });
  for (std::size_t i = 0; i < result.candidates.size(); ++i) result.candidates[i].id = i + 1;
  // A stationary object already present in history can disappear from the foreground, so
  // differencing alone cannot certify a clear path. The gauge channel needs no history: where it
  // is active (a valid route, up to its farthest wall support) an empty gauge is evidence of a
  // clear path, and the evaluated range shrinks to it.
  const double gauge_end =
      gauge_on && config.gauge_certifies_clear ? std::min(end_x, route.max_x) : 0.0;
  const bool gauge_clear = gauge_end > config.detection_roi.min[0];
  result.status = AnalysisStatus::OK;
  result.reason = !result.candidates.empty() ? "CANDIDATES_FOUND"
                  : !config.background_history_frames || gauge_clear
                      ? "NO_CANDIDATE_IN_EVALUATED_REGION"
                      : "BACKGROUND_CANNOT_CONFIRM_CLEAR";
  if (config.background_history_frames) {
    if (gauge_clear) {
      result.evaluated_range_m = gauge_end;
      for (auto& segment : result.corridor)
        segment.coverage_valid = segment.end.x <= gauge_end + 1e-6;
    } else {
      result.evaluation_region_valid = false;
      for (auto& segment : result.corridor) segment.coverage_valid = false;
    }
    history_.pop_front();
    history_.push_back(make_history(current, range_cache));
  }
}

GeometricDetector::HistoryFrame GeometricDetector::make_history(
    const std::unordered_map<std::int64_t, std::size_t>& nearest,
    const std::vector<double>& ranges) const {
  HistoryFrame frame{{}, odometry_m_, motion_epoch_};
  frame.ranges.reserve(nearest.size());
  for (const auto& [cell, index] : nearest) frame.ranges.emplace(cell, ranges[index]);
  return frame;
}
}  // namespace metro_perception_core
