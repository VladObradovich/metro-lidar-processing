#include "metro_perception_core/detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

namespace metro_perception_core {
namespace {
constexpr double kPi = 3.14159265358979323846;
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

Plane estimate_ground(const AlgorithmConfig& config, const PreprocessedFrame& frame) {
  std::vector<const PointXYZ*> seeds;
  const auto& points = frame.geometry_points;
  const std::size_t stride = std::max<std::size_t>(1, points.size() / 6000);
  for (std::size_t i = 0; i < points.size(); i += stride) {
    const auto& p = points[i].point;
    if (p.x >= 2 && p.x <= 90 && std::abs(p.y) <= 3 && p.z >= -3 && p.z <= 0.5) seeds.push_back(&p);
  }
  if (seeds.size() < config.min_ground_inliers) return {};
  Plane best;
  std::uint32_t random = 0x9e3779b9u;
  auto pick = [&] {
    random = random * 1664525u + 1013904223u;
    return std::size_t(random) % seeds.size();
  };
  for (int attempt = 0; attempt < 96; ++attempt) {
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
    if (std::hypot(a, b) > config.ground_max_slope || c < -3 || c > 0.5) continue;
    std::size_t inliers = 0;
    std::uint8_t along = 0, across = 0;
    double max_x = 0;
    for (const auto* point : seeds) {
      if (std::abs(point->z - a * point->x - b * point->y - c) > config.ground_inlier_tolerance_m)
        continue;
      ++inliers;
      if (point->x < 25) along |= 1;
      if (point->x >= 25) along |= 2;
      if (point->y < -0.25) across |= 1;
      if (point->y > 0.25) across |= 2;
      max_x = std::max(max_x, point->x);
    }
    // A small patch, rail or platform edge cannot establish the route surface.
    if (inliers < config.min_ground_inliers || along != 3 || across != 3) continue;
    if (!best.valid || inliers > best.inliers || (inliers == best.inliers && c < best.c)) {
      best = {a, b, c, inliers, max_x, true};
    }
  }
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
}  // namespace

void GeometricDetector::process(const AlgorithmConfig& config, FrameResult& result,
                                std::int64_t measurement_time_ns) {
  if (measurement_time_ns && last_stamp_ns_ &&
      (measurement_time_ns <= last_stamp_ns_ ||
       measurement_time_ns - last_stamp_ns_ > 2000000000LL))
    reset();
  if (measurement_time_ns) last_stamp_ns_ = measurement_time_ns;
  const auto& frame = result.preprocessed;
  const auto ground = estimate_ground(config, frame);
  if (!ground.valid) {
    reset();
    result.status = AnalysisStatus::INVALID_GEOMETRY;
    result.reason = "GROUND_UNSUPPORTED";
    return;
  }
  const double end_x = std::min(config.detection_roi.max[0], ground.max_supported_x + 3.0);
  if (end_x < 25.0 || frame.detection_indices.empty()) {
    reset();
    result.status = AnalysisStatus::INVALID_GEOMETRY;
    result.reason = "CORRIDOR_UNOBSERVABLE";
    return;
  }
  CorridorSegment segment;
  segment.start = {config.detection_roi.min[0], 0, ground.c};
  segment.end = {end_x, 0, ground.a * end_x + ground.c};
  segment.width_m = 2.0 * config.corridor_half_width_m;
  segment.height_m = config.corridor_height_m;
  const double norm = std::sqrt(1 + ground.a * ground.a + ground.b * ground.b);
  segment.ground_plane = {-ground.a / norm, -ground.b / norm, 1 / norm, -ground.c / norm};
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
  RangeMap current;
  if (config.background_history_frames) {
    // The full range image includes floor and tunnel returns. Foreground is a
    // nearer surface in the same angular cell, as in analyze_bag.py.
    for (const auto index : frame.detection_indices) {
      const auto& p = frame.geometry_points[index].point;
      const double height = ground.height(p);
      if (p.x > end_x || p.x < config.detection_roi.min[0] ||
          std::abs(p.y) > config.corridor_half_width_m || height < -0.5 ||
          height > config.corridor_height_m)
        continue;
      const double range = std::hypot(std::hypot(p.x - origin.x, p.y - origin.y), p.z - origin.z);
      const auto [az, el] = angular_cell(p);
      const auto cell_key = key(az, el);
      const auto found = current.find(cell_key);
      if (found == current.end())
        current.emplace(cell_key, range);
      else
        found->second = std::min(found->second, range);
    }
    if (history_.size() < config.background_history_frames) {
      history_.push_back(std::move(current));
      result.status = AnalysisStatus::INVALID_GEOMETRY;
      result.reason = "BASELINE_WARMUP";
      result.evaluation_region_valid = false;
      result.evaluated_range_m = 0;
      result.corridor.front().coverage_valid = false;
      return;
    }
  }
  std::unordered_map<std::int64_t, bool> foreground_cache;
  auto foreground = [&](std::int64_t cell_key) {
    if (!config.background_history_frames) return true;
    const auto known = foreground_cache.find(cell_key);
    if (known != foreground_cache.end()) return known->second;
    const auto current_cell = current.find(cell_key);
    if (current_cell == current.end()) return false;
    std::vector<double> history_ranges;
    const auto end = history_.size() - config.background_lag_frames;
    for (std::size_t i = 0; i < end; ++i) {
      const auto found = history_[i].find(cell_key);
      if (found != history_[i].end()) history_ranges.push_back(found->second);
    }
    // Newly occupied cells follow the script's "appeared" path. A sporadic
    // single baseline return is insufficient evidence either way.
    bool is_foreground = history_ranges.empty();
    if (history_ranges.size() >= 2) {
      std::sort(history_ranges.begin(), history_ranges.end());
      const double baseline = history_ranges[history_ranges.size() / 2];
      is_foreground = current_cell->second < baseline - config.background_margin_m -
                                                 config.background_relative_margin * baseline;
    }
    foreground_cache.emplace(cell_key, is_foreground);
    return is_foreground;
  };
  std::unordered_map<std::int64_t, std::size_t> lookup;
  std::vector<Cell> cells;
  for (const auto index : frame.detection_indices) {
    const auto& p = frame.geometry_points[index].point;
    const double height = ground.height(p);
    if (p.x > end_x || p.x < config.detection_roi.min[0] ||
        std::abs(p.y) > config.corridor_half_width_m || height < config.obstacle_min_height_m ||
        height > config.corridor_height_m)
      continue;
    const double sx = p.x - origin.x, sy = p.y - origin.y, sz = p.z - origin.z;
    const double range = std::sqrt(sx * sx + sy * sy + sz * sz);
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
    if (entry.second.size() < config.min_candidate_cells) continue;
    PointXYZ lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    PointXYZ nearest;
    double distance = INFINITY;
    std::size_t support = 0;
    for (const auto cell_index : entry.second) {
      const auto& cell = cells[cell_index];
      for (const auto point_index : cell.points) {
        const auto& p = frame.geometry_points[point_index].point;
        const double sensor_range =
            std::hypot(std::hypot(p.x - origin.x, p.y - origin.y), p.z - origin.z);
        if (sensor_range > cell.range + 0.5) continue;
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
        hi.z = std::max(hi.z, p.z);
        if (p.x - origin.x < distance) {
          distance = p.x - origin.x;
          nearest = p;
        }
        ++support;
      }
    }
    if (support < config.min_candidate_points || hi.z - lo.z < config.obstacle_min_height_m)
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
    history_.push_back(std::move(current));
  }
}
}  // namespace metro_perception_core
