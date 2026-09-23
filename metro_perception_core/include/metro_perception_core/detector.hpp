#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include "metro_perception_core/config.hpp"
#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
class GeometricDetector {
 public:
  void process(const AlgorithmConfig& config, FrameResult& result,
               std::int64_t measurement_time_ns);
  void reset() {
    history_.clear();
    last_stamp_ns_ = 0;
    geometry_failures_ = 0;
    motion_profile_.clear();
    motion_stamp_ns_ = 0;
    odometry_m_ = 0;
    speed_mps_ = 0;
    unconfirmed_s_ = 0;
    pending_speed_mps_ = 0;
    pending_frames_ = 0;
    speed_known_ = false;
    ++motion_epoch_;
  }

 private:
  using RangeMap = std::unordered_map<std::int64_t, double>;
  // Range of the nearest return per angular cell, with the forward odometry at
  // recording time.
  struct HistoryFrame {
    RangeMap ranges;
    double odometry_m{0};
    std::uint64_t epoch{0};
  };
  void estimate_motion(const AlgorithmConfig& config, const PreprocessedFrame& frame,
                       const std::array<double, 3>& ground, std::int64_t measurement_time_ns,
                       FrameResult& result);
  HistoryFrame make_history(const std::unordered_map<std::int64_t, PointXYZ>& nearest,
                            const PointXYZ& origin) const;

  std::deque<HistoryFrame> history_;
  std::int64_t last_stamp_ns_{0};
  std::size_t geometry_failures_{0};
  // Lidar-only forward odometry; the epoch changes whenever the chain breaks.
  std::vector<float> motion_profile_;  // Lateral profile of the previous frame.
  std::int64_t motion_stamp_ns_{0};
  double odometry_m_{0}, speed_mps_{0}, unconfirmed_s_{0}, pending_speed_mps_{0};
  std::size_t pending_frames_{0};
  bool speed_known_{false};
  std::uint64_t motion_epoch_{0};
};
}  // namespace metro_perception_core
