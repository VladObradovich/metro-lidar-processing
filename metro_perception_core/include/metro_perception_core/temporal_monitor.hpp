#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include "metro_perception_core/config.hpp"
#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
// A candidate distance may decide the path state only if it is marked valid, finite and positive.
inline bool usable_distance(bool distance_valid, double distance_m) {
  return distance_valid && std::isfinite(distance_m) && distance_m > 0;
}

// Decision shared by the online monitor and evaluate_bag (G4). Candidates are followed over
// frames: a track is predicted by the lidar ego motion (objects are assumed fixed in the
// world, moving ones stay inside the gate), confirmed after N hits in its last M frames and
// kept for release_misses - 1 missed frames. OBSTACLE needs a confirmed track with a usable
// distance; unconfirmed evidence is UNKNOWN, never a clear path. The default configuration
// (1/1, release 1) is exactly the per-frame decision.
class TemporalMonitor {
 public:
  TemporalMonitor() = default;
  explicit TemporalMonitor(TemporalConfig config);
  Assessment update(const FrameResult& frame, std::int64_t measurement_time_ns);
  Assessment on_timeout() const;
  void reset();
  const TemporalConfig& config() const { return config_; }

 private:
  struct Track {
    std::uint64_t id{0};
    PointXYZ center, size;
    double distance_m{0};
    std::uint32_t history{0};  // Bit 0 = this frame.
    std::uint32_t age_frames{0}, misses{0};
    std::uint8_t channels{0};
    std::uint32_t support{0};  // Of the last measurement.
    double offset{NAN};        // Closest evidence offset of the last measurement.
    double offset_before{NAN}, offset_earlier{NAN};  // Of the two measurements before it.
    bool confirmed{false};
  };
  bool update_tracks(const FrameResult& frame, std::int64_t measurement_time_ns);
  std::vector<TrackSummary> summaries() const;

  TemporalConfig config_;
  std::vector<Track> tracks_;
  std::int64_t last_time_ns_{0};
  std::uint64_t next_id_{1};
};
}  // namespace metro_perception_core
