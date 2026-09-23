#pragma once
#include <cmath>

#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
// A candidate distance may decide the path state only if it is marked valid, finite and positive.
inline bool usable_distance(bool distance_valid, double distance_m) {
  return distance_valid && std::isfinite(distance_m) && distance_m > 0;
}

// Per-frame decision shared by the online monitor and evaluate_bag. Temporal confirmation
// across frames belongs to G4; until then every frame is decided on its own.
class TemporalMonitor {
 public:
  Assessment update(const FrameResult& frame, std::int64_t measurement_time_ns);
  Assessment on_timeout() const;
  void reset() {}  // Add history together with G4, not before.
};
}  // namespace metro_perception_core
