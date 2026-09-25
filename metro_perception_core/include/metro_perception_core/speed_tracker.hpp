#pragma once
#include <cstddef>
#include <vector>

#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
// Lidar-only forward speed from how well the tunnel profiles of consecutive frames
// match per forward shift (GeometricDetector builds the profiles). Kept apart from
// the profiles so that its rules can be tested on error curves directly.
class SpeedTracker {
 public:
  static constexpr double kStepM = 0.05;  // One forward shift of the profile.
  struct Step {
    bool valid{false};
    double displacement_m{0};
  };
  // errors[s]: mean profile difference for a forward shift of s steps over dt_s,
  // INFINITY without enough comparable pairs; empty when the frames cannot be
  // compared. Fills the tracker fields of `diagnostics` when it is given.
  Step update(const std::vector<double>& errors, double dt_s, double min_contrast,
              MotionDiagnostics* diagnostics = nullptr);
  void reset();
  bool speed_known() const { return speed_known_; }
  double speed_mps() const { return speed_mps_; }

 private:
  double speed_mps_{0}, unconfirmed_s_{0}, pending_speed_mps_{0}, escape_speed_mps_{0};
  std::size_t pending_frames_{0}, escape_frames_{0};
  bool speed_known_{false};
};
}  // namespace metro_perception_core
