#include "metro_perception_core/temporal_monitor.hpp"

#include <algorithm>
#include <limits>

namespace metro_perception_core {
Assessment TemporalMonitor::update(const FrameResult& frame, std::int64_t) {
  Assessment result;
  result.calibration_trust = frame.calibration_trust;
  if (frame.status != AnalysisStatus::OK) {
    result.reason = frame.reason;
    return result;
  }
  const ObstacleCandidate* nearest = nullptr;
  for (const auto& candidate : frame.candidates) {
    if (!candidate.distance_valid) continue;
    if (!nearest || candidate.distance_m < nearest->distance_m) nearest = &candidate;
  }
  if (nearest) {
    // B0 reports a measured candidate immediately; temporal confirmation is G4.
    result.state = State::OBSTACLE;
    result.reason = frame.calibration_trust == CalibrationTrust::ASSUMED
                        ? "OBSTACLE_WITH_ASSUMED_CALIBRATION"
                        : "OBSTACLE_CANDIDATE";
    result.distance_m = nearest->distance_m;
    result.distance_valid = true;
    return result;
  }
  if (frame.calibration_trust == CalibrationTrust::ASSUMED) {
    result.reason = "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR";
  } else if (frame.calibration_trust == CalibrationTrust::VERIFIED &&
             frame.evaluation_region_valid) {
    result.state = State::NO_OBSTACLE_DETECTED;
    result.reason = "NO_CANDIDATE_IN_EVALUATED_REGION";
  } else {
    result.reason =
        frame.reason == "BACKGROUND_CANNOT_CONFIRM_CLEAR" ? frame.reason : "CORRIDOR_UNOBSERVABLE";
  }
  return result;
}
Assessment TemporalMonitor::on_timeout() const {
  Assessment result;
  result.reason = "INPUT_PAUSED_OR_STOPPED";
  return result;
}
}  // namespace metro_perception_core
