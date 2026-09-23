#include "metro_perception_core/temporal_monitor.hpp"

namespace metro_perception_core {
Assessment TemporalMonitor::update(const FrameResult& frame, std::int64_t) {
  Assessment result;
  result.calibration_trust = frame.calibration_trust;
  if (frame.status != AnalysisStatus::OK) {
    result.reason = frame.reason.empty() ? "ANALYSIS_FAILED" : frame.reason;
    return result;
  }
  const ObstacleCandidate* nearest = nullptr;
  for (const auto& candidate : frame.candidates) {
    if (!usable_distance(candidate.distance_valid, candidate.distance_m)) continue;
    if (!nearest || candidate.distance_m < nearest->distance_m) nearest = &candidate;
  }
  if (nearest) {
    // B0 reports a measured candidate immediately; temporal confirmation is G4.
    result.state = State::OBSTACLE;
    switch (frame.calibration_trust) {
      case CalibrationTrust::ASSUMED:
        result.reason = "OBSTACLE_WITH_ASSUMED_CALIBRATION";
        break;
      case CalibrationTrust::VERIFIED:
        result.reason = "OBSTACLE_CANDIDATE";
        break;
      default:
        result.reason = "OBSTACLE_WITH_UNKNOWN_CALIBRATION";
    }
    result.distance_m = nearest->distance_m;
    result.distance_valid = true;
    return result;
  }
  // Something was found but cannot be ranged: never report this as a clear path.
  if (!frame.candidates.empty()) {
    result.reason = "CANDIDATE_DISTANCE_INVALID";
    return result;
  }
  const bool region_usable = frame.evaluation_region_valid &&
                             std::isfinite(frame.evaluated_range_m) && frame.evaluated_range_m > 0;
  switch (frame.calibration_trust) {
    case CalibrationTrust::ASSUMED:
      result.reason = "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR";
      break;
    case CalibrationTrust::UNKNOWN:
      result.reason = "CALIBRATION_TRUST_UNKNOWN";
      break;
    case CalibrationTrust::VERIFIED:
      if (region_usable) {
        result.state = State::NO_OBSTACLE_DETECTED;
        result.reason = "NO_CANDIDATE_IN_EVALUATED_REGION";
      } else if (frame.reason == "BACKGROUND_CANNOT_CONFIRM_CLEAR") {
        result.reason = frame.reason;
      } else {
        result.reason =
            frame.evaluation_region_valid ? "EVALUATION_REGION_INVALID" : "CORRIDOR_UNOBSERVABLE";
      }
      break;
  }
  return result;
}
Assessment TemporalMonitor::on_timeout() const {
  Assessment result;
  result.reason = "INPUT_PAUSED_OR_STOPPED";
  return result;
}
}  // namespace metro_perception_core
