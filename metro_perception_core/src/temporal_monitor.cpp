#include "metro_perception_core/temporal_monitor.hpp"
namespace metro_perception_core {
Assessment TemporalMonitor::update(const FrameResult& frame, std::int64_t) {
  Assessment result;
  result.calibration_trust = frame.calibration_trust;
  if (frame.status != AnalysisStatus::OK) {
    result.reason = frame.reason;
    return result;
  }
  // Assumed mounting may support a future obstacle candidate, but an empty candidate
  // set must never prove that the path is clear without verified calibration.
  if (frame.calibration_trust == CalibrationTrust::ASSUMED && frame.candidates.empty()) {
    result.reason = "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR";
    return result;
  }
  if (frame.calibration_trust != CalibrationTrust::VERIFIED) {
    result.reason = "CALIBRATION_TRUST_UNKNOWN";
    return result;
  }
  // R02/X03: temporal confirmation and final state transitions are still pending.
  result.reason = "MONITOR_NOT_IMPLEMENTED";
  return result;
}
Assessment TemporalMonitor::on_timeout() const {
  Assessment result;
  result.reason = "INPUT_PAUSED_OR_STOPPED";
  return result;
}
}  // namespace metro_perception_core
