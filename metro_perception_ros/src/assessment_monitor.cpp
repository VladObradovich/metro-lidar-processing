#include "metro_perception_ros/assessment_monitor.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "metro_perception_ros/measurement_time.hpp"
#include "metro_perception_ros/wire_enum_decode.hpp"

namespace metro_perception_ros {
namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}

AssessmentMonitor::AssessmentMonitor(double timeout_s,
                                     metro_perception_core::TemporalConfig temporal)
    : timeout_ms_(timeout_s * 1000), monitor_(temporal) {
  if (!std::isfinite(timeout_s) || timeout_s <= 0) {
    throw std::invalid_argument("timeout_s must be positive");
  }
  output_.state = PathAssessment::UNKNOWN;
  output_.reason = "WAITING_FOR_INPUT";
  output_.stale = true;
  output_.distance_m = kNaN;
}

GateDecision AssessmentMonitor::on_analysis(const FrameAnalysis& frame, Clock::time_point now) {
  // An unknown age cannot be treated as fresh. Reject before the gate, so the message has no
  // side effects and the last valid result keeps ageing towards the watchdog.
  const bool age_valid = std::isfinite(frame.processing_age_ms) && frame.processing_age_ms >= 0;
  const auto stamp = decode_measurement_time_ns(frame.header.stamp);
  const auto decision =
      age_valid
          ? gate_.admit({frame.source_instance_id, frame.session_id, frame.frame_sequence, stamp})
          : GateDecision{false, false, "INVALID_PROCESSING_AGE"};
  if (!decision.accepted) {
    ++output_.rejected_analyses;
    output_.last_rejection_reason = decision.reason;
    return decision;
  }
  if (decision.reset) monitor_.reset();

  metro_perception_core::FrameResult result;
  result.status = decode_analysis_status(frame.processing_status);
  result.reason = frame.reason;
  if (result.status == metro_perception_core::AnalysisStatus::BAD_INPUT &&
      frame.processing_status != FrameAnalysis::BAD_INPUT) {
    result.reason = "INVALID_PROCESSING_STATUS";
  }
  result.calibration_trust = decode_calibration_trust(frame.calibration_trust);
  result.evaluation_region_valid = frame.evaluation_region_valid;
  result.evaluated_range_m = frame.evaluated_range_m;
  result.ego_motion_valid = frame.ego_motion_valid;
  result.ego_speed_mps = frame.ego_speed_mps;
  for (const auto& item : frame.candidates) {
    metro_perception_core::ObstacleCandidate candidate;
    candidate.id = item.candidate_id;
    candidate.center = {item.bbox.center.position.x, item.bbox.center.position.y,
                        item.bbox.center.position.z};
    candidate.size = {item.bbox.size.x, item.bbox.size.y, item.bbox.size.z};
    candidate.nearest_point = {item.nearest_point.x, item.nearest_point.y, item.nearest_point.z};
    candidate.distance_m = item.distance_m;
    candidate.distance_valid = item.distance_valid;
    candidate.support_points = item.support_points;
    for (const auto& reason : item.reasons) {
      if (reason == "MOTION")
        candidate.channels |= metro_perception_core::ObstacleCandidate::kMotion;
      if (reason == "GAUGE") candidate.channels |= metro_perception_core::ObstacleCandidate::kGauge;
    }
    result.candidates.push_back(candidate);
  }
  const auto assessment = monitor_.update(result, *stamp);
  const bool analysed = result.status == metro_perception_core::AnalysisStatus::OK;

  output_.header = frame.header;
  output_.source_instance_id = frame.source_instance_id;
  output_.session_id = frame.session_id;
  output_.frame_sequence = frame.frame_sequence;
  output_.state = static_cast<std::uint8_t>(assessment.state);
  output_.reason = assessment.reason;
  output_.calibration_trust = static_cast<std::uint8_t>(assessment.calibration_trust);
  output_.distance_valid = assessment.distance_valid;
  output_.distance_m = assessment.distance_valid ? assessment.distance_m : kNaN;
  // Objects and corridor always come from the frame the decision was made on.
  output_.reported_objects = analysed ? frame.candidates : decltype(frame.candidates){};
  output_.corridor = analysed ? frame.corridor : decltype(frame.corridor){};
  output_.tracks.clear();
  for (const auto& track : assessment.tracks) {
    auto& item = output_.tracks.emplace_back();
    item.track_id = track.id;
    item.confirmed = track.confirmed;
    item.coasting = track.coasting;
    item.hits = track.hits;
    item.age_frames = track.age_frames;
    item.bbox.center.position.x = track.center.x;
    item.bbox.center.position.y = track.center.y;
    item.bbox.center.position.z = track.center.z;
    item.bbox.center.orientation.w = 1.0;
    item.bbox.size.x = track.size.x;
    item.bbox.size.y = track.size.y;
    item.bbox.size.z = track.size.z;
    item.distance_m = track.distance_m;
    if (track.channels & metro_perception_core::ObstacleCandidate::kMotion)
      item.reasons.push_back("MOTION");
    if (track.channels & metro_perception_core::ObstacleCandidate::kGauge)
      item.reasons.push_back("GAUGE");
  }
  output_.evaluation_region_valid = analysed && frame.evaluation_region_valid;
  output_.evaluated_range_m = analysed ? frame.evaluated_range_m : 0.0;
  output_.stale = false;
  processing_age_ms_ = frame.processing_age_ms;
  received_at_ = now;
  seen_ = true;
  output_.result_age_ms = processing_age_ms_;
  return decision;
}

const AssessmentMonitor::PathAssessment& AssessmentMonitor::output(Clock::time_point now) {
  if (seen_) {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(now - received_at_).count();
    output_.result_age_ms = elapsed_ms + processing_age_ms_;
    if (output_.result_age_ms > timeout_ms_) expire();
  }
  return output_;
}

void AssessmentMonitor::expire() {
  output_.state = PathAssessment::UNKNOWN;
  output_.reason = monitor_.on_timeout().reason;
  output_.stale = true;
  output_.distance_valid = false;
  output_.distance_m = kNaN;
  output_.reported_objects.clear();
  output_.corridor.clear();
  output_.tracks.clear();
  output_.evaluation_region_valid = false;
  output_.evaluated_range_m = 0.0;
}

}  // namespace metro_perception_ros
