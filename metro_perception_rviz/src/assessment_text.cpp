#include "metro_perception_rviz/assessment_text.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace metro_perception_rviz {
namespace {
using metro_perception_interfaces::msg::ObstacleTrack;
using metro_perception_interfaces::msg::PathAssessment;

const char* state_name(std::uint8_t state) {
  switch (state) {
    case PathAssessment::OBSTACLE:
      return "OBSTACLE";
    case PathAssessment::NO_OBSTACLE_DETECTED:
      return "NO_OBSTACLE_DETECTED";
    default:
      return "UNKNOWN";
  }
}

const char* trust_name(std::uint8_t trust) {
  switch (trust) {
    case PathAssessment::CALIBRATION_TRUST_ASSUMED:
      return "ASSUMED";
    case PathAssessment::CALIBRATION_TRUST_VERIFIED:
      return "VERIFIED";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

std::string assessment_text(const PathAssessment& assessment) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(1);
  out << "state: " << state_name(assessment.state) << '\n';
  out << "distance_m: ";
  if (assessment.distance_valid && std::isfinite(assessment.distance_m))
    out << assessment.distance_m << '\n';
  else
    out << "- (distance_valid: false)\n";
  out << "reason: " << assessment.reason << '\n';
  out << "evaluated_range_m: ";
  if (assessment.evaluation_region_valid)
    out << assessment.evaluated_range_m << '\n';
  else
    out << "- (evaluation_region_valid: false)\n";
  out << "calibration_trust: " << trust_name(assessment.calibration_trust) << '\n';
  out << "result_age_ms: " << std::setprecision(0) << assessment.result_age_ms
      << std::setprecision(1) << '\n';
  out << "stale: " << (assessment.stale ? "true" : "false") << '\n';
  out << "frame_sequence: " << assessment.frame_sequence << '\n';
  out << "reported_objects: " << assessment.reported_objects.size() << '\n';

  std::vector<const ObstacleTrack*> confirmed;
  for (const auto& track : assessment.tracks)
    if (track.confirmed) confirmed.push_back(&track);
  std::sort(confirmed.begin(), confirmed.end(), [](const ObstacleTrack* a, const ObstacleTrack* b) {
    return a->distance_m < b->distance_m;
  });
  out << "confirmed_tracks:" << (confirmed.empty() ? " []" : "") << '\n';
  for (const auto* track : confirmed) {
    const auto& size = track->bbox.size;
    out << "- id " << track->track_id << ": " << track->distance_m << " m, " << size.x << " x "
        << size.y << " x " << size.z << " m, hits " << track->hits
        << (track->coasting ? ", predicted" : "") << '\n';
  }
  return out.str();
}

}  // namespace metro_perception_rviz
