#pragma once

#include <string>

#include "metro_perception_interfaces/msg/path_assessment.hpp"

namespace metro_perception_rviz {

// The assessment as `key: value` lines in the manner of `ros2 topic echo`, with
// enums named and only the fields an operator reads: state, distance, reason,
// checked range, age and tracks.
std::string assessment_text(const metro_perception_interfaces::msg::PathAssessment& assessment);

}  // namespace metro_perception_rviz
