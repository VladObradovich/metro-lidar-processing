#pragma once

#include <chrono>
#include <cstdint>

#include "metro_perception_core/temporal_monitor.hpp"
#include "metro_perception_interfaces/msg/frame_analysis.hpp"
#include "metro_perception_interfaces/msg/path_assessment.hpp"
#include "metro_perception_ros/frame_key_gate.hpp"

namespace metro_perception_ros {

// ROS-free core of obstacle_monitor_node: gates FrameAnalysis keys, applies the shared
// TemporalMonitor decision and the steady-clock watchdog. Time is passed in for testing.
class AssessmentMonitor {
 public:
  using Clock = std::chrono::steady_clock;
  using FrameAnalysis = metro_perception_interfaces::msg::FrameAnalysis;
  using PathAssessment = metro_perception_interfaces::msg::PathAssessment;

  explicit AssessmentMonitor(double timeout_s);

  // Accepted analyses replace the published result; rejected ones only update diagnostics.
  GateDecision on_analysis(const FrameAnalysis& frame, Clock::time_point now);

  // Result to publish at `now`. After the timeout it is UNKNOWN and stale, keeps the key and
  // header of the last accepted observation and carries no objects, corridor or distance.
  const PathAssessment& output(Clock::time_point now);

 private:
  void expire();

  double timeout_ms_;
  FrameKeyGate gate_;
  metro_perception_core::TemporalMonitor monitor_;
  PathAssessment output_;
  bool seen_{false};
  Clock::time_point received_at_{};
  double processing_age_ms_{0};
};

}  // namespace metro_perception_ros
