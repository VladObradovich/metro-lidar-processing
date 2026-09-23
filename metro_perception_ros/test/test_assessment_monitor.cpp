#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "metro_perception_ros/assessment_monitor.hpp"

using metro_perception_interfaces::msg::FrameAnalysis;
using metro_perception_interfaces::msg::PathAssessment;
using metro_perception_ros::AssessmentMonitor;
using namespace std::chrono_literals;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const AssessmentMonitor::Clock::time_point t0{std::chrono::seconds(1000)};

struct Spec {
  std::string source{"src-a"};
  std::uint64_t session{0};
  std::uint64_t sequence{1};
  std::int32_t stamp_sec{100};
  std::uint8_t status{FrameAnalysis::OK};
  std::uint8_t trust{FrameAnalysis::CALIBRATION_TRUST_VERIFIED};
  std::vector<std::pair<double, bool>> candidates;  // distance_m, distance_valid
  bool region_valid{true};
  double range_m{40};
  double processing_age_ms{0};
  std::string reason{"OK"};
};

FrameAnalysis frame(const Spec& s) {
  FrameAnalysis f;
  f.header.frame_id = "lidar_assumed";
  f.header.stamp.sec = s.stamp_sec;
  f.source_instance_id = s.source;
  f.session_id = s.session;
  f.frame_sequence = s.sequence;
  f.processing_status = s.status;
  f.reason = s.reason;
  f.calibration_trust = s.trust;
  f.evaluation_region_valid = s.region_valid;
  f.evaluated_range_m = s.range_m;
  f.processing_age_ms = s.processing_age_ms;
  std::uint64_t id = 0;
  for (const auto& [distance, valid] : s.candidates) {
    auto& c = f.candidates.emplace_back();
    c.candidate_id = ++id;
    c.distance_m = distance;
    c.distance_valid = valid;
    c.bbox.center.position.x = std::isfinite(distance) ? distance : 1.0;
    c.bbox.size.x = c.bbox.size.y = c.bbox.size.z = 0.5;
  }
  auto& segment = f.corridor.emplace_back();
  segment.start.x = 2;
  segment.end.x = s.range_m;
  segment.width_m = 4;
  segment.height_m = 3.5;
  segment.geometry_valid = true;
  segment.coverage_valid = s.region_valid;
  return f;
}

void expect_key(const PathAssessment& out, const FrameAnalysis& in) {
  EXPECT_EQ(out.header, in.header);
  EXPECT_EQ(out.source_instance_id, in.source_instance_id);
  EXPECT_EQ(out.session_id, in.session_id);
  EXPECT_EQ(out.frame_sequence, in.frame_sequence);
}

void expect_no_distance(const PathAssessment& out) {
  EXPECT_FALSE(out.distance_valid);
  EXPECT_TRUE(std::isnan(out.distance_m)) << out.distance_m;
}

// Copied objects must be identical; NaN distances compare equal to themselves here.
void expect_same_objects(
    const std::vector<metro_perception_interfaces::msg::ObstacleCandidate>& a,
    const std::vector<metro_perception_interfaces::msg::ObstacleCandidate>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    auto x = a[i], y = b[i];
    EXPECT_EQ(std::isnan(x.distance_m), std::isnan(y.distance_m)) << i;
    if (std::isnan(x.distance_m)) x.distance_m = y.distance_m = 0;
    EXPECT_EQ(x, y) << i;
  }
}

// Fresh result of `in`: objects and corridor must be exactly those of that frame.
void expect_fresh(const PathAssessment& out, const FrameAnalysis& in, std::uint8_t state,
                  const std::string& reason) {
  expect_key(out, in);
  EXPECT_EQ(out.state, state) << out.reason;
  EXPECT_EQ(out.reason, reason);
  EXPECT_FALSE(out.stale);
  expect_same_objects(out.reported_objects, in.candidates);
  EXPECT_EQ(out.corridor, in.corridor);
  EXPECT_EQ(out.evaluation_region_valid, in.evaluation_region_valid);
  EXPECT_EQ(out.evaluated_range_m, in.evaluated_range_m);
  EXPECT_EQ(out.calibration_trust, in.calibration_trust);
}

void expect_stale(const PathAssessment& out, const FrameAnalysis& last) {
  expect_key(out, last);  // Heartbeat keeps the last observation, never a new stamp.
  EXPECT_EQ(out.state, PathAssessment::UNKNOWN);
  EXPECT_EQ(out.reason, "INPUT_PAUSED_OR_STOPPED");
  EXPECT_TRUE(out.stale);
  expect_no_distance(out);
  EXPECT_TRUE(out.reported_objects.empty());
  EXPECT_TRUE(out.corridor.empty());
  EXPECT_FALSE(out.evaluation_region_valid);
  EXPECT_EQ(out.evaluated_range_m, 0.0);
}
}  // namespace

TEST(AssessmentMonitor, WaitsForInputAsStaleUnknown) {
  AssessmentMonitor monitor(0.5);
  const auto& out = monitor.output(t0);
  EXPECT_EQ(out.state, PathAssessment::UNKNOWN);
  EXPECT_EQ(out.reason, "WAITING_FOR_INPUT");
  EXPECT_TRUE(out.stale);
  expect_no_distance(out);
  EXPECT_THROW(AssessmentMonitor{0.0}, std::invalid_argument);
  EXPECT_THROW(AssessmentMonitor{kNaN}, std::invalid_argument);
}

TEST(AssessmentMonitor, ReportsAllCandidatesAndCorridorOfTheDecidingFrame) {
  AssessmentMonitor monitor(0.5);
  // The candidate without a usable distance is reported for diagnostics, not ranged.
  const auto in = frame({.candidates = {{30.0, true}, {kNaN, true}, {12.5, true}, {5.0, false}}});
  ASSERT_TRUE(monitor.on_analysis(in, t0).accepted);
  const auto& out = monitor.output(t0);
  expect_fresh(out, in, PathAssessment::OBSTACLE, "OBSTACLE_CANDIDATE");
  EXPECT_TRUE(out.distance_valid);
  EXPECT_DOUBLE_EQ(out.distance_m, 12.5);
  EXPECT_EQ(out.reported_objects.size(), 4u);
}

TEST(AssessmentMonitor, CandidatesWithoutUsableDistanceAreUnknownNotClear) {
  AssessmentMonitor monitor(0.5);
  const auto in = frame({.candidates = {{kNaN, true}, {0.0, true}, {-4.0, true}, {9.0, false}}});
  monitor.on_analysis(in, t0);
  const auto& out = monitor.output(t0);
  expect_fresh(out, in, PathAssessment::UNKNOWN, "CANDIDATE_DISTANCE_INVALID");
  expect_no_distance(out);
}

TEST(AssessmentMonitor, ClearPathNeedsVerifiedTrustAndUsableRegion) {
  AssessmentMonitor monitor(0.5);
  const auto clear = frame({.sequence = 1});
  monitor.on_analysis(clear, t0);
  expect_fresh(monitor.output(t0), clear, PathAssessment::NO_OBSTACLE_DETECTED,
               "NO_CANDIDATE_IN_EVALUATED_REGION");
  expect_no_distance(monitor.output(t0));

  const auto assumed = frame({.sequence = 2, .trust = FrameAnalysis::CALIBRATION_TRUST_ASSUMED});
  monitor.on_analysis(assumed, t0);
  expect_fresh(monitor.output(t0), assumed, PathAssessment::UNKNOWN,
               "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR");

  const auto hidden = frame({.sequence = 3, .region_valid = false, .range_m = 0});
  monitor.on_analysis(hidden, t0);
  expect_fresh(monitor.output(t0), hidden, PathAssessment::UNKNOWN, "CORRIDOR_UNOBSERVABLE");
}

TEST(AssessmentMonitor, AnalysisFailureClearsObjectsAndCorridor) {
  AssessmentMonitor monitor(0.5);
  auto failed = frame({.status = FrameAnalysis::INVALID_GEOMETRY,
                       .candidates = {{10.0, true}},
                       .reason = "GROUND_UNSUPPORTED"});
  monitor.on_analysis(failed, t0);
  const auto& out = monitor.output(t0);
  expect_key(out, failed);
  EXPECT_EQ(out.state, PathAssessment::UNKNOWN);
  EXPECT_EQ(out.reason, "GROUND_UNSUPPORTED");
  EXPECT_FALSE(out.stale);
  expect_no_distance(out);
  EXPECT_TRUE(out.reported_objects.empty());
  EXPECT_TRUE(out.corridor.empty());
  EXPECT_FALSE(out.evaluation_region_valid);
  EXPECT_EQ(out.evaluated_range_m, 0.0);

  const auto bogus = frame({.sequence = 2, .status = 99, .candidates = {{10.0, true}}});
  monitor.on_analysis(bogus, t0);
  EXPECT_EQ(monitor.output(t0).reason, "INVALID_PROCESSING_STATUS");
  EXPECT_TRUE(monitor.output(t0).reported_objects.empty());
}

TEST(AssessmentMonitor, DuplicateLateAndBackwardFramesKeepTheLastValidResult) {
  AssessmentMonitor monitor(0.5);
  const auto obstacle = frame({.sequence = 5, .stamp_sec = 105, .candidates = {{20.0, true}}});
  monitor.on_analysis(obstacle, t0);
  const std::vector<std::pair<Spec, std::string>> rejected = {
      {{.sequence = 5, .stamp_sec = 106}, "DUPLICATE_OR_LATE_SEQUENCE"},
      {{.sequence = 4, .stamp_sec = 106}, "DUPLICATE_OR_LATE_SEQUENCE"},
      {{.sequence = 6, .stamp_sec = 104}, "STAMP_WENT_BACKWARDS"},
  };
  std::uint64_t count = 0;
  for (const auto& [spec, reason] : rejected) {
    SCOPED_TRACE(reason);
    EXPECT_FALSE(monitor.on_analysis(frame(spec), t0 + 100ms).accepted);
    const auto& out = monitor.output(t0 + 100ms);
    expect_fresh(out, obstacle, PathAssessment::OBSTACLE, "OBSTACLE_CANDIDATE");
    EXPECT_DOUBLE_EQ(out.distance_m, 20.0);
    EXPECT_EQ(out.rejected_analyses, ++count);
    EXPECT_EQ(out.last_rejection_reason, reason);
  }
  // A rejected message does not refresh the result age: the watchdog still fires on time.
  expect_stale(monitor.output(t0 + 501ms), obstacle);
  // A forward stamp jump inside the session is a valid frame.
  const auto jump = frame({.sequence = 6, .stamp_sec = 900});
  ASSERT_TRUE(monitor.on_analysis(jump, t0 + 600ms).accepted);
  expect_fresh(monitor.output(t0 + 600ms), jump, PathAssessment::NO_OBSTACLE_DETECTED,
               "NO_CANDIDATE_IN_EVALUATED_REGION");
}

TEST(AssessmentMonitor, NewSessionResetsOldSessionIsRejected) {
  AssessmentMonitor monitor(0.5);
  monitor.on_analysis(frame({.session = 2, .sequence = 40, .stamp_sec = 400}), t0);
  const auto reset = frame({.session = 3, .sequence = 1, .stamp_sec = 10});
  const auto decision = monitor.on_analysis(reset, t0 + 10ms);
  EXPECT_TRUE(decision.accepted);
  EXPECT_TRUE(decision.reset);
  expect_fresh(monitor.output(t0 + 10ms), reset, PathAssessment::NO_OBSTACLE_DETECTED,
               "NO_CANDIDATE_IN_EVALUATED_REGION");
  EXPECT_FALSE(monitor
                   .on_analysis(frame({.session = 2, .sequence = 41, .candidates = {{3.0, true}}}),
                                t0 + 20ms)
                   .accepted);
  const auto& out = monitor.output(t0 + 20ms);
  expect_fresh(out, reset, PathAssessment::NO_OBSTACLE_DETECTED,
               "NO_CANDIDATE_IN_EVALUATED_REGION");
  EXPECT_EQ(out.last_rejection_reason, "OLD_SESSION");
}

TEST(AssessmentMonitor, RetiredSourceCannotComeBack) {
  AssessmentMonitor monitor(0.5);
  monitor.on_analysis(frame({.source = "a", .sequence = 10, .candidates = {{8.0, true}}}), t0);
  const auto b = frame({.source = "b", .sequence = 1, .stamp_sec = 50});
  ASSERT_TRUE(monitor.on_analysis(b, t0 + 10ms).accepted);
  expect_fresh(monitor.output(t0 + 10ms), b, PathAssessment::NO_OBSTACLE_DETECTED,
               "NO_CANDIDATE_IN_EVALUATED_REGION");
  for (const auto& spec : {Spec{.source = "a", .sequence = 11, .candidates = {{8.0, true}}},
                           Spec{.source = "a", .session = 7, .sequence = 1}}) {
    EXPECT_FALSE(monitor.on_analysis(frame(spec), t0 + 20ms).accepted);
    const auto& out = monitor.output(t0 + 20ms);
    expect_fresh(out, b, PathAssessment::NO_OBSTACLE_DETECTED, "NO_CANDIDATE_IN_EVALUATED_REGION");
    EXPECT_EQ(out.last_rejection_reason, "RETIRED_SOURCE");
  }
}

TEST(AssessmentMonitor, InvalidKeyNeitherReplacesSourceNorRefreshesAge) {
  AssessmentMonitor monitor(0.5);
  const auto valid = frame({.sequence = 3, .candidates = {{15.0, true}}});
  monitor.on_analysis(valid, t0);
  const std::vector<std::pair<FrameAnalysis, std::string>> invalid = [&] {
    auto no_source = frame({.source = "", .sequence = 4});
    auto no_stamp = frame({.source = "b", .sequence = 4});
    no_stamp.header.stamp.sec = 0;
    auto bad_nanos = frame({.source = "b", .sequence = 4});
    bad_nanos.header.stamp.nanosec = 1000000000u;
    auto no_sequence = frame({.source = "b", .sequence = 0});
    return std::vector<std::pair<FrameAnalysis, std::string>>{{no_source, "INVALID_SOURCE_ID"},
                                                              {no_stamp, "INVALID_STAMP"},
                                                              {bad_nanos, "INVALID_STAMP"},
                                                              {no_sequence, "INVALID_SEQUENCE"}};
  }();
  for (const auto& [message, reason] : invalid) {
    SCOPED_TRACE(reason);
    EXPECT_FALSE(monitor.on_analysis(message, t0 + 400ms).accepted);
    const auto& out = monitor.output(t0 + 400ms);
    expect_fresh(out, valid, PathAssessment::OBSTACLE, "OBSTACLE_CANDIDATE");
    EXPECT_EQ(out.last_rejection_reason, reason);
  }
  EXPECT_EQ(monitor.output(t0 + 400ms).rejected_analyses, invalid.size());
  expect_stale(monitor.output(t0 + 501ms), valid);
  // The original source was never displaced by the invalid keys.
  const auto next = frame({.sequence = 4, .candidates = {{14.0, true}}});
  ASSERT_TRUE(monitor.on_analysis(next, t0 + 600ms).accepted);
  expect_fresh(monitor.output(t0 + 600ms), next, PathAssessment::OBSTACLE, "OBSTACLE_CANDIDATE");
}

TEST(AssessmentMonitor, TimeoutClearsResultAndFreshFrameRestoresIt) {
  AssessmentMonitor monitor(0.5);
  const auto obstacle = frame({.sequence = 1, .candidates = {{11.0, true}}});
  monitor.on_analysis(obstacle, t0);
  EXPECT_FALSE(monitor.output(t0 + 500ms).stale);
  for (const auto at : {t0 + 501ms, t0 + 700ms, t0 + 5s}) {
    const auto& out = monitor.output(at);
    expect_stale(out, obstacle);
    EXPECT_GT(out.result_age_ms, 500.0);
  }
  const auto fresh = frame({.sequence = 2, .stamp_sec = 106, .candidates = {{10.0, true}}});
  ASSERT_TRUE(monitor.on_analysis(fresh, t0 + 6s).accepted);
  const auto& out = monitor.output(t0 + 6s);
  expect_fresh(out, fresh, PathAssessment::OBSTACLE, "OBSTACLE_CANDIDATE");
  EXPECT_DOUBLE_EQ(out.distance_m, 10.0);
  EXPECT_LT(out.result_age_ms, 1.0);
}

TEST(AssessmentMonitor, ProcessingAgeCountsTowardsTimeout) {
  AssessmentMonitor monitor(0.5);
  const auto slow = frame({.sequence = 1, .processing_age_ms = 450});
  monitor.on_analysis(slow, t0);
  EXPECT_FALSE(monitor.output(t0 + 40ms).stale);
  expect_stale(monitor.output(t0 + 60ms), slow);
  // Already older than the timeout on arrival: accepted, but never published as fresh.
  const auto late = frame({.sequence = 2, .candidates = {{9.0, true}}, .processing_age_ms = 600});
  EXPECT_TRUE(monitor.on_analysis(late, t0 + 100ms).accepted);
  expect_stale(monitor.output(t0 + 100ms), late);
}

TEST(AssessmentMonitor, UnknownProcessingAgeIsRejectedImmediately) {
  AssessmentMonitor monitor(0.5);
  const auto rejected_first = frame({.sequence = 1, .processing_age_ms = kNaN});
  EXPECT_FALSE(monitor.on_analysis(rejected_first, t0).accepted);
  EXPECT_EQ(monitor.output(t0).reason, "WAITING_FOR_INPUT");
  EXPECT_TRUE(monitor.output(t0).stale);

  const auto valid = frame({.sequence = 2, .candidates = {{12.0, true}}});
  ASSERT_TRUE(monitor.on_analysis(valid, t0).accepted);
  std::uint64_t count = 1;
  for (const double age : {kNaN, std::numeric_limits<double>::infinity(), -1.0}) {
    SCOPED_TRACE(age);
    const auto bad = frame({.sequence = 3, .processing_age_ms = age});
    const auto decision = monitor.on_analysis(bad, t0 + 100ms);
    EXPECT_FALSE(decision.accepted);
    EXPECT_EQ(std::string(decision.reason), "INVALID_PROCESSING_AGE");
    // The very next publication still shows the previous valid frame, never the bad one.
    const auto& out = monitor.output(t0 + 100ms);
    expect_fresh(out, valid, PathAssessment::OBSTACLE, "OBSTACLE_CANDIDATE");
    EXPECT_EQ(out.rejected_analyses, ++count);
    EXPECT_EQ(out.last_rejection_reason, "INVALID_PROCESSING_AGE");
  }
  // The rejection did not refresh the age or consume the sequence number.
  expect_stale(monitor.output(t0 + 501ms), valid);
  EXPECT_TRUE(monitor.on_analysis(frame({.sequence = 3}), t0 + 600ms).accepted);
}
