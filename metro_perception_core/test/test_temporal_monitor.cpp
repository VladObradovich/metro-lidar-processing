#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "metro_perception_core/temporal_monitor.hpp"
using namespace metro_perception_core;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Case {
  std::string name;
  AnalysisStatus status;
  CalibrationTrust trust;
  bool region_valid;
  double range_m;
  std::vector<std::pair<bool, double>> candidates;  // distance_valid, distance_m
  State state;
  std::string reason;
  double distance_m;  // NaN when the decision has no distance.
  std::string frame_reason{"OK"};
};

FrameResult frame_of(const Case& c) {
  FrameResult frame;
  frame.status = c.status;
  frame.reason = c.frame_reason;
  frame.calibration_trust = c.trust;
  frame.evaluation_region_valid = c.region_valid;
  frame.evaluated_range_m = c.range_m;
  std::uint64_t id = 0;
  for (const auto& [valid, distance] : c.candidates) {
    ObstacleCandidate candidate;
    candidate.id = ++id;
    candidate.distance_valid = valid;
    candidate.distance_m = distance;
    frame.candidates.push_back(candidate);
  }
  return frame;
}

// Invariants that must hold for every decision, whatever the input.
void expect_consistent(const FrameResult& frame, const Assessment& a, const std::string& name) {
  SCOPED_TRACE(name);
  EXPECT_EQ(a.calibration_trust, frame.calibration_trust);
  if (a.state == State::OBSTACLE) {
    EXPECT_TRUE(a.distance_valid);
    EXPECT_TRUE(std::isfinite(a.distance_m) && a.distance_m > 0) << a.distance_m;
  } else {
    EXPECT_FALSE(a.distance_valid);
    EXPECT_TRUE(std::isnan(a.distance_m)) << a.distance_m;
  }
  if (a.state == State::NO_OBSTACLE_DETECTED) {
    EXPECT_EQ(frame.status, AnalysisStatus::OK);
    EXPECT_EQ(frame.calibration_trust, CalibrationTrust::VERIFIED);
    EXPECT_TRUE(frame.candidates.empty());
    EXPECT_TRUE(frame.evaluation_region_valid && std::isfinite(frame.evaluated_range_m) &&
                frame.evaluated_range_m > 0);
  }
}
}  // namespace

TEST(TemporalMonitorTable, DecisionFollowsNearestUsableCandidate) {
  using S = AnalysisStatus;
  using T = CalibrationTrust;
  // clang-format off
  const std::vector<Case> cases = {
      {"one candidate", S::OK, T::VERIFIED, true, 40, {{true, 12.0}},
       State::OBSTACLE, "OBSTACLE_CANDIDATE", 12.0},
      {"nearest of several", S::OK, T::VERIFIED, true, 40, {{true, 30.0}, {true, 12.5}, {true, 20}},
       State::OBSTACLE, "OBSTACLE_CANDIDATE", 12.5},
      {"NaN distance", S::OK, T::VERIFIED, true, 40, {{true, kNaN}},
       State::UNKNOWN, "CANDIDATE_DISTANCE_INVALID", kNaN},
      {"infinite distance", S::OK, T::VERIFIED, true, 40, {{true, kInf}},
       State::UNKNOWN, "CANDIDATE_DISTANCE_INVALID", kNaN},
      {"zero distance", S::OK, T::VERIFIED, true, 40, {{true, 0.0}},
       State::UNKNOWN, "CANDIDATE_DISTANCE_INVALID", kNaN},
      {"negative distance", S::OK, T::VERIFIED, true, 40, {{true, -3.0}},
       State::UNKNOWN, "CANDIDATE_DISTANCE_INVALID", kNaN},
      {"distance flagged invalid", S::OK, T::VERIFIED, true, 40, {{false, 10.0}},
       State::UNKNOWN, "CANDIDATE_DISTANCE_INVALID", kNaN},
      {"NaN beside usable", S::OK, T::VERIFIED, true, 40, {{true, kNaN}, {true, 15.0}},
       State::OBSTACLE, "OBSTACLE_CANDIDATE", 15.0},
      {"closer unflagged ignored", S::OK, T::VERIFIED, true, 40, {{false, 5.0}, {true, 15.0}},
       State::OBSTACLE, "OBSTACLE_CANDIDATE", 15.0},
      {"negative beside usable", S::OK, T::VERIFIED, true, 40, {{true, -1.0}, {true, 9.0}},
       State::OBSTACLE, "OBSTACLE_CANDIDATE", 9.0},
      {"verified clear", S::OK, T::VERIFIED, true, 40, {},
       State::NO_OBSTACLE_DETECTED, "NO_CANDIDATE_IN_EVALUATED_REGION", kNaN},
      {"verified unobservable", S::OK, T::VERIFIED, false, 0, {},
       State::UNKNOWN, "CORRIDOR_UNOBSERVABLE", kNaN},
      {"verified zero range", S::OK, T::VERIFIED, true, 0, {},
       State::UNKNOWN, "EVALUATION_REGION_INVALID", kNaN},
      {"verified NaN range", S::OK, T::VERIFIED, true, kNaN, {},
       State::UNKNOWN, "EVALUATION_REGION_INVALID", kNaN},
      {"verified rolling background", S::OK, T::VERIFIED, false, 40, {},
       State::UNKNOWN, "BACKGROUND_CANNOT_CONFIRM_CLEAR", kNaN, "BACKGROUND_CANNOT_CONFIRM_CLEAR"},
      {"assumed clear", S::OK, T::ASSUMED, true, 40, {},
       State::UNKNOWN, "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR", kNaN},
      {"assumed unobservable", S::OK, T::ASSUMED, false, 0, {},
       State::UNKNOWN, "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR", kNaN},
      {"assumed obstacle", S::OK, T::ASSUMED, false, 40, {{true, 8.0}},
       State::OBSTACLE, "OBSTACLE_WITH_ASSUMED_CALIBRATION", 8.0},
      {"assumed NaN candidate", S::OK, T::ASSUMED, true, 40, {{true, kNaN}},
       State::UNKNOWN, "CANDIDATE_DISTANCE_INVALID", kNaN},
      {"unknown trust clear", S::OK, T::UNKNOWN, true, 40, {},
       State::UNKNOWN, "CALIBRATION_TRUST_UNKNOWN", kNaN},
      {"unknown trust obstacle", S::OK, T::UNKNOWN, true, 40, {{true, 7.0}},
       State::OBSTACLE, "OBSTACLE_WITH_UNKNOWN_CALIBRATION", 7.0},
      {"bad input keeps its reason", S::BAD_INPUT, T::VERIFIED, true, 40, {{true, 10.0}},
       State::UNKNOWN, "INVALID_TIMESTAMP", kNaN, "INVALID_TIMESTAMP"},
      {"tf unavailable", S::TF_UNAVAILABLE, T::UNKNOWN, false, 0, {},
       State::UNKNOWN, "TF_UNAVAILABLE", kNaN, "TF_UNAVAILABLE"},
      {"invalid geometry with candidates", S::INVALID_GEOMETRY, T::VERIFIED, true, 40,
       {{true, 5.0}}, State::UNKNOWN, "GROUND_UNSUPPORTED", kNaN, "GROUND_UNSUPPORTED"},
      {"failure without reason", S::NOT_IMPLEMENTED, T::VERIFIED, true, 40, {},
       State::UNKNOWN, "ANALYSIS_FAILED", kNaN, ""},
  };
  // clang-format on
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    const auto frame = frame_of(c);
    const auto a = TemporalMonitor().update(frame, 1);
    EXPECT_EQ(a.state, c.state);
    EXPECT_EQ(a.reason, c.reason);
    if (std::isnan(c.distance_m)) {
      EXPECT_TRUE(std::isnan(a.distance_m));
    } else {
      EXPECT_DOUBLE_EQ(a.distance_m, c.distance_m);
    }
    expect_consistent(frame, a, c.name);
  }
}

TEST(TemporalMonitorTable, UnusableCandidateNeverReportsClearPath) {
  const std::vector<std::pair<bool, double>> values = {{true, kNaN},  {true, kInf}, {true, -kInf},
                                                       {true, 0.0},   {true, -2.0}, {false, 4.0},
                                                       {false, kNaN}, {true, 6.0},  {true, 3.0}};
  const std::vector<CalibrationTrust> trusts = {
      CalibrationTrust::UNKNOWN, CalibrationTrust::ASSUMED, CalibrationTrust::VERIFIED};
  for (const auto trust : trusts) {
    for (std::size_t i = 0; i < values.size(); ++i) {
      for (std::size_t j = i; j < values.size(); ++j) {
        Case c{"grid", AnalysisStatus::OK,     trust,          true,
               40,     {values[i], values[j]}, State::UNKNOWN, "",
               kNaN};
        const auto frame = frame_of(c);
        const auto a = TemporalMonitor().update(frame, 1);
        const std::string name = std::to_string(i) + "," + std::to_string(j);
        EXPECT_NE(a.state, State::NO_OBSTACLE_DETECTED) << name;
        expect_consistent(frame, a, name);
        const bool any_usable = usable_distance(values[i].first, values[i].second) ||
                                usable_distance(values[j].first, values[j].second);
        EXPECT_EQ(a.state == State::OBSTACLE, any_usable) << name;
      }
    }
  }
}

TEST(TemporalMonitorTable, TimeoutIsUnknownWithoutDistance) {
  const auto a = TemporalMonitor().on_timeout();
  EXPECT_EQ(a.state, State::UNKNOWN);
  EXPECT_EQ(a.reason, "INPUT_PAUSED_OR_STOPPED");
  EXPECT_FALSE(a.distance_valid);
  EXPECT_TRUE(std::isnan(a.distance_m));
}
