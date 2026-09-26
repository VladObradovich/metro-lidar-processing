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

TEST(TemporalMonitorTable, AssumedClearNeedsTheOptionAndEvaluatedRange) {
  // With assumed_clear an empty, well evaluated region is clear and keeps trust ASSUMED; every
  // other reason for UNKNOWN is unchanged, and trust UNKNOWN is never clear.
  using S = AnalysisStatus;
  using T = CalibrationTrust;
  TemporalConfig config;
  config.assumed_clear = true;
  config.assumed_clear_min_range_m = 50;
  // clang-format off
  const std::vector<Case> cases = {
      {"assumed clear", S::OK, T::ASSUMED, true, 80, {},
       State::NO_OBSTACLE_DETECTED, "NO_CANDIDATE_ASSUMED_CALIBRATION", kNaN},
      {"assumed short range", S::OK, T::ASSUMED, true, 40, {},
       State::UNKNOWN, "EVALUATED_RANGE_TOO_SHORT", kNaN},
      {"assumed unobservable", S::OK, T::ASSUMED, false, 0, {},
       State::UNKNOWN, "CORRIDOR_UNOBSERVABLE", kNaN},
      {"assumed rolling background", S::OK, T::ASSUMED, false, 80, {},
       State::UNKNOWN, "BACKGROUND_CANNOT_CONFIRM_CLEAR", kNaN, "BACKGROUND_CANNOT_CONFIRM_CLEAR"},
      {"assumed unconfirmed candidate", S::OK, T::ASSUMED, true, 80, {{true, 30.0}},
       State::UNKNOWN, "CANDIDATE_UNCONFIRMED", kNaN},
      {"assumed invalid geometry", S::INVALID_GEOMETRY, T::ASSUMED, true, 80, {},
       State::UNKNOWN, "GROUND_UNSUPPORTED", kNaN, "GROUND_UNSUPPORTED"},
      {"unknown trust", S::OK, T::UNKNOWN, true, 80, {},
       State::UNKNOWN, "CALIBRATION_TRUST_UNKNOWN", kNaN},
      {"verified short range", S::OK, T::VERIFIED, true, 40, {},
       State::NO_OBSTACLE_DETECTED, "NO_CANDIDATE_IN_EVALUATED_REGION", kNaN},
  };
  // clang-format on
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    TemporalConfig single = config;
    single.confirm_hits = single.confirm_window = 2;  // One sighting stays unconfirmed.
    const auto a = TemporalMonitor(single).update(frame_of(c), 1);
    EXPECT_EQ(a.state, c.state);
    EXPECT_EQ(a.reason, c.reason);
    EXPECT_EQ(a.calibration_trust, c.trust);
    EXPECT_TRUE(std::isnan(a.distance_m));
  }
  config.assumed_clear_min_range_m = 0;
  EXPECT_THROW(config.validate(), std::invalid_argument);
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

namespace {
struct Seen {
  double x, y;
  std::uint8_t channels{ObstacleCandidate::kMotion};
  std::uint32_t points{100};
};
FrameResult tracked_frame(const std::vector<Seen>& seen, double ego_speed = 0.0,
                          bool ego_valid = true) {
  FrameResult frame;
  frame.status = AnalysisStatus::OK;
  frame.reason = "OK";
  frame.calibration_trust = CalibrationTrust::VERIFIED;
  frame.evaluation_region_valid = true;
  frame.evaluated_range_m = 80;
  frame.ego_motion_valid = ego_valid;
  frame.ego_speed_mps = ego_speed;
  std::uint64_t id = 0;
  for (const auto& s : seen) {
    ObstacleCandidate c;
    c.id = ++id;
    c.center = {s.x, s.y, 0.0};
    c.size = {0.5, 0.5, 1.7};
    c.distance_m = s.x - 0.25;
    c.distance_valid = true;
    c.channels = s.channels;
    c.support_points = s.points;
    frame.candidates.push_back(c);
  }
  return frame;
}
constexpr std::int64_t kFrameNs = 100000000;  // 10 Hz.
TemporalConfig rule(std::size_t hits, std::size_t window, std::size_t release = 1) {
  TemporalConfig config;
  config.confirm_hits = config.gauge_confirm_hits = hits;
  config.confirm_window = config.gauge_confirm_window = window;
  config.release_misses = release;
  return config;
}
}  // namespace

TEST(Tracker, DefaultRuleIsThePerFrameDecisionOverASequence) {
  const std::vector<std::vector<Seen>> sequence = {
      {{30, 0}}, {}, {{29, 0.2}, {12, -0.5}}, {{11, -0.5}}, {}, {}, {{40, 0}}, {{5, 0.1}}};
  TemporalMonitor persistent;
  for (std::size_t i = 0; i < sequence.size(); ++i) {
    const auto frame = tracked_frame(sequence[i], 10.0);
    const auto a = persistent.update(frame, (i + 1) * kFrameNs);
    const auto fresh = TemporalMonitor().update(frame, 1);
    SCOPED_TRACE(i);
    EXPECT_EQ(a.state, fresh.state);
    EXPECT_EQ(a.reason, fresh.reason);
    EXPECT_EQ(a.distance_valid, fresh.distance_valid);
    if (a.distance_valid) EXPECT_DOUBLE_EQ(a.distance_m, fresh.distance_m);
  }
}

TEST(Tracker, SingleFrameSpikeIsNeverAnObstacleNorAClearPath) {
  TemporalMonitor monitor(rule(2, 3));
  auto a = monitor.update(tracked_frame({{30, 0}}), kFrameNs);
  EXPECT_EQ(a.state, State::UNKNOWN);
  EXPECT_EQ(a.reason, "CANDIDATE_UNCONFIRMED");
  // Recent unconfirmed evidence still forbids a clear path until its window has passed.
  for (int i = 2; i <= 3; ++i) {
    a = monitor.update(tracked_frame({}), i * kFrameNs);
    EXPECT_EQ(a.state, State::UNKNOWN) << i;
    EXPECT_EQ(a.reason, "CANDIDATE_UNCONFIRMED") << i;
  }
  a = monitor.update(tracked_frame({}), 4 * kFrameNs);
  EXPECT_EQ(a.state, State::NO_OBSTACLE_DETECTED);
  EXPECT_TRUE(a.tracks.empty());
}

TEST(Tracker, ObjectFixedInTheWorldIsConfirmedWhileApproaching) {
  TemporalMonitor monitor(rule(2, 3));
  const auto first = monitor.update(tracked_frame({{50, 0}}, 10.0), kFrameNs);
  EXPECT_EQ(first.state, State::UNKNOWN);
  ASSERT_EQ(first.tracks.size(), 1u);
  const auto id = first.tracks.front().id;
  const auto second = monitor.update(tracked_frame({{49, 0.05}}, 10.0), 2 * kFrameNs);
  ASSERT_EQ(second.state, State::OBSTACLE) << second.reason;
  EXPECT_EQ(second.reason, "OBSTACLE_CANDIDATE");
  EXPECT_DOUBLE_EQ(second.distance_m, 48.75);
  ASSERT_EQ(second.tracks.size(), 1u);
  EXPECT_EQ(second.tracks.front().id, id);
  EXPECT_TRUE(second.tracks.front().confirmed);
}

TEST(Tracker, ObjectCrossingTheRouteKeepsItsTrack) {
  TemporalMonitor monitor(rule(3, 3));
  std::uint64_t id = 0;
  for (int i = 0; i < 3; ++i) {
    const auto a = monitor.update(tracked_frame({{20, -1.0 + 0.3 * i}}), (i + 1) * kFrameNs);
    ASSERT_EQ(a.tracks.size(), 1u);
    if (i == 0) id = a.tracks.front().id;
    EXPECT_EQ(a.tracks.front().id, id);
    EXPECT_EQ(a.state, i == 2 ? State::OBSTACLE : State::UNKNOWN) << i;
  }
}

TEST(Tracker, ConfirmedObstacleCoastsThenIsReleased) {
  TemporalMonitor monitor(rule(2, 3, 3));
  monitor.update(tracked_frame({{40, 0}}, 10.0), kFrameNs);
  ASSERT_EQ(monitor.update(tracked_frame({{39, 0}}, 10.0), 2 * kFrameNs).state, State::OBSTACLE);
  for (int miss = 1; miss <= 2; ++miss) {
    const auto a = monitor.update(tracked_frame({}, 10.0), (2 + miss) * kFrameNs);
    ASSERT_EQ(a.state, State::OBSTACLE) << miss;
    EXPECT_EQ(a.reason, "OBSTACLE_COASTING");
    EXPECT_NEAR(a.distance_m, 38.75 - miss * 1.0, 1e-9);  // Predicted by the ego travel.
    EXPECT_TRUE(a.tracks.front().coasting);
  }
  const auto released = monitor.update(tracked_frame({}, 10.0), 5 * kFrameNs);
  EXPECT_NE(released.state, State::OBSTACLE);
  EXPECT_TRUE(released.tracks.empty());
}

TEST(Tracker, TimeGapBackwardStepAndResetRestartTracks) {
  for (const int variant : {0, 1, 2}) {
    SCOPED_TRACE(variant);
    TemporalMonitor monitor(rule(2, 3, 3));
    monitor.update(tracked_frame({{40, 0}}), 10 * kFrameNs);
    ASSERT_EQ(monitor.update(tracked_frame({{40, 0}}), 11 * kFrameNs).state, State::OBSTACLE);
    std::int64_t next = 12 * kFrameNs;
    if (variant == 0) next = 30 * kFrameNs;  // 1.8 s pause > max_gap_s.
    if (variant == 1) next = 5 * kFrameNs;   // Time went backwards.
    if (variant == 2) monitor.reset();
    const auto a = monitor.update(tracked_frame({{40, 0}}), next);
    EXPECT_EQ(a.state, State::UNKNOWN);
    EXPECT_EQ(a.reason, "CANDIDATE_UNCONFIRMED");
  }
}

TEST(Tracker, GaugeOnlyTracksUseTheirOwnRule) {
  auto config = rule(2, 3);
  config.gauge_confirm_hits = config.gauge_confirm_window = 3;
  TemporalMonitor monitor(config);
  const Seen gauge{30, 0, ObstacleCandidate::kGauge}, motion{15, 1, ObstacleCandidate::kMotion};
  std::vector<State> states;
  for (int i = 0; i < 3; ++i) {
    const auto a = monitor.update(tracked_frame({gauge, motion}), (i + 1) * kFrameNs);
    std::size_t confirmed_gauge = 0;
    for (const auto& t : a.tracks)
      if (t.channels == ObstacleCandidate::kGauge && t.confirmed) ++confirmed_gauge;
    EXPECT_EQ(confirmed_gauge, i == 2 ? 1u : 0u) << i;
    states.push_back(a.state);
  }
  EXPECT_EQ(states, (std::vector<State>{State::UNKNOWN, State::OBSTACLE, State::OBSTACLE}));
}

TEST(Tracker, UnknownEgoMotionWidensTheGate) {
  // The train approaches at 25 m/s but the odometry is invalid: the object seems to move
  // 2.5 m per frame. With an unknown ego motion it stays one track (gate 4.0 m at 20 m); with
  // a trusted zero ego speed it does not fit the gate (2.0 m) and is never confirmed.
  for (const bool ego_valid : {false, true}) {
    TemporalMonitor monitor(rule(2, 3));
    State last = State::UNKNOWN;
    for (int i = 0; i < 3; ++i)
      last = monitor.update(tracked_frame({{20 - 2.5 * i, 0}}, 0.0, ego_valid), (i + 1) * kFrameNs)
                 .state;
    EXPECT_EQ(last == State::OBSTACLE, !ego_valid) << ego_valid;
  }
}

TEST(Tracker, FailedFrameDoesNotAgeTracks) {
  TemporalMonitor monitor(rule(2, 3));
  monitor.update(tracked_frame({{30, 0}}), kFrameNs);
  FrameResult failed;
  failed.status = AnalysisStatus::INVALID_GEOMETRY;
  failed.reason = "GROUND_UNSUPPORTED";
  const auto during = monitor.update(failed, 2 * kFrameNs);
  EXPECT_EQ(during.state, State::UNKNOWN);
  EXPECT_EQ(during.reason, "GROUND_UNSUPPORTED");
  ASSERT_EQ(during.tracks.size(), 1u);
  const auto after = monitor.update(tracked_frame({{30, 0}}), 3 * kFrameNs);
  EXPECT_EQ(after.state, State::OBSTACLE) << after.reason;
  EXPECT_EQ(after.tracks.front().id, during.tracks.front().id);
}

TEST(Tracker, RepeatedMeasurementIsNoNewEvidence) {
  // A re-delivered cloud has the measurement time of the previous frame: it must not
  // count as a second sighting.
  TemporalMonitor monitor(rule(2, 3));
  monitor.update(tracked_frame({{30, 0}}), kFrameNs);
  const auto repeat = monitor.update(tracked_frame({{30, 0}}), kFrameNs);
  EXPECT_EQ(repeat.state, State::UNKNOWN);
  EXPECT_EQ(repeat.reason, "CANDIDATE_UNCONFIRMED");
  ASSERT_EQ(repeat.tracks.size(), 1u);
  EXPECT_EQ(repeat.tracks.front().hits, 1u);
  EXPECT_EQ(repeat.tracks.front().age_frames, 1u);
  EXPECT_EQ(monitor.update(tracked_frame({{30, 0}}), 2 * kFrameNs).state, State::OBSTACLE);
}

TEST(Tracker, MotionIsConfirmedAtOnceOnlyWhileStandingStill) {
  auto config = rule(2, 3);
  config.still_speed_mps = 0.5;
  const Seen motion{20, 1.9, ObstacleCandidate::kMotion}, gauge{20, 0, ObstacleCandidate::kGauge};
  EXPECT_EQ(TemporalMonitor(config).update(tracked_frame({motion}, 0.1), kFrameNs).state,
            State::OBSTACLE);
  // Moving, gauge-only, sparse or without ego motion, the N-of-M rule applies.
  const Seen sparse{20, 1.9, ObstacleCandidate::kMotion, 10};
  EXPECT_EQ(TemporalMonitor(config).update(tracked_frame({sparse}, 0.1), kFrameNs).state,
            State::UNKNOWN);
  EXPECT_EQ(TemporalMonitor(config).update(tracked_frame({motion}, 10.0), kFrameNs).state,
            State::UNKNOWN);
  EXPECT_EQ(TemporalMonitor(config).update(tracked_frame({gauge}, 0.1), kFrameNs).state,
            State::UNKNOWN);
  EXPECT_EQ(TemporalMonitor(config).update(tracked_frame({motion}, 0.0, false), kFrameNs).state,
            State::UNKNOWN);
  // Disabled by default.
  EXPECT_EQ(TemporalMonitor(rule(2, 3)).update(tracked_frame({motion}, 0.1), kFrameNs).state,
            State::UNKNOWN);
}

TEST(Tracker, InvalidConfigurationIsRejected) {
  auto config = rule(3, 2);
  EXPECT_THROW(TemporalMonitor{config}, std::invalid_argument);
  config = rule(1, 1);
  config.gate_base_m = 0;
  EXPECT_THROW(TemporalMonitor{config}, std::invalid_argument);
  config = rule(1, 1);
  config.still_speed_mps = -1;
  EXPECT_THROW(TemporalMonitor{config}, std::invalid_argument);
}

TEST(Tracker, TracksBeyondTheEvaluatedRangeAlsoNeedTheFarRule) {
  auto config = rule(2, 3);
  config.far_confirm_hits = 4;
  config.far_confirm_window = 5;
  TemporalMonitor monitor(config);
  const Seen near{50, 0, ObstacleCandidate::kGauge}, far{100, 0, ObstacleCandidate::kGauge};
  for (int i = 0; i < 4; ++i) {
    const auto a = monitor.update(tracked_frame({near, far}), (i + 1) * kFrameNs);
    bool near_confirmed = false, far_confirmed = false;
    for (const auto& t : a.tracks)
      (t.distance_m < 80 ? near_confirmed : far_confirmed) = t.confirmed;
    EXPECT_EQ(near_confirmed, i >= 1) << i;  // The evaluated range is 80 m.
    EXPECT_EQ(far_confirmed, i >= 3) << i;
  }
}

TEST(Tracker, EdgeMeasurementsKeepATrackButNeverConfirmIt) {
  TemporalMonitor monitor(rule(2, 3));
  auto frame_with = [](bool edge, double x) {
    auto frame = tracked_frame({{x, 0.9, ObstacleCandidate::kGauge}}, 10.0);
    frame.candidates.front().edge = edge;
    return frame;
  };
  // Approaching at 1 m per frame: at the edge for five frames, then clearly inside.
  for (int i = 0; i < 5; ++i) {
    const auto a = monitor.update(frame_with(true, 60.0 - i), (i + 1) * kFrameNs);
    EXPECT_EQ(a.state, State::UNKNOWN) << i;  // Never a clear path while it is seen.
    ASSERT_EQ(a.tracks.size(), 1u) << i;
    EXPECT_FALSE(a.tracks.front().confirmed) << i;
  }
  monitor.update(frame_with(false, 55.0), 6 * kFrameNs);
  const auto a = monitor.update(frame_with(false, 54.0), 7 * kFrameNs);
  EXPECT_EQ(a.state, State::OBSTACLE) << a.reason;
  ASSERT_EQ(a.tracks.size(), 1u);  // The same track, now confirmed.
  EXPECT_TRUE(a.tracks.front().confirmed);
}

TEST(Tracker, TracksBeyondTheFarDistanceAlsoNeedTheFarRule) {
  auto config = rule(2, 3);
  config.far_confirm_hits = 3;
  config.far_confirm_window = 4;
  config.far_confirm_from_m = 70;  // Nearer than the evaluated range of 80 m.
  TemporalMonitor monitor(config);
  const Seen near{50, 0, ObstacleCandidate::kGauge}, far{75, 0, ObstacleCandidate::kGauge};
  for (int i = 0; i < 3; ++i) {
    const auto a = monitor.update(tracked_frame({near, far}), (i + 1) * kFrameNs);
    bool near_confirmed = false, far_confirmed = false;
    for (const auto& t : a.tracks)
      (t.distance_m < 70 ? near_confirmed : far_confirmed) = t.confirmed;
    EXPECT_EQ(near_confirmed, i >= 1) << i;
    EXPECT_EQ(far_confirmed, i >= 2) << i;
  }
}
