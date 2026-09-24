#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>

#include "metro_perception_core/pipeline.hpp"
#include "metro_perception_core/temporal_monitor.hpp"
using namespace metro_perception_core;
TEST(Pipeline, EmptyAndUnimplementedInputNeverClaimClearPath) {
  PerceptionPipeline pipeline;
  TemporalMonitor monitor;
  FrameInput input;
  for (int i = 0; i < 2; ++i) {
    const auto frame = pipeline.process(input);
    const auto assessment = monitor.update(frame, 0);
    EXPECT_EQ(assessment.state, State::UNKNOWN);
    EXPECT_FALSE(assessment.distance_valid);
    EXPECT_FALSE(frame.evaluation_region_valid);
    input.points.push_back({10, 0, 0});
  }
  EXPECT_EQ(monitor.on_timeout().state, State::UNKNOWN);
}
TEST(Pipeline, RejectsInvalidLimits) {
  AlgorithmConfig config;
  config.max_points = 0;
  EXPECT_THROW(PerceptionPipeline pipeline(config), std::invalid_argument);
  config = {};
  config.ground_max_gap_m = 4.9;  // Shorter than one support bin.
  EXPECT_THROW(PerceptionPipeline pipeline(config), std::invalid_argument);
}

TEST(Preprocessing, FiltersBeforeTranslationAndPreservesRawIndices) {
  FrameInput input;
  input.context.transform_available = true;
  input.context.calibration_verified = true;
  input.context.sensor_to_target.translation = {1, 2, 3};
  input.context.sensor_to_target.rotation = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  input.points = {{0, 0, 0},    {NAN, 1, 2}, {1, INFINITY, 2}, {.1, 0, 0},
                  {0, -10, -1}, {8, -10, 0}, {0, -200, 0}};
  auto result = PerceptionPipeline().process(input);
  const auto& p = result.preprocessed;
  ASSERT_EQ(p.geometry_points.size(), 2u);
  EXPECT_EQ(p.invalid_points, 3u);
  EXPECT_EQ(p.blind_points, 1u);
  EXPECT_EQ(p.outside_roi_points, 1u);
  EXPECT_EQ(p.geometry_points[0].raw_index, 4u);
  EXPECT_DOUBLE_EQ(p.geometry_points[0].point.x, 11);
  EXPECT_DOUBLE_EQ(p.geometry_points[0].point.y, 2);
  EXPECT_DOUBLE_EQ(p.geometry_points[0].point.z, 2);
  EXPECT_EQ(p.geometry_points[1].raw_index, 5u);
  EXPECT_EQ(p.detection_indices, std::vector<std::size_t>{0});
  EXPECT_DOUBLE_EQ(p.sensor_origin.z, 3);
  EXPECT_EQ(result.status, AnalysisStatus::INVALID_GEOMETRY);
  EXPECT_EQ(result.calibration_trust, CalibrationTrust::VERIFIED);
  EXPECT_FALSE(result.evaluation_region_valid);
}
TEST(Preprocessing, FailsClosedAndDoesNotAccumulate) {
  FrameInput input;
  input.points = {{10, 0, 0}};
  PerceptionPipeline pipeline;
  EXPECT_EQ(pipeline.process(input).status, AnalysisStatus::TF_UNAVAILABLE);
  input.context.transform_available = true;
  auto result = pipeline.process(input);
  EXPECT_EQ(result.reason, "CALIBRATION_UNVERIFIED");
  EXPECT_TRUE(result.preprocessed.transform_applied);
  input.context.calibration_verified = true;
  input.context.sensor_to_target.rotation[0] = -1;  // Reflection is not a rigid rotation.
  EXPECT_EQ(pipeline.process(input).reason, "INVALID_TRANSFORM");
  input.context.sensor_to_target = {};
  EXPECT_EQ(pipeline.process(input).preprocessed.geometry_points.size(), 1u);
  input.points = {{0, 0, 0}};
  result = pipeline.process(input);
  EXPECT_EQ(result.reason, "EMPTY_GEOMETRY_ROI");
  EXPECT_TRUE(result.preprocessed.geometry_points.empty());
  EXPECT_EQ(TemporalMonitor().update(result, 1).state, State::UNKNOWN);
}
TEST(Preprocessing, ValidatesRoiAndBlindMask) {
  AlgorithmConfig config;
  config.geometry_roi.min[0] = NAN;
  EXPECT_THROW(PerceptionPipeline pipeline(config), std::invalid_argument);
  config = {};
  config.detection_roi.max[0] = 500;
  EXPECT_THROW(PerceptionPipeline pipeline(config), std::invalid_argument);
  config = {};
  config.blind_radius_m = -1;
  EXPECT_THROW(PerceptionPipeline pipeline(config), std::invalid_argument);
}

TEST(CalibrationTrust, AssumedCalibrationIsCarriedAndCannotConfirmClearPath) {
  FrameInput input;
  input.points = {{10, 0, 0}};
  input.context.transform_available = true;
  input.context.allow_unverified_calibration = true;

  const auto preprocessed = PerceptionPipeline().process(input);
  EXPECT_EQ(preprocessed.status, AnalysisStatus::INVALID_GEOMETRY);
  EXPECT_EQ(preprocessed.calibration_trust, CalibrationTrust::ASSUMED);

  FrameResult future_detector_result;
  future_detector_result.status = AnalysisStatus::OK;
  future_detector_result.reason = "OK";
  future_detector_result.evaluation_region_valid = true;
  future_detector_result.calibration_trust = CalibrationTrust::ASSUMED;

  const auto assessment = TemporalMonitor().update(future_detector_result, 1);
  EXPECT_EQ(assessment.state, State::UNKNOWN);
  EXPECT_EQ(assessment.reason, "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR");
  EXPECT_EQ(assessment.calibration_trust, CalibrationTrust::ASSUMED);
  EXPECT_FALSE(assessment.distance_valid);
}

TEST(CalibrationTrust, VerifiedGeometryCanConfirmClearPath) {
  FrameResult frame;
  frame.status = AnalysisStatus::OK;
  frame.reason = "OK";
  frame.evaluation_region_valid = true;
  frame.evaluated_range_m = 40.0;
  frame.calibration_trust = CalibrationTrust::VERIFIED;

  const auto assessment = TemporalMonitor().update(frame, 1);
  EXPECT_EQ(assessment.state, State::NO_OBSTACLE_DETECTED);
  EXPECT_EQ(assessment.reason, "NO_CANDIDATE_IN_EVALUATED_REGION");
  EXPECT_EQ(assessment.calibration_trust, CalibrationTrust::VERIFIED);
}

namespace {
FrameInput synthetic_scene(bool obstacle, bool verified) {
  FrameInput input;
  input.context.transform_available = true;
  input.context.calibration_verified = verified;
  input.context.allow_unverified_calibration = !verified;
  for (int x = 4; x <= 50; ++x) {
    for (int yi = -12; yi <= 12; ++yi) {
      input.points.push_back({double(x), yi * 0.2, -1.0});
    }
  }
  if (obstacle) {
    for (int xi = 0; xi < 7; ++xi)
      for (int yi = 0; yi < 9; ++yi)
        for (int zi = 0; zi < 12; ++zi)
          input.points.push_back({20.0 + xi * 0.08, -0.32 + yi * 0.08, -0.7 + zi * 0.12});
  }
  return input;
}
}  // namespace

TEST(Detector, FindsObstacleAndLongitudinalDistance) {
  AlgorithmConfig config;
  config.background_history_frames = 0;
  const auto frame = PerceptionPipeline(config).process(synthetic_scene(true, false));
  ASSERT_EQ(frame.status, AnalysisStatus::OK)
      << frame.reason << " geometry=" << frame.preprocessed.geometry_points.size()
      << " detection=" << frame.preprocessed.detection_indices.size();
  ASSERT_FALSE(frame.candidates.empty());
  EXPECT_TRUE(frame.evaluation_region_valid);
  EXPECT_NEAR(frame.candidates.front().distance_m, 20.0, 0.2);
  EXPECT_GT(frame.candidates.front().support_points, 5u);
  const auto assessment = TemporalMonitor().update(frame, 1);
  EXPECT_EQ(assessment.state, State::OBSTACLE);
  EXPECT_TRUE(assessment.distance_valid);
  EXPECT_EQ(assessment.calibration_trust, CalibrationTrust::ASSUMED);
}

TEST(Detector, ClearNeedsVerifiedCalibration) {
  AlgorithmConfig config;
  config.background_history_frames = 0;
  const auto assumed = PerceptionPipeline(config).process(synthetic_scene(false, false));
  ASSERT_EQ(assumed.status, AnalysisStatus::OK)
      << assumed.reason << " geometry=" << assumed.preprocessed.geometry_points.size();
  EXPECT_TRUE(assumed.candidates.empty());
  EXPECT_EQ(TemporalMonitor().update(assumed, 1).state, State::UNKNOWN);
  const auto verified = PerceptionPipeline(config).process(synthetic_scene(false, true));
  ASSERT_EQ(verified.status, AnalysisStatus::OK) << verified.reason;
  EXPECT_EQ(TemporalMonitor().update(verified, 1).state, State::NO_OBSTACLE_DETECTED);
}

TEST(Detector, SparseGroundCannotClaimClearOrObstacle) {
  FrameInput input;
  input.context.transform_available = true;
  input.context.calibration_verified = true;
  for (int i = 0; i < 20; ++i) input.points.push_back({20, i * 0.02, 0.5});
  const auto frame = PerceptionPipeline().process(input);
  EXPECT_EQ(frame.status, AnalysisStatus::INVALID_GEOMETRY);
  EXPECT_EQ(frame.reason, "GROUND_UNSUPPORTED");
  EXPECT_FALSE(frame.evaluation_region_valid);
  EXPECT_EQ(TemporalMonitor().update(frame, 1).state, State::UNKNOWN);
}

TEST(Detector, RollingBackgroundSuppressesStaticSceneAndFindsNewObject) {
  PerceptionPipeline pipeline;
  const auto clear = synthetic_scene(false, false);
  for (int i = 0; i < 10; ++i) {
    const auto warmup = pipeline.process(clear);
    EXPECT_EQ(warmup.reason, "BASELINE_WARMUP");
    EXPECT_FALSE(warmup.evaluation_region_valid);
  }
  const auto background = pipeline.process(clear);
  ASSERT_EQ(background.status, AnalysisStatus::OK) << background.reason;
  EXPECT_TRUE(background.candidates.empty());
  EXPECT_FALSE(background.evaluation_region_valid);
  EXPECT_EQ(TemporalMonitor().update(background, 0).state, State::UNKNOWN);
  const auto positive = pipeline.process(synthetic_scene(true, false));
  ASSERT_EQ(positive.status, AnalysisStatus::OK) << positive.reason;
  ASSERT_FALSE(positive.candidates.empty());
  EXPECT_NEAR(positive.candidates.front().distance_m, 20.0, 0.2);
  pipeline.reset();
  EXPECT_EQ(pipeline.process(synthetic_scene(true, false)).reason, "BASELINE_WARMUP");

  PerceptionPipeline verified_pipeline;
  const auto verified_clear = synthetic_scene(false, true);
  for (int i = 0; i < 10; ++i) verified_pipeline.process(verified_clear);
  const auto verified_background = verified_pipeline.process(verified_clear);
  EXPECT_EQ(verified_background.reason, "BACKGROUND_CANNOT_CONFIRM_CLEAR");
  EXPECT_EQ(TemporalMonitor().update(verified_background, 1).state, State::UNKNOWN);
}

namespace {
FrameInput verified_input() {
  FrameInput input;
  input.context.transform_available = true;
  input.context.calibration_verified = true;
  return input;
}
void add_floor(FrameInput& input, int x_from, int x_to, double grade = 0.0) {
  for (int x = x_from; x <= x_to; ++x)
    for (int yi = -12; yi <= 12; ++yi)
      input.points.push_back({double(x), yi * 0.2, -1.0 + grade * x});
}
AlgorithmConfig single_frame_config() {
  AlgorithmConfig config;
  config.background_history_frames = 0;
  return config;
}
}  // namespace

TEST(Ground, SlopedFloorIsFollowed) {
  auto input = verified_input();
  const double grade = 0.02;
  add_floor(input, 4, 50, grade);
  for (int yi = 0; yi < 9; ++yi)
    for (int zi = 0; zi < 12; ++zi)
      input.points.push_back({20.0, -0.32 + yi * 0.08, -1.0 + grade * 20 + 0.3 + zi * 0.12});
  const auto frame = PerceptionPipeline(single_frame_config()).process(input);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  ASSERT_EQ(frame.corridor.size(), 1u);
  const auto& plane = frame.corridor.front().ground_plane;
  EXPECT_NEAR(-plane[0] / plane[2], grade, 1e-3);
  EXPECT_NEAR(-plane[3] / plane[2], -1.0, 0.02);
  ASSERT_FALSE(frame.candidates.empty());
  EXPECT_NEAR(frame.candidates.front().distance_m, 20.0, 0.2);
}

TEST(Ground, WallIsNeverChosenAsFloor) {
  auto wall_only = verified_input();
  for (int x = 4; x <= 50; ++x)
    for (int zi = 0; zi <= 15; ++zi) wall_only.points.push_back({double(x), 1.5, -1.0 + zi * 0.1});
  const auto rejected = PerceptionPipeline(single_frame_config()).process(wall_only);
  EXPECT_EQ(rejected.reason, "GROUND_UNSUPPORTED");
  EXPECT_EQ(TemporalMonitor().update(rejected, 1).state, State::UNKNOWN);

  auto with_floor = wall_only;
  add_floor(with_floor, 4, 50);
  const auto frame = PerceptionPipeline(single_frame_config()).process(with_floor);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  EXPECT_NEAR(frame.corridor.front().ground_plane[2], 1.0, 1e-3);
  EXPECT_NEAR(-frame.corridor.front().ground_plane[3], -1.0, 0.02);
}

TEST(Ground, DenseStructureBesideCorridorIsNotTheFloor) {
  // Platforms on both sides hold more returns than the track bed. They must not become
  // the route floor, or an obstacle standing on the bed loses its body below "ground".
  FrameInput input;
  input.context.transform_available = true;
  input.context.allow_unverified_calibration = true;
  for (int x = 4; x <= 50; ++x)
    for (int yi = -12; yi <= 12; ++yi) input.points.push_back({double(x), yi * 0.2, -1.9});
  for (int xi = 12; xi <= 200; ++xi)
    for (const double y : {2.6, 2.7, 2.8, 2.9})
      for (const double side : {-1.0, 1.0}) input.points.push_back({xi * 0.25, side * y, -0.3});
  for (int xi = 0; xi < 5; ++xi)
    for (int yi = 0; yi < 8; ++yi)
      for (int zi = 0; zi <= 21; ++zi)
        input.points.push_back({20.0 + xi * 0.08, -0.28 + yi * 0.08, -1.9 + zi * 0.08});
  AlgorithmConfig config;
  config.background_history_frames = 0;
  const auto frame = PerceptionPipeline(config).process(input);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  ASSERT_FALSE(frame.corridor.empty());
  const auto& plane = frame.corridor.front().ground_plane;  // Normalised: z + 1.9 = 0.
  EXPECT_NEAR(-plane[3] / plane[2], -1.9, 0.05);
  ASSERT_EQ(frame.candidates.size(), 1u);
  EXPECT_NEAR(frame.candidates.front().distance_m, 20.0, 0.1);
  EXPECT_GT(frame.candidates.front().size.z, 1.2);  // Whole body above the bed.
}

TEST(Ground, OneSidedPlatformCannotEstablishRoute) {
  auto input = verified_input();
  for (int x = 4; x <= 50; ++x)
    for (int yi = 0; yi <= 10; ++yi) input.points.push_back({double(x), 1.0 + yi * 0.2, 0.0});
  const auto frame = PerceptionPipeline(single_frame_config()).process(input);
  EXPECT_EQ(frame.reason, "GROUND_UNSUPPORTED");
  EXPECT_FALSE(frame.evaluation_region_valid);
}

TEST(Ground, FloorGapEndsUsableRange) {
  auto input = verified_input();
  add_floor(input, 4, 40);
  add_floor(input, 60, 88);
  const auto frame = PerceptionPipeline(single_frame_config()).process(input);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  EXPECT_NEAR(frame.evaluated_range_m, 43.0, 0.01);
  EXPECT_GT(frame.corridor.front().ground_inliers, 0u);

  // Returns at the far edge of the next 5 m bin: 37 -> 51.7 m is a 14.7 m gap.
  auto edge_gap = verified_input();
  add_floor(edge_gap, 4, 37);
  add_floor(edge_gap, 37, 37);  // Enough support for the 37-42 m bin to end at 37 m.
  for (const double x : {51.7, 51.9})
    for (int yi = -12; yi <= 12; ++yi) edge_gap.points.push_back({x, yi * 0.2, -1.0});
  add_floor(edge_gap, 52, 60);
  const auto edge = PerceptionPipeline(single_frame_config()).process(edge_gap);
  ASSERT_EQ(edge.status, AnalysisStatus::OK) << edge.reason;
  EXPECT_NEAR(edge.evaluated_range_m, 40.0, 0.01);

  auto near_gap = verified_input();
  add_floor(near_gap, 20, 60);  // Nothing observed in the first 18 m.
  const auto unobservable = PerceptionPipeline(single_frame_config()).process(near_gap);
  EXPECT_EQ(unobservable.reason, "CORRIDOR_UNOBSERVABLE");
  EXPECT_EQ(TemporalMonitor().update(unobservable, 1).state, State::UNKNOWN);
}

TEST(Ground, ShortDropoutKeepsBackgroundHistory) {
  PerceptionPipeline pipeline;
  const auto clear = synthetic_scene(false, false);
  FrameInput no_floor = clear;
  no_floor.points = {{20, 0, 0.5}};
  for (int i = 0; i < 10; ++i) pipeline.process(clear);
  EXPECT_EQ(pipeline.process(no_floor).reason, "GROUND_UNSUPPORTED");
  const auto resumed = pipeline.process(synthetic_scene(true, false));
  ASSERT_EQ(resumed.status, AnalysisStatus::OK) << resumed.reason;
  EXPECT_FALSE(resumed.candidates.empty());

  for (int i = 0; i < 6; ++i) EXPECT_EQ(pipeline.process(no_floor).reason, "GROUND_UNSUPPORTED");
  EXPECT_EQ(pipeline.process(clear).reason, "BASELINE_WARMUP");
}

namespace {
void add_box(FrameInput& input, double x, double y0, double y1, double h0, double h1) {
  for (double dx = 0; dx <= 0.49; dx += 0.08)
    for (double y = y0; y <= y1 + 1e-9; y += 0.08)
      for (double h = h0; h <= h1 + 1e-9; h += 0.08) input.points.push_back({x + dx, y, -1.0 + h});
}
}  // namespace

TEST(Candidates, OutsideCorridorIsIgnored) {
  for (const auto& [y0, y1] : {std::pair{2.6, 3.0}, std::pair{2.1, 2.4}}) {
    auto input = verified_input();
    add_floor(input, 4, 50);
    add_box(input, 20, y0, y1, 0.3, 1.7);
    const auto frame = PerceptionPipeline(single_frame_config()).process(input);
    ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
    EXPECT_TRUE(frame.candidates.empty()) << "box at y=" << y0 << ".." << y1;
  }
}

TEST(Candidates, BoundaryObjectKeepsFullExtentAndInsideDistance) {
  auto input = verified_input();
  add_floor(input, 4, 50);
  add_box(input, 20, 1.6, 2.4, 0.3, 1.7);
  const auto frame = PerceptionPipeline(single_frame_config()).process(input);
  ASSERT_EQ(frame.candidates.size(), 1u) << frame.reason;
  const auto& candidate = frame.candidates.front();
  EXPECT_NEAR(candidate.size.y, 0.8, 0.1);
  EXPECT_NEAR(candidate.size.z, 1.4, 0.1);
  EXPECT_LE(candidate.nearest_point.y, 2.0);
  EXPECT_NEAR(candidate.distance_m, 20.0, 0.1);
}

TEST(Candidates, LowTargetNeedsMinimumHeight) {
  auto flat = verified_input();
  add_floor(flat, 4, 50);
  add_box(flat, 20, -0.4, 0.4, 0.0, 0.2);
  EXPECT_TRUE(PerceptionPipeline(single_frame_config()).process(flat).candidates.empty());

  auto low = verified_input();
  add_floor(low, 4, 50);
  add_box(low, 20, -0.4, 0.4, 0.0, 0.7);
  const auto frame = PerceptionPipeline(single_frame_config()).process(low);
  ASSERT_FALSE(frame.candidates.empty()) << frame.reason;
  EXPECT_NEAR(frame.candidates.front().distance_m, 20.0, 0.1);
}

namespace {
// Straight tunnel with irregular posts beside the track, seen from a train that
// has travelled `travelled` metres.
FrameInput moving_tunnel(int frame_index, double speed_mps, bool obstacle) {
  auto input = verified_input();
  input.context.calibration_verified = false;
  input.context.allow_unverified_calibration = true;
  input.context.measurement_time_ns = 1000000000LL + frame_index * 100000000LL;
  const double travelled = speed_mps * 0.1 * frame_index;
  add_floor(input, 1, 100);
  const double spacing[] = {9.3, 13.7, 8.9, 14.4, 11.8, 10.1, 15.2, 12.2};
  double post = 3;
  for (int i = 0; post < 260; post += spacing[i++ % 8]) {
    const double x = post - travelled;
    if (x < 1 || x > 100) continue;
    for (const double y : {-1.9, 1.8})
      for (double dx = 0; dx <= 0.2; dx += 0.1)
        for (double h = 0.0; h <= 2.4; h += 0.1) input.points.push_back({x + dx, y, -1.0 + h});
  }
  // Walls with niches of irregular length, outside the corridor band.
  const double niche[] = {0.7, 2.9, 1.3, 2.1, 0.9, 3.0, 1.6, 2.4, 1.1};
  double edge = 0;
  for (int i = 0; edge < 260; edge += niche[i++ % 9]) {
    const double lateral = i % 2 ? 3.0 : 2.6;
    for (double xw = edge; xw < edge + niche[i % 9]; xw += 0.1) {
      const double x = xw - travelled;
      if (x < 1 || x > 100) continue;
      for (double h = 0.3; h <= 2.9; h += 0.3)
        for (const double side : {-1.0, 1.0}) input.points.push_back({x, side * lateral, -1.0 + h});
    }
  }
  if (obstacle) add_box(input, 20, -0.4, 0.4, 0.3, 1.7);
  return input;
}
}  // namespace

TEST(EgoMotion, MovingTunnelStructureStaysInBackground) {
  AlgorithmConfig uncompensated;
  uncompensated.ego_motion_compensation = false;
  PerceptionPipeline moving, reference(uncompensated);
  FrameResult frame, frame_reference;
  for (int i = 0; i < 16; ++i) {
    frame = moving.process(moving_tunnel(i, 10, false));
    frame_reference = reference.process(moving_tunnel(i, 10, false));
  }
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  EXPECT_TRUE(frame.ego_motion_valid);
  EXPECT_NEAR(frame.ego_speed_mps, 10.0, 0.6);
  EXPECT_TRUE(frame.candidates.empty())
      << frame.candidates.size() << " at " << frame.candidates.front().distance_m;
  ASSERT_EQ(frame_reference.status, AnalysisStatus::OK);
  EXPECT_FALSE(frame_reference.candidates.empty());  // Approaching posts look new.

  const auto positive = moving.process(moving_tunnel(16, 10, true));
  ASSERT_FALSE(positive.candidates.empty()) << positive.reason;
  EXPECT_NEAR(positive.candidates.front().distance_m, 20.0, 0.1);
}

TEST(EgoMotion, StationaryTrainReportsZeroSpeed) {
  PerceptionPipeline pipeline;
  FrameResult frame;
  for (int i = 0; i < 12; ++i) frame = pipeline.process(moving_tunnel(i, 0, false));
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  EXPECT_TRUE(frame.ego_motion_valid);
  EXPECT_NEAR(frame.ego_speed_mps, 0.0, 1e-9);
  EXPECT_TRUE(frame.candidates.empty());
}

TEST(EgoMotion, LongGapAtSpeedStaysInsideSearchRange) {
  PerceptionPipeline pipeline;
  for (int i = 0; i < 12; ++i) pipeline.process(moving_tunnel(i, 10, false));
  // 1.9 s later the predicted shift (19 m) exceeds the searchable range.
  const auto frame = pipeline.process(moving_tunnel(30, 10, false));
  EXPECT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  EXPECT_TRUE(frame.candidates.empty());
}

TEST(Candidates, HeightThresholdUsesOnlyInsideReturns) {
  auto low_inside = verified_input();
  add_floor(low_inside, 4, 50);
  add_box(low_inside, 20, 1.6, 1.96, 0.3, 0.5);  // 0.16 m of height inside the corridor.
  EXPECT_TRUE(PerceptionPipeline(single_frame_config()).process(low_inside).candidates.empty());

  auto with_tall_outside = low_inside;
  add_box(with_tall_outside, 20, 2.04, 2.4, 0.3, 1.7);  // Connected, past the edge.
  const auto frame = PerceptionPipeline(single_frame_config()).process(with_tall_outside);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  EXPECT_TRUE(frame.candidates.empty());
}

TEST(EgoMotion, TooFewComparableShiftsNeverConfirmSpeed) {
  // Uniform walls over 1.8 m only: shifts beyond two 5 cm steps lack overlap, so
  // no best shift can be compared with the rest.
  auto scene = [](int frame_index) {
    auto input = verified_input();
    input.context.calibration_verified = false;
    input.context.allow_unverified_calibration = true;
    input.context.measurement_time_ns = 1000000000LL + frame_index * 100000000LL;
    add_floor(input, 1, 100);
    for (double x = 3.0; x < 4.79; x += 0.05)
      for (const double h : {0.5, 1.5, 2.5})
        for (const double side : {-1.0, 1.0}) input.points.push_back({x, side * 2.6, -1.0 + h});
    return input;
  };
  PerceptionPipeline pipeline;
  for (int i = 0; i < 8; ++i) {
    const auto frame = pipeline.process(scene(i));
    EXPECT_FALSE(frame.ego_motion_valid) << "frame " << i;
  }
}

namespace {
// Tunnel bending left on radius `radius` m: floor and walls follow y = x^2 / (2 radius).
FrameInput curved_tunnel(double radius) {
  auto input = verified_input();
  const auto centre = [radius](double x) { return x * x / (2 * radius); };
  for (int x = 1; x <= 90; ++x)
    for (int yi = -9; yi <= 9; ++yi)
      input.points.push_back({double(x), centre(x) + yi * 0.2, -1.0});
  for (double x = 1; x <= 90; x += 0.25)
    for (double h = 0.2; h <= 3.0; h += 0.2)
      for (const double side : {-1.0, 1.0})
        input.points.push_back({x, centre(x) + side * 2.3, -1.0 + h});
  return input;
}
}  // namespace

TEST(Route, CurvedTunnelIsFollowedAndItsWallsAreNotObstacles) {
  const auto scene = curved_tunnel(300);
  const auto frame = PerceptionPipeline(single_frame_config()).process(scene);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  ASSERT_TRUE(frame.route.valid);
  EXPECT_NEAR(frame.route.c2, 1.0 / 600, 0.2 / 600);
  EXPECT_NEAR(frame.route.c1, 0.0, 0.01);
  EXPECT_GT(frame.route.max_x, 60.0);
  EXPECT_TRUE(frame.candidates.empty()) << frame.candidates.front().distance_m;
  ASSERT_GT(frame.corridor.size(), 1u);  // Published as a chain along the curve.
  EXPECT_NEAR(frame.corridor.back().end.y, frame.route.center(frame.corridor.back().end.x), 1e-9);

  auto straight = single_frame_config();
  straight.route_estimation = false;  // The old straight corridor cuts into the wall.
  EXPECT_FALSE(PerceptionPipeline(straight).process(scene).candidates.empty());
}

TEST(Route, ObstacleOnTheCurveIsFoundAndDoesNotBendTheRoute) {
  auto scene = curved_tunnel(300);
  const double y = 40.0 * 40.0 / 600;
  add_box(scene, 40, y - 0.3, y + 0.3, 0.3, 1.7);
  const auto frame = PerceptionPipeline(single_frame_config()).process(scene);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  ASSERT_TRUE(frame.route.valid);
  EXPECT_NEAR(frame.route.c2, 1.0 / 600, 0.2 / 600);
  ASSERT_EQ(frame.candidates.size(), 1u);
  EXPECT_NEAR(frame.candidates.front().distance_m, 40.0, 0.1);
}

TEST(Gauge, ObstacleFixedInTheWorldIsFoundWhileApproaching) {
  // The rolling baseline saw the object at the same place in the tunnel: only the gauge
  // channel, which has no history, can report it.
  auto scene = [](int i) {
    auto input = moving_tunnel(i, 10, false);
    add_box(input, 60.0 - i, -0.3, 0.3, 0.3, 1.7);  // Fixed in the world, 1 m per frame.
    return input;
  };
  AlgorithmConfig without_gauge;
  without_gauge.static_channel = false;
  PerceptionPipeline pipeline, motion_only(without_gauge);
  FrameResult frame, motion;
  for (int i = 0; i <= 16; ++i) {
    frame = pipeline.process(scene(i));
    motion = motion_only.process(scene(i));
  }
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  ASSERT_TRUE(frame.route.valid);
  ASSERT_EQ(frame.candidates.size(), 1u);
  EXPECT_NEAR(frame.candidates.front().distance_m, 44.0, 0.1);
  EXPECT_TRUE(frame.candidates.front().channels & ObstacleCandidate::kGauge);
  EXPECT_TRUE(motion.candidates.empty());  // The failure this channel exists for.
}

TEST(Gauge, LongStructureInsideTheGaugeIsRejectedShortOneIsNot) {
  auto scene = [](int i, bool long_structure) {
    auto input = moving_tunnel(i, 0, false);  // Stationary: the history absorbs both.
    // Tall, so that only the length decides (low objects off the rails are a separate rule).
    if (long_structure)
      for (double x = 30; x < 36; x += 0.5) add_box(input, x, 0.8, 1.0, 0.3, 1.7);
    else
      add_box(input, 30, 0.8, 1.0, 0.3, 1.7);
    return input;
  };
  for (const bool long_structure : {true, false}) {
    PerceptionPipeline pipeline;
    FrameResult frame;
    for (int i = 0; i < 12; ++i) frame = pipeline.process(scene(i, long_structure));
    ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
    ASSERT_TRUE(frame.route.valid);
    EXPECT_EQ(frame.candidates.empty(), long_structure) << frame.candidates.size();
    if (!long_structure) {
      EXPECT_EQ(frame.candidates.front().channels, ObstacleCandidate::kGauge);
      EXPECT_NEAR(frame.candidates.front().distance_m, 30.0, 0.1);
    }
  }
}

TEST(Ground, DenseWalkwaysAboveASparseFloorAreNotTheFloor) {
  // A forward-looking lidar sees the floor at grazing angles and the walkways beside it
  // densely: the floor must win by coverage and by being the lowest surface.
  FrameInput input;
  input.context.transform_available = true;
  input.context.allow_unverified_calibration = true;
  for (int x = 4; x <= 60; ++x)
    for (int yi = -7; yi <= 7; ++yi) input.points.push_back({double(x), yi * 0.2, -1.5});
  for (double x = 4; x <= 60; x += 0.05)
    for (const double y : {1.7, 1.9, 2.1, 2.3})
      for (const double side : {-1.0, 1.0}) input.points.push_back({x, side * y, -1.2});
  AlgorithmConfig config;
  config.background_history_frames = 0;
  const auto frame = PerceptionPipeline(config).process(input);
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  const auto& plane = frame.corridor.front().ground_plane;
  EXPECT_NEAR(-plane[3] / plane[2], -1.5, 0.05);
}

TEST(Candidates, LowObjectsCountOnlyBetweenTheRails) {
  // Rails (+-0.76 m), the contact rail and cable ducts are low and off-centre; a low object
  // between the rails or a tall one anywhere in the corridor is an obstacle.
  auto scene = [](double y0, double y1, double h1) {
    auto input = verified_input();
    add_floor(input, 4, 50);
    add_box(input, 20, y0, y1, 0.3, h1);
    return input;
  };
  const auto config = single_frame_config();
  const auto between = PerceptionPipeline(config).process(scene(-0.2, 0.2, 0.7));
  ASSERT_EQ(between.candidates.size(), 1u) << between.reason;
  EXPECT_NEAR(between.candidates.front().distance_m, 20.0, 0.1);
  EXPECT_TRUE(PerceptionPipeline(config).process(scene(0.7, 0.85, 0.7)).candidates.empty());
  EXPECT_TRUE(PerceptionPipeline(config).process(scene(1.45, 1.65, 0.7)).candidates.empty());
  EXPECT_EQ(PerceptionPipeline(config).process(scene(0.7, 0.85, 1.7)).candidates.size(), 1u);
}

TEST(Candidates, RejectedComponentsAreRecordedOnlyOnRequest) {
  auto input = verified_input();
  add_floor(input, 4, 50);
  add_box(input, 20, 0.7, 0.85, 0.3, 0.7);  // Low and off the rails.
  auto config = single_frame_config();
  EXPECT_TRUE(PerceptionPipeline(config).process(input).rejected.empty());
  config.record_rejected = true;
  const auto frame = PerceptionPipeline(config).process(input);
  EXPECT_TRUE(frame.candidates.empty());
  ASSERT_EQ(frame.rejected.size(), 1u);
  EXPECT_EQ(frame.rejected.front().reason, "LOW_OFF_CENTRE");
  EXPECT_NEAR(frame.rejected.front().center.x, 20.25, 0.3);
}

TEST(Gauge, TrackLineCrossingTheRailBoundaryIsNotSliced) {
  // A low line along the track (a rail seen with a small lateral error of the route) drifts
  // across low_object_half_width_m. The part between the rails must not survive as a short
  // gauge candidate: the whole line is one structure and too long for the gauge.
  auto scene = [](int i) {
    auto input = moving_tunnel(i, 0, false);
    for (double x = 3.0; x < 30.0; x += 0.02)
      for (const double h : {0.30, 0.36, 0.42})
        input.points.push_back({x, 0.35 + 0.03 * x, -1.0 + h});
    return input;
  };
  PerceptionPipeline pipeline;
  FrameResult frame;
  for (int i = 0; i < 12; ++i) frame = pipeline.process(scene(i));
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  ASSERT_TRUE(frame.route.valid);
  EXPECT_TRUE(frame.candidates.empty()) << frame.candidates.front().center.x;
}

TEST(Candidates, LongStructureBesideTheGaugeIsNotAnObstacle) {
  // New returns at the corridor edge that run along the route (odometry error, disocclusion)
  // are a wall or a platform edge. A compact object there, and anything reaching into the
  // gauge, stay candidates.
  auto run = [](const std::function<void(FrameInput&)>& add) {
    PerceptionPipeline pipeline;
    FrameResult frame;
    for (int i = 0; i <= 11; ++i) {
      auto input = moving_tunnel(i, 0, false);
      if (i == 11) add(input);
      frame = pipeline.process(input);
    }
    return frame;
  };
  // Near enough that the grazing wall stays one component (farther it breaks into pieces).
  auto wall = [](FrameInput& input) {
    for (double x = 5; x < 11; x += 0.5) add_box(input, x, 1.7, 1.9, 0.3, 1.7);
  };
  const auto long_edge = run(wall);
  ASSERT_EQ(long_edge.status, AnalysisStatus::OK) << long_edge.reason;
  EXPECT_TRUE(long_edge.candidates.empty()) << long_edge.candidates.front().size.x;
  const auto compact = run([](FrameInput& input) { add_box(input, 20, 1.6, 1.9, 0.3, 1.7); });
  ASSERT_EQ(compact.candidates.size(), 1u);
  EXPECT_EQ(compact.candidates.front().channels, ObstacleCandidate::kMotion);
  const auto reaching = run([&](FrameInput& input) {
    wall(input);
    add_box(input, 8, 0.5, 1.7, 0.3, 1.7);
  });
  // The object reaching into the gauge keeps the component (the wall is merged into it).
  EXPECT_TRUE(std::any_of(reaching.candidates.begin(), reaching.candidates.end(),
                          [](const auto& c) { return c.center.y - c.size.y / 2 < 1.0; }));
}

TEST(Candidates, OutsideTheVehicleEnvelopeCountsOnlyStandingStill) {
  // Between the vehicle envelope and the corridor edge the train passes by. While it moves,
  // new returns that stay there are not candidates; reaching into the envelope they are, and
  // standing still the whole corridor counts.
  auto run = [](double speed, double y0) {
    PerceptionPipeline pipeline;
    FrameResult frame;
    for (int i = 0; i <= 12; ++i) {
      auto input = moving_tunnel(i, speed, false);
      if (i >= 11) add_box(input, 30.0 - speed * 0.1 * (i - 11), y0, 1.9, 0.3, 1.7);
      frame = pipeline.process(input);
    }
    return frame;
  };
  const auto moving = run(10, 1.6);
  ASSERT_EQ(moving.status, AnalysisStatus::OK) << moving.reason;
  ASSERT_TRUE(moving.ego_motion_valid);
  EXPECT_TRUE(moving.candidates.empty()) << moving.candidates.front().center.y;
  EXPECT_EQ(run(10, 1.2).candidates.size(), 1u);
  EXPECT_EQ(run(0, 1.6).candidates.size(), 1u);
}

TEST(Gauge, ObstacleBesideTrackEquipmentIsNotMergedWithIt) {
  // Low equipment by the rail is in the gauge band; a person appearing right behind it must
  // be reported at its own distance, not as one component starting at the equipment.
  auto scene = [](int i) {
    auto input = moving_tunnel(i, 0, false);
    // Guard rail at a switch: continuous, low and off-centre, from near the train.
    for (double x = 6.0; x < 19.95; x += 0.02)
      for (const double y : {0.60, 0.66})
        for (const double h : {0.30, 0.36, 0.42}) input.points.push_back({x, y, -1.0 + h});
    if (i >= 11) add_box(input, 20, 0.40, 0.90, 0.3, 1.7);
    return input;
  };
  PerceptionPipeline pipeline;
  FrameResult frame;
  for (int i = 0; i <= 11; ++i) frame = pipeline.process(scene(i));
  ASSERT_EQ(frame.status, AnalysisStatus::OK) << frame.reason;
  ASSERT_TRUE(frame.route.valid);
  ASSERT_EQ(frame.candidates.size(), 1u);
  EXPECT_NEAR(frame.candidates.front().distance_m, 20.0, 0.1);
  EXPECT_NEAR(frame.candidates.front().center.x, 20.25, 0.5);
  EXPECT_LT(frame.candidates.front().size.x, 1.0);
}
