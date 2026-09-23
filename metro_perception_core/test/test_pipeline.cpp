#include <gtest/gtest.h>

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
