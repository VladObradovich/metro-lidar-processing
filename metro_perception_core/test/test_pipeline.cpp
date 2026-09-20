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
  EXPECT_EQ(result.status, AnalysisStatus::NOT_IMPLEMENTED);
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
