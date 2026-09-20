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
