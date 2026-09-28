#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <utility>
#include <vector>

#include "metro_perception_core/speed_tracker.hpp"
using namespace metro_perception_core;

namespace {
constexpr double kDt = 0.1;          // 10 Hz: one 5 cm shift per frame is 0.5 m/s.
constexpr double kContrast = 0.005;  // AlgorithmConfig::ego_min_contrast.

// Profile error over shifts 0..50 (up to 25 m/s): flat at 0.10 with V-shaped minima
// of the given depth, three shifts wide on each side.
std::vector<double> curve(std::initializer_list<std::pair<int, double>> minima) {
  std::vector<double> errors(51, 0.10);
  for (const auto& [shift, depth] : minima)
    for (int i = 0; i < 51; ++i)
      errors[i] =
          std::min(errors[i], 0.10 - depth * std::max(0.0, 1.0 - std::abs(i - shift) / 3.0));
  return errors;
}

SpeedTracker::Step feed(SpeedTracker& tracker, const std::vector<double>& errors, int frames) {
  SpeedTracker::Step step;
  for (int i = 0; i < frames; ++i) step = tracker.update(errors, kDt, kContrast);
  return step;
}

double speed_of(const SpeedTracker::Step& step) { return step.displacement_m / kDt; }
}  // namespace

TEST(SpeedTracker, AcquiresOnlyAMinimumThatStandsOut) {
  SpeedTracker clear;
  EXPECT_FALSE(feed(clear, curve({{32, 0.02}}), 2).valid);
  const auto acquired = feed(clear, curve({{32, 0.02}}), 1);  // Third consistent frame.
  ASSERT_TRUE(acquired.valid);
  EXPECT_DOUBLE_EQ(speed_of(acquired), 16.0);

  // A lining alias almost as good as the true shift: no speed at all.
  SpeedTracker ambiguous;
  feed(ambiguous, curve({{32, 0.02}, {12, 0.018}}), 10);
  EXPECT_FALSE(ambiguous.speed_known());
}

TEST(SpeedTracker, AliasOutOfPhysicalReachIsIgnoredWhileTracking) {
  SpeedTracker tracker;
  feed(tracker, curve({{32, 0.02}}), 3);
  // For 1.6 s an alias one lining period away matches better than the true shift.
  const auto step = feed(tracker, curve({{12, 0.03}, {32, 0.01}}), 16);
  ASSERT_TRUE(step.valid);
  EXPECT_NEAR(speed_of(step), 16.0, 0.5);
}

TEST(SpeedTracker, DistantMinimumThatPersistsReplacesTheSpeed) {
  SpeedTracker tracker;
  feed(tracker, curve({{32, 0.02}}), 3);
  // The true shift is gone and a distant one stands out: the speed is held until the
  // distant minimum has persisted for 25 frames, then it is taken.
  auto step = feed(tracker, curve({{12, 0.03}}), 24);
  ASSERT_TRUE(step.valid);
  EXPECT_DOUBLE_EQ(speed_of(step), 16.0);
  step = feed(tracker, curve({{12, 0.03}}), 1);
  ASSERT_TRUE(step.valid);
  EXPECT_DOUBLE_EQ(speed_of(step), 6.0);
}

TEST(SpeedTracker, FeaturelessTunnelNeverSettlesAtStandstill) {
  // Moving through a featureless tunnel only the scan pattern matches at zero shift,
  // weakly (0.8 of the median): the train is never taken as standing still.
  SpeedTracker tracker;
  feed(tracker, curve({{32, 0.02}}), 3);
  for (int i = 0; i < 200; ++i) {
    const auto step = tracker.update(curve({{0, 0.02}}), kDt, kContrast);
    if (step.valid) EXPECT_GT(speed_of(step), 10.0) << "frame " << i;
  }
}

TEST(SpeedTracker, StandstillIsTakenOnASharpMatch) {
  SpeedTracker tracker;
  const auto step = feed(tracker, curve({{0, 0.06}}), 3);
  ASSERT_TRUE(step.valid);
  EXPECT_DOUBLE_EQ(speed_of(step), 0.0);
}

TEST(SpeedTracker, SpeedChangesNoFasterThanTheTrainAtSpeed) {
  SpeedTracker tracker;
  feed(tracker, curve({{32, 0.02}}), 3);
  // The shift drops by two steps (1 m/s) at once; 2 m/s^2 allows 0.2 m/s per frame.
  const auto step = feed(tracker, curve({{30, 0.02}}), 1);
  ASSERT_TRUE(step.valid);
  EXPECT_NEAR(speed_of(step), 15.8, 1e-9);
}

TEST(SpeedTracker, StartingTrainIsFollowedFromStandstill) {
  SpeedTracker tracker;
  feed(tracker, curve({{0, 0.06}}), 3);
  // Starting off, a shift of 3 m/s clearly beats the standstill that is still tracked.
  const auto step = feed(tracker, curve({{0, 0.02}, {6, 0.04}}), 3);
  ASSERT_TRUE(step.valid);
  EXPECT_DOUBLE_EQ(speed_of(step), 3.0);
}

TEST(SpeedTracker, IncomparableFramesKeepTheState) {
  SpeedTracker tracker;
  feed(tracker, curve({{32, 0.02}}), 3);
  EXPECT_FALSE(tracker.update({}, kDt, kContrast).valid);
  EXPECT_TRUE(tracker.speed_known());
  EXPECT_DOUBLE_EQ(tracker.speed_mps(), 16.0);
}
