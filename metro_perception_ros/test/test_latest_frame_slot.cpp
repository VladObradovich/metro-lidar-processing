#include <gtest/gtest.h>

#include "metro_perception_ros/latest_frame_slot.hpp"

using metro_perception_ros::LatestFrameSlot;
using metro_perception_ros::SubmitResult;

TEST(LatestFrameSlot, KeepsOnlyNewestPendingFrame) {
  LatestFrameSlot<int> slot;
  EXPECT_EQ(slot.submit(1), SubmitResult::STORED);
  EXPECT_EQ(slot.submit(2), SubmitResult::OVERWROTE);
  auto value = slot.wait_take();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, 2);
}

TEST(LatestFrameSlot, ClearAndStopAreDeterministic) {
  LatestFrameSlot<int> slot;
  EXPECT_EQ(slot.submit(1), SubmitResult::STORED);
  EXPECT_TRUE(slot.clear());
  EXPECT_FALSE(slot.clear());
  slot.stop();
  EXPECT_FALSE(slot.wait_take().has_value());
  EXPECT_EQ(slot.submit(2), SubmitResult::STOPPED);
}
