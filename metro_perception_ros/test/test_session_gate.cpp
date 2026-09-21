#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

#include "metro_perception_ros/session_gate.hpp"

TEST(SessionGate, ResetRejectsCompletedOldWork) {
  metro_perception_ros::SessionGate gate;
  const auto work_session = gate.current();
  gate.advance();
  bool published = false;
  EXPECT_FALSE(gate.publish_if_current(work_session, [&] { published = true; }));
  EXPECT_FALSE(published);
  EXPECT_TRUE(gate.publish_if_current(gate.current(), [&] { published = true; }));
  EXPECT_TRUE(published);
}

TEST(SessionGate, ResetCannotCompleteInsidePublication) {
  metro_perception_ros::SessionGate gate;
  std::promise<void> publishing, release, resetting;
  auto released = release.get_future();
  auto publication = std::async(std::launch::async, [&] {
    return gate.publish_if_current(0, [&] {
      publishing.set_value();
      released.wait();
    });
  });
  publishing.get_future().wait();
  auto reset = std::async(std::launch::async, [&] {
    resetting.set_value();
    gate.advance();
  });
  resetting.get_future().wait();
  EXPECT_EQ(reset.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
  release.set_value();
  EXPECT_TRUE(publication.get());
  reset.get();
  EXPECT_EQ(gate.current(), 1u);
  EXPECT_FALSE(gate.publish_if_current(0, [] {}));
}
