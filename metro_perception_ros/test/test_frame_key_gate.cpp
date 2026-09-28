#include <gtest/gtest.h>

#include <string>

#include "metro_perception_ros/frame_key_gate.hpp"

using metro_perception_ros::FrameKey;
using metro_perception_ros::FrameKeyGate;

namespace {
FrameKey key(const std::string& source, std::uint64_t session, std::uint64_t sequence,
             std::int64_t stamp = 100) {
  return {source, session, sequence, stamp};
}

void expect(const metro_perception_ros::GateDecision& d, bool accepted, bool reset,
            const std::string& reason) {
  EXPECT_EQ(d.accepted, accepted) << d.reason;
  EXPECT_EQ(d.reset, reset) << d.reason;
  EXPECT_EQ(std::string(d.reason), reason);
}
}  // namespace

TEST(FrameKeyGate, FirstFrameResetsAndInOrderFramesDoNot) {
  FrameKeyGate gate;
  expect(gate.admit(key("a", 0, 1, 100)), true, true, "FIRST_SOURCE");
  expect(gate.admit(key("a", 0, 2, 100)), true, false, "IN_ORDER");  // Equal stamp is allowed.
  expect(gate.admit(key("a", 0, 5, 900)), true, false, "IN_ORDER");  // Gaps and stamp jumps too.
  EXPECT_EQ(gate.active()->sequence, 5u);
}

TEST(FrameKeyGate, DuplicateLateAndBackwardFramesAreRejectedWithoutSideEffects) {
  FrameKeyGate gate;
  gate.admit(key("a", 0, 5, 500));
  expect(gate.admit(key("a", 0, 5, 600)), false, false, "DUPLICATE_OR_LATE_SEQUENCE");
  expect(gate.admit(key("a", 0, 3, 600)), false, false, "DUPLICATE_OR_LATE_SEQUENCE");
  expect(gate.admit(key("a", 0, 6, 499)), false, false, "STAMP_WENT_BACKWARDS");
  EXPECT_EQ(gate.active()->sequence, 5u);
  EXPECT_EQ(*gate.active()->stamp_ns, 500);
  expect(gate.admit(key("a", 0, 6, 500)), true, false, "IN_ORDER");
}

TEST(FrameKeyGate, NewerSessionResetsOlderSessionIsRejected) {
  FrameKeyGate gate;
  gate.admit(key("a", 3, 50, 500));
  expect(gate.admit(key("a", 2, 51, 600)), false, false, "OLD_SESSION");
  // A new session may restart time and sequence: that is what a seek or loop looks like.
  expect(gate.admit(key("a", 4, 1, 10)), true, true, "NEW_SESSION");
  expect(gate.admit(key("a", 3, 99, 900)), false, false, "OLD_SESSION");
  expect(gate.admit(key("a", 4, 2, 11)), true, false, "IN_ORDER");
}

TEST(FrameKeyGate, NewSourceRetiresThePreviousOneUntilRestart) {
  FrameKeyGate gate;
  gate.admit(key("a", 0, 10, 500));
  expect(gate.admit(key("b", 0, 1, 5)), true, true, "NEW_SOURCE");
  EXPECT_EQ(gate.retired_sources(), 1u);
  // Whatever the key, the retired source never comes back.
  expect(gate.admit(key("a", 0, 11, 600)), false, false, "RETIRED_SOURCE");
  expect(gate.admit(key("a", 9, 1, 600)), false, false, "RETIRED_SOURCE");
  EXPECT_EQ(gate.active()->source, "b");
  expect(gate.admit(key("c", 0, 1, 5)), true, true, "NEW_SOURCE");
  expect(gate.admit(key("b", 0, 2, 6)), false, false, "RETIRED_SOURCE");
  EXPECT_EQ(gate.retired_sources(), 2u);
}

TEST(FrameKeyGate, InvalidKeysNeverReplaceOrRetireTheActiveSource) {
  FrameKeyGate gate;
  expect(gate.admit(key("", 0, 1)), false, false, "INVALID_SOURCE_ID");
  EXPECT_FALSE(gate.active());
  gate.admit(key("a", 1, 5, 500));
  expect(gate.admit({"b", 0, 1, std::nullopt}), false, false, "INVALID_STAMP");
  expect(gate.admit(key("b", 0, 0)), false, false, "INVALID_SEQUENCE");
  expect(gate.admit(key("", 2, 9)), false, false, "INVALID_SOURCE_ID");
  EXPECT_EQ(gate.retired_sources(), 0u);
  EXPECT_EQ(gate.active()->source, "a");
  EXPECT_EQ(gate.active()->session, 1u);
  expect(gate.admit(key("a", 1, 6, 500)), true, false, "IN_ORDER");
  // The source that only sent invalid keys was never retired and may still take over.
  expect(gate.admit(key("b", 0, 1, 1)), true, true, "NEW_SOURCE");
}
