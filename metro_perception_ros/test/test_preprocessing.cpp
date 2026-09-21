#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>

#include "metro_perception_ros/pointcloud_adapter.hpp"
#include "metro_perception_ros/preprocessing.hpp"
using namespace metro_perception_ros;
using namespace metro_perception_core;
namespace {
geometry_msgs::msg::TransformStamped transform(int sec, double x) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "base_link";
  t.child_frame_id = "lidar";
  t.header.stamp.sec = sec;
  t.transform.rotation.w = 1;
  t.transform.translation.x = x;
  return t;
}
std_msgs::msg::Header header(int sec) {
  std_msgs::msg::Header h;
  h.frame_id = "lidar";
  h.stamp.sec = sec;
  return h;
}
}  // namespace
TEST(Transform, UsesMeasurementTimeNotLatestAndRejectsExtrapolation) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PreprocessingConfig c;
  c.source_frame = "lidar";
  c.calibration_verified = true;
  buffer.setTransform(transform(10, 1), "test", false);
  buffer.setTransform(transform(20, 11), "test", false);
  const auto context = resolve_context(header(15), c, buffer);
  ASSERT_TRUE(context.transform_available);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.translation.x, 6);
  EXPECT_FALSE(resolve_context(header(0), c, buffer).transform_available);
  EXPECT_FALSE(resolve_context(header(9), c, buffer).transform_available);
  EXPECT_FALSE(resolve_context(header(21), c, buffer).transform_available);
  auto h = header(15);
  h.frame_id = "wrong";
  EXPECT_FALSE(resolve_context(h, c, buffer).transform_available);
}
TEST(Transform, StaticTransformWorksWithoutClockAndRemainsUnverified) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PreprocessingConfig c;
  c.source_frame = "lidar";
  EXPECT_FALSE(resolve_context(header(123), c, buffer).transform_available);
  buffer.setTransform(transform(0, 3), "test", true);
  const auto context = resolve_context(header(123), c, buffer);
  EXPECT_TRUE(context.transform_available);
  EXPECT_FALSE(context.calibration_verified);
  EXPECT_DOUBLE_EQ(context.sensor_origin.x, 3);
}
TEST(Profile, LoadsAndRejectsUnsafeConfiguration) {
  const auto path = std::string("/tmp/metro-profile-test-") + std::to_string(getpid()) + ".yaml";
  auto save = [&](const std::string& tail) {
    std::ofstream file(path);
    file << "source_frame: lidar\ntarget_frame: base_link\n" << tail;
  };
  save(
      "calibration_verified: false\ntranslation_m: [1, 2, 3]\nrotation_rpy_rad: [0, 0, "
      "1.5707963267948966]\n");
  const auto c = load_preprocessing(path);
  ASSERT_TRUE(c.has_static_transform);
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  buffer.setTransform(c.static_transform, "test", true);
  FrameInput frame;
  frame.points = {{0, -10, 0}};
  frame.context = resolve_context(header(123), c, buffer);
  const auto result = PerceptionPipeline(c.algorithm).process(frame);
  ASSERT_EQ(result.preprocessed.geometry_points.size(), 1u);
  EXPECT_NEAR(result.preprocessed.geometry_points[0].point.x, 11, 1e-6);
  EXPECT_EQ(result.reason, "CALIBRATION_UNVERIFIED");
  save("calibration_verified: true\ntranslation_m: null\n");
  EXPECT_THROW(load_preprocessing(path), std::invalid_argument);
  save("calibration_verified: false\ntranslation_m: [0, .nan, 0]\nrotation_rpy_rad: [0, 0, 0]\n");
  EXPECT_THROW(load_preprocessing(path), std::invalid_argument);
  save("calibration_verified: false\nrotation_rpy_rad: [0, 0, 0]\n");
  EXPECT_THROW(load_preprocessing(path), std::invalid_argument);
  std::remove(path.c_str());
}

TEST(Transform, SameFrameDoesNotRequireTfTree) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PreprocessingConfig c;
  c.source_frame = "lidar_livox";
  c.target_frame = "lidar_livox";
  auto h = header(123);
  h.frame_id = "lidar_livox";
  const auto context = resolve_context(h, c, buffer);
  EXPECT_TRUE(context.transform_available);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.rotation[0], 1);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.rotation[4], 1);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.rotation[8], 1);
  EXPECT_DOUBLE_EQ(context.sensor_origin.x, 0);
}

TEST(Profile, DefaultForwardSectorUsesExplicitAssumption) {
  auto c = load_preprocessing("");
  EXPECT_EQ(c.source_frame, "hesai_lidar");
  EXPECT_EQ(c.target_frame, "lidar_assumed");
  EXPECT_TRUE(c.allow_unverified_calibration);
  EXPECT_FALSE(c.calibration_verified);
  EXPECT_TRUE(c.has_static_transform);
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  buffer.setTransform(c.static_transform, "default_profile", true);
  auto h = header(123);
  h.frame_id = "hesai_lidar";
  FrameInput input;
  input.points = {{0, -10, 0}};
  input.context = resolve_context(h, c, buffer);
  const auto result = PerceptionPipeline(c.algorithm).process(input);
  EXPECT_EQ(result.status, AnalysisStatus::NOT_IMPLEMENTED);
  EXPECT_TRUE(result.preprocessed.transform_applied);
  EXPECT_EQ(result.calibration_trust, CalibrationTrust::ASSUMED);
  ASSERT_EQ(result.preprocessed.geometry_points.size(), 1u);
  EXPECT_NEAR(result.preprocessed.geometry_points[0].point.x, 10, 1e-8);
  EXPECT_DOUBLE_EQ(result.preprocessed.sensor_origin.z, 0);
  h.frame_id = "different_sensor";
  EXPECT_FALSE(resolve_context(h, c, buffer).transform_available);
  input.context.allow_unverified_calibration = false;
  EXPECT_EQ(PerceptionPipeline(c.algorithm).process(input).reason, "CALIBRATION_UNVERIFIED");
}
