#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>

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
  SourceFrameBinding binding;
  buffer.setTransform(transform(10, 1), "test", false);
  buffer.setTransform(transform(20, 11), "test", false);
  const auto context = resolve_context(header(15), c, binding, buffer);
  ASSERT_TRUE(context.transform_available);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.translation.x, 6);
  EXPECT_FALSE(resolve_context(header(0), c, binding, buffer).transform_available);
  EXPECT_FALSE(resolve_context(header(-1), c, binding, buffer).transform_available);
  auto invalid_nanosec = header(15);
  invalid_nanosec.stamp.nanosec = 1000000000u;
  EXPECT_FALSE(resolve_context(invalid_nanosec, c, binding, buffer).transform_available);
  EXPECT_FALSE(resolve_context(header(9), c, binding, buffer).transform_available);
  EXPECT_FALSE(resolve_context(header(21), c, binding, buffer).transform_available);
  auto h = header(15);
  h.frame_id = "wrong";
  EXPECT_FALSE(resolve_context(h, c, binding, buffer).transform_available);
}
TEST(Transform, StaticTransformWorksWithoutClockAndRemainsUnverified) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PreprocessingConfig c;
  c.source_frame = "lidar";
  SourceFrameBinding binding;
  EXPECT_FALSE(resolve_context(header(123), c, binding, buffer).transform_available);
  buffer.setTransform(transform(0, 3), "test", true);
  const auto context = resolve_context(header(123), c, binding, buffer);
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
  SourceFrameBinding binding;
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  buffer.setTransform(c.static_transform, "test", true);
  FrameInput frame;
  frame.points = {{0, -10, 0}};
  frame.context = resolve_context(header(123), c, binding, buffer);
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
  save("calibration_verified: false\ndetector:\n  low_bump_min_prominence_m: -1\n");
  EXPECT_THROW(load_preprocessing(path), std::invalid_argument);
  save(
      "calibration_verified: false\ndetector:\n  static_half_width_m: 1.25\n"
      "  static_inner_half_width_m: 1.3\n");
  EXPECT_THROW(load_preprocessing(path), std::invalid_argument);
  {
    std::ofstream file(path);
    file << "source_frame_mode: bind_first\nsource_frame: null\ntarget_frame: base_link\n"
            "calibration_verified: true\ncalibration_source: measured\n"
            "translation_m: [0, 0, 0]\nrotation_rpy_rad: [0, 0, 0]\n";
  }
  EXPECT_THROW(load_preprocessing(path), std::invalid_argument);
  std::remove(path.c_str());
}

TEST(Profile, TemporalRuleIsReadAndValidated) {
  // The shipped profile carries the confirmation rule chosen for G4.
  const auto shipped = load_temporal_config("");
  EXPECT_EQ(shipped.confirm_hits, 2u);
  EXPECT_EQ(shipped.confirm_window, 3u);
  EXPECT_EQ(shipped.release_misses, 2u);
  EXPECT_TRUE(shipped.assumed_clear);
  EXPECT_DOUBLE_EQ(shipped.assumed_clear_min_range_m, 50.0);
  EXPECT_DOUBLE_EQ(shipped.far_confirm_from_m, 60.0);
  EXPECT_EQ(shipped.far_confirm_hits, 4u);
  EXPECT_EQ(shipped.far_confirm_window, 5u);
  EXPECT_DOUBLE_EQ(shipped.motion_gauge_speed_mps, 3.0);
  const auto path = std::string("/tmp/metro-temporal-test-") + std::to_string(getpid()) + ".yaml";
  auto save = [&](const std::string& text) { std::ofstream(path) << text; };
  // Without a temporal section the decision stays per frame.
  save("source_frame: lidar\n");
  const auto per_frame = load_temporal_config(path);
  EXPECT_EQ(per_frame.confirm_hits, 1u);
  EXPECT_EQ(per_frame.confirm_window, 1u);
  EXPECT_EQ(per_frame.release_misses, 1u);
  EXPECT_FALSE(per_frame.assumed_clear);
  save(
      "temporal:\n  confirm_hits: 3\n  confirm_window: 5\n  max_gap_s: 1.5\n"
      "  far_confirm_hits: 4\n  far_confirm_window: 5\n");
  const auto read = load_temporal_config(path);
  EXPECT_EQ(read.confirm_hits, 3u);
  EXPECT_EQ(read.confirm_window, 5u);
  EXPECT_DOUBLE_EQ(read.max_gap_s, 1.5);
  EXPECT_EQ(read.far_confirm_hits, 4u);
  EXPECT_EQ(read.far_confirm_window, 5u);
  save("temporal:\n  confirm_hits: 4\n  confirm_window: 3\n");
  EXPECT_THROW(load_temporal_config(path), std::invalid_argument);
  save("temporal:\n  assumed_clear: true\n  assumed_clear_min_range_m: -1\n");
  EXPECT_THROW(load_temporal_config(path), std::invalid_argument);
  save("temporal: 2\n");
  EXPECT_THROW(load_temporal_config(path), std::invalid_argument);
  std::remove(path.c_str());
}

TEST(Transform, SameFrameDoesNotRequireTfTree) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PreprocessingConfig c;
  c.source_frame = "lidar_livox";
  c.target_frame = "lidar_livox";
  SourceFrameBinding binding;
  EXPECT_TRUE(bind_source_frame(c, binding, "lidar_livox"));
  EXPECT_TRUE(binding.frame_id.empty());
  auto h = header(123);
  h.frame_id = "lidar_livox";
  const auto context = resolve_context(h, c, binding, buffer);
  EXPECT_TRUE(context.transform_available);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.rotation[0], 1);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.rotation[4], 1);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.rotation[8], 1);
  EXPECT_DOUBLE_EQ(context.sensor_origin.x, 0);
}

TEST(Profile, DefaultForwardSectorBindsRuntimeFramePerSession) {
  const auto c = load_preprocessing("");
  EXPECT_EQ(c.source_frame_mode, SourceFrameMode::BIND_FIRST);
  EXPECT_TRUE(c.source_frame.empty());
  EXPECT_EQ(c.target_frame, "lidar_assumed");
  EXPECT_TRUE(c.allow_unverified_calibration);
  EXPECT_FALSE(c.calibration_verified);
  EXPECT_TRUE(c.has_static_transform);
  // The organizers' 2.1 x 3.0 m envelope and the 1.075 m lidar height above the rail heads.
  EXPECT_DOUBLE_EQ(c.algorithm.corridor_half_width_m, 1.05);
  EXPECT_DOUBLE_EQ(c.algorithm.corridor_height_m, 3.0);
  EXPECT_DOUBLE_EQ(c.algorithm.static_half_width_m, 1.05);
  EXPECT_DOUBLE_EQ(c.algorithm.envelope_half_width_m, 1.05);
  EXPECT_DOUBLE_EQ(c.algorithm.sensor_height_above_rail_m, 1.075);
  EXPECT_DOUBLE_EQ(c.algorithm.route_smoothing, 0.5);
  EXPECT_DOUBLE_EQ(c.algorithm.far_gauge_max_x_m, 0.0);  // The far gauge is off.
  EXPECT_DOUBLE_EQ(c.algorithm.route_margin_per_m, 0.005);
  EXPECT_DOUBLE_EQ(c.algorithm.route_support_margin_m, 5.0);
  EXPECT_EQ(c.algorithm.ground_hold_frames, 3u);
  EXPECT_TRUE(c.algorithm.pole_rejection);
  EXPECT_DOUBLE_EQ(c.algorithm.ground_max_x_m, 90.0);  // The floor is not followed further.
  EXPECT_DOUBLE_EQ(c.algorithm.low_bump_min_prominence_m, 0.15);
  EXPECT_TRUE(c.algorithm.hanging_channel);

  SourceFrameBinding binding;
  EXPECT_FALSE(bind_source_frame(c, binding, ""));
  EXPECT_TRUE(bind_source_frame(c, binding, "private_pandar"));
  EXPECT_EQ(binding.frame_id, "private_pandar");
  EXPECT_TRUE(bind_source_frame(c, binding, "private_pandar"));
  EXPECT_FALSE(bind_source_frame(c, binding, "different_sensor"));

  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  const auto transform = resolved_static_transform(c, binding);
  ASSERT_TRUE(transform.has_value());
  EXPECT_EQ(transform->child_frame_id, "private_pandar");
  buffer.setTransform(*transform, "default_profile", true);

  auto h = header(123);
  h.frame_id = "private_pandar";
  FrameInput input;
  input.points = {{0, -10, 0}};
  input.context = resolve_context(h, c, binding, buffer);
  const auto result = PerceptionPipeline(c.algorithm).process(input);
  EXPECT_EQ(result.status, AnalysisStatus::INVALID_GEOMETRY);
  EXPECT_TRUE(result.preprocessed.transform_applied);
  EXPECT_EQ(result.calibration_trust, CalibrationTrust::ASSUMED);
  ASSERT_EQ(result.preprocessed.geometry_points.size(), 1u);
  EXPECT_NEAR(result.preprocessed.geometry_points[0].point.x, 10, 1e-8);

  h.frame_id = "different_sensor";
  EXPECT_FALSE(resolve_context(h, c, binding, buffer).transform_available);

  binding.reset();
  EXPECT_TRUE(bind_source_frame(c, binding, "different_sensor"));
  const auto rebound = resolved_static_transform(c, binding);
  ASSERT_TRUE(rebound.has_value());
  buffer.setTransform(*rebound, "default_profile", true);
  EXPECT_TRUE(resolve_context(h, c, binding, buffer).transform_available);
}

TEST(Transform, BoundedWaitAcceptsTransformArrivingAfterCloud) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PreprocessingConfig c;
  c.source_frame = "lidar";
  c.target_frame = "base_link";
  c.calibration_verified = true;
  SourceFrameBinding binding;

  std::thread producer([&buffer] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    buffer.setTransform(transform(123, 7), "delayed_test", false);
  });
  const auto context =
      resolve_context(header(123), c, binding, buffer, std::chrono::milliseconds(100));
  producer.join();

  ASSERT_TRUE(context.transform_available);
  EXPECT_DOUBLE_EQ(context.sensor_to_target.translation.x, 7);
}

TEST(Transform, BoundedWaitDoesNotFallBackToLatestTransform) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  PreprocessingConfig c;
  c.source_frame = "lidar";
  c.target_frame = "base_link";
  c.calibration_verified = true;
  SourceFrameBinding binding;
  buffer.setTransform(transform(200, 9), "future_test", false);

  const auto started = std::chrono::steady_clock::now();
  const auto context =
      resolve_context(header(123), c, binding, buffer, std::chrono::milliseconds(20));
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_FALSE(context.transform_available);
  EXPECT_LT(elapsed, std::chrono::milliseconds(250));
}
