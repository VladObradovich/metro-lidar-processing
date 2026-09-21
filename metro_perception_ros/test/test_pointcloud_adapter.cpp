#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "metro_perception_ros/pointcloud_adapter.hpp"
using metro_perception_ros::decode_cloud;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
PointCloud2 fixture(bool big, bool wide) {
  PointCloud2 msg;
  msg.header.stamp.sec = 1;
  msg.width = 1;
  msg.height = 2;
  msg.is_bigendian = big;
  const std::size_t size = wide ? 8 : 4;
  msg.point_step = 1 + 3 * size;  // Unaligned fields.
  msg.row_step = msg.point_step + 5;
  msg.data.resize(msg.row_step * msg.height, 0xee);
  const std::uint16_t one = 1;
  const bool host_big = *reinterpret_cast<const std::uint8_t*>(&one) == 0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    PointField field;
    field.name = std::string(1, "xyz"[axis]);
    field.offset = 1 + axis * size;
    field.datatype = wide ? PointField::FLOAT64 : PointField::FLOAT32;
    field.count = 1;
    msg.fields.push_back(field);
    for (std::size_t row = 0; row < 2; ++row) {
      const double value = 1 + row * 3 + axis;
      const float narrow = static_cast<float>(value);
      auto* target = msg.data.data() + row * msg.row_step + field.offset;
      if (wide) {
        std::memcpy(target, &value, size);
      } else {
        std::memcpy(target, &narrow, size);
      }
      if (big != host_big) {
        std::reverse(target, target + size);
      }
    }
  }
  std::reverse(msg.fields.begin(), msg.fields.end());
  return msg;
}
TEST(Adapter, SupportsPaddingUnalignedEndianFloat64AndMissingRing) {
  for (bool big : {false, true}) {
    for (bool wide : {false, true}) {
      auto frame = decode_cloud(fixture(big, wide));
      ASSERT_EQ(frame.points.size(), 2u);
      EXPECT_DOUBLE_EQ(frame.points[0].x, 1);
      EXPECT_DOUBLE_EQ(frame.points[1].x, 4);
      EXPECT_DOUBLE_EQ(frame.points[1].z, 6);
    }
  }
}
TEST(Adapter, RejectsMalformedLayoutsBeforeReadingOrAllocating) {
  auto msg = fixture(false, false);
  msg.data.pop_back();
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);
  msg = fixture(false, false);
  msg.fields[0].offset = msg.point_step;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);
  msg = fixture(false, false);
  msg.fields[0].count = 2;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);
  msg = fixture(false, false);
  msg.width = 0xffffffff;
  msg.height = 0xffffffff;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);

  msg = fixture(false, false);
  msg.fields.erase(std::remove_if(msg.fields.begin(), msg.fields.end(),
                                  [](const PointField& field) { return field.name == "z"; }),
                   msg.fields.end());
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);

  msg = fixture(false, false);
  msg.fields.push_back(*std::find_if(msg.fields.begin(), msg.fields.end(),
                                     [](const PointField& field) { return field.name == "x"; }));
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);

  msg = fixture(false, false);
  auto x = std::find_if(msg.fields.begin(), msg.fields.end(),
                        [](const PointField& field) { return field.name == "x"; });
  x->datatype = PointField::UINT32;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);

  msg = fixture(false, false);
  x = std::find_if(msg.fields.begin(), msg.fields.end(),
                   [](const PointField& field) { return field.name == "x"; });
  const auto y = std::find_if(msg.fields.begin(), msg.fields.end(),
                              [](const PointField& field) { return field.name == "y"; });
  x->offset = y->offset;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);
}

TEST(Adapter, RejectsInvalidMeasurementTimestamps) {
  auto msg = fixture(false, false);
  msg.header.stamp.sec = 0;
  msg.header.stamp.nanosec = 0;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);

  msg = fixture(false, false);
  msg.header.stamp.sec = -1;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);

  msg = fixture(false, false);
  msg.header.stamp.nanosec = 1000000000u;
  EXPECT_THROW(decode_cloud(msg), std::invalid_argument);

  metro_perception_core::FrameContext context;
  context.transform_available = true;
  context.calibration_verified = true;
  const metro_perception_core::PerceptionPipeline pipeline;
  const auto result = metro_perception_ros::process_cloud(msg, pipeline, 10, context);
  EXPECT_EQ(result.status, metro_perception_core::AnalysisStatus::BAD_INPUT);
  EXPECT_EQ(result.reason, "INVALID_TIMESTAMP");
}

TEST(Adapter, NonFiniteCoordinatesAreFilteredWithoutPropagation) {
  auto msg = fixture(false, false);
  const auto x = std::find_if(msg.fields.begin(), msg.fields.end(),
                              [](const PointField& field) { return field.name == "x"; });
  ASSERT_NE(x, msg.fields.end());

  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(msg.data.data() + x->offset, &nan, sizeof(nan));

  metro_perception_core::FrameContext context;
  context.transform_available = true;
  context.calibration_verified = true;
  const metro_perception_core::PerceptionPipeline pipeline;

  auto result = metro_perception_ros::process_cloud(msg, pipeline, 10, context);
  EXPECT_EQ(result.status, metro_perception_core::AnalysisStatus::NOT_IMPLEMENTED);
  EXPECT_EQ(result.preprocessed.invalid_points, 1u);
  ASSERT_EQ(result.preprocessed.geometry_points.size(), 1u);
  EXPECT_TRUE(std::isfinite(result.preprocessed.geometry_points.front().point.x));

  const float inf = std::numeric_limits<float>::infinity();
  std::memcpy(msg.data.data() + msg.row_step + x->offset, &inf, sizeof(inf));
  result = metro_perception_ros::process_cloud(msg, pipeline, 10, context);
  EXPECT_EQ(result.status, metro_perception_core::AnalysisStatus::INVALID_GEOMETRY);
  EXPECT_EQ(result.reason, "EMPTY_GEOMETRY_ROI");
  EXPECT_EQ(result.preprocessed.invalid_points, 2u);
  EXPECT_TRUE(result.preprocessed.geometry_points.empty());
}

TEST(Adapter, EnforcesIndependentPointAndByteLimits) {
  const auto msg = fixture(false, false);
  EXPECT_THROW(decode_cloud(msg, 0), std::invalid_argument);
  EXPECT_THROW(decode_cloud(msg, metro_perception_ros::kMaxPointLimit + 1), std::invalid_argument);
  EXPECT_THROW(decode_cloud(msg, 10, 0), std::invalid_argument);
  EXPECT_THROW(decode_cloud(msg, 10, metro_perception_ros::kMaxCloudByteLimit + 1),
               std::invalid_argument);
  EXPECT_THROW(decode_cloud(msg, 1), std::invalid_argument);
  ASSERT_GT(msg.data.size(), 1u);
  EXPECT_THROW(decode_cloud(msg, 10, msg.data.size() - 1), std::invalid_argument);

  metro_perception_core::FrameContext context;
  context.transform_available = true;
  context.calibration_verified = true;
  const metro_perception_core::PerceptionPipeline pipeline;
  const auto result =
      metro_perception_ros::process_cloud(msg, pipeline, 10, context, msg.data.size() - 1);
  EXPECT_EQ(result.status, metro_perception_core::AnalysisStatus::BAD_INPUT);
  EXPECT_EQ(result.reason, "POINTCLOUD_BYTE_LIMIT_EXCEEDED");
}

TEST(Adapter, RejectedCloudsDoNotInvokeStatefulResolver) {
  const metro_perception_core::PerceptionPipeline pipeline;
  int calls = 0;
  auto resolve = [&] {
    ++calls;
    metro_perception_core::FrameContext context;
    context.transform_available = true;
    context.calibration_verified = true;
    return context;
  };
  auto msg = fixture(false, false);
  msg.data.clear();
  EXPECT_EQ(metro_perception_ros::process_cloud_with_context(msg, pipeline, 10, resolve).status,
            metro_perception_core::AnalysisStatus::BAD_INPUT);
  msg = fixture(false, false);
  metro_perception_ros::process_cloud_with_context(msg, pipeline, 1, resolve);
  metro_perception_ros::process_cloud_with_context(msg, pipeline, 10, resolve, 1);
  msg.header.stamp.sec = 0;
  metro_perception_ros::process_cloud_with_context(msg, pipeline, 10, resolve);
  msg = fixture(false, false);
  msg.width = 0;
  metro_perception_ros::process_cloud_with_context(msg, pipeline, 10, resolve);
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(
      metro_perception_ros::process_cloud_with_context(fixture(false, false), pipeline, 10, resolve)
          .status,
      metro_perception_core::AnalysisStatus::NOT_IMPLEMENTED);
  EXPECT_EQ(calls, 1);
}
