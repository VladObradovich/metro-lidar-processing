#include <gtest/gtest.h>

#include <cstdint>

#include "metro_perception_core/types.hpp"
#include "metro_perception_ros/wire_enum_decode.hpp"

using metro_perception_core::AnalysisStatus;
using metro_perception_core::CalibrationTrust;
using metro_perception_ros::decode_analysis_status;
using metro_perception_ros::decode_calibration_trust;

TEST(WireEnumDecode, AcceptsKnownAnalysisStatuses) {
  EXPECT_EQ(decode_analysis_status(0), AnalysisStatus::OK);
  EXPECT_EQ(decode_analysis_status(1), AnalysisStatus::NOT_IMPLEMENTED);
  EXPECT_EQ(decode_analysis_status(2), AnalysisStatus::BAD_INPUT);
  EXPECT_EQ(decode_analysis_status(3), AnalysisStatus::TF_UNAVAILABLE);
  EXPECT_EQ(decode_analysis_status(4), AnalysisStatus::INVALID_GEOMETRY);
}

TEST(WireEnumDecode, UnknownAnalysisStatusFailsClosed) {
  EXPECT_EQ(decode_analysis_status(5), AnalysisStatus::BAD_INPUT);
  EXPECT_EQ(decode_analysis_status(255), AnalysisStatus::BAD_INPUT);
}

TEST(WireEnumDecode, AcceptsKnownCalibrationTrust) {
  EXPECT_EQ(decode_calibration_trust(0), CalibrationTrust::UNKNOWN);
  EXPECT_EQ(decode_calibration_trust(1), CalibrationTrust::ASSUMED);
  EXPECT_EQ(decode_calibration_trust(2), CalibrationTrust::VERIFIED);
}

TEST(WireEnumDecode, UnknownCalibrationTrustFailsClosed) {
  EXPECT_EQ(decode_calibration_trust(3), CalibrationTrust::UNKNOWN);
  EXPECT_EQ(decode_calibration_trust(255), CalibrationTrust::UNKNOWN);
}
