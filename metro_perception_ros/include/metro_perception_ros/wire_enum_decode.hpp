#pragma once

#include <cstdint>

#include "metro_perception_core/types.hpp"

namespace metro_perception_ros {

inline metro_perception_core::AnalysisStatus decode_analysis_status(std::uint8_t value) {
  using metro_perception_core::AnalysisStatus;
  switch (value) {
    case static_cast<std::uint8_t>(AnalysisStatus::OK):
      return AnalysisStatus::OK;
    case static_cast<std::uint8_t>(AnalysisStatus::NOT_IMPLEMENTED):
      return AnalysisStatus::NOT_IMPLEMENTED;
    case static_cast<std::uint8_t>(AnalysisStatus::BAD_INPUT):
      return AnalysisStatus::BAD_INPUT;
    case static_cast<std::uint8_t>(AnalysisStatus::TF_UNAVAILABLE):
      return AnalysisStatus::TF_UNAVAILABLE;
    case static_cast<std::uint8_t>(AnalysisStatus::INVALID_GEOMETRY):
      return AnalysisStatus::INVALID_GEOMETRY;
    default:
      return AnalysisStatus::BAD_INPUT;
  }
}

inline metro_perception_core::CalibrationTrust decode_calibration_trust(std::uint8_t value) {
  using metro_perception_core::CalibrationTrust;
  switch (value) {
    case static_cast<std::uint8_t>(CalibrationTrust::UNKNOWN):
      return CalibrationTrust::UNKNOWN;
    case static_cast<std::uint8_t>(CalibrationTrust::ASSUMED):
      return CalibrationTrust::ASSUMED;
    case static_cast<std::uint8_t>(CalibrationTrust::VERIFIED):
      return CalibrationTrust::VERIFIED;
    default:
      return CalibrationTrust::UNKNOWN;
  }
}

}  // namespace metro_perception_ros
