#pragma once

#include <cstdint>
#include <optional>

#include "builtin_interfaces/msg/time.hpp"

namespace metro_perception_ros {

inline std::optional<std::int64_t> decode_measurement_time_ns(
    const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) return std::nullopt;
  const auto value = std::int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
  if (value <= 0) return std::nullopt;
  return value;
}

}  // namespace metro_perception_ros
