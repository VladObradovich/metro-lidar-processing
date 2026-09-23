#pragma once
#include <cstdint>
#include <deque>
#include <unordered_map>

#include "metro_perception_core/config.hpp"
#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
class GeometricDetector {
 public:
  void process(const AlgorithmConfig& config, FrameResult& result,
               std::int64_t measurement_time_ns);
  void reset() {
    history_.clear();
    last_stamp_ns_ = 0;
  }

 private:
  using RangeMap = std::unordered_map<std::int64_t, double>;
  std::deque<RangeMap> history_;
  std::int64_t last_stamp_ns_{0};
};
}  // namespace metro_perception_core
