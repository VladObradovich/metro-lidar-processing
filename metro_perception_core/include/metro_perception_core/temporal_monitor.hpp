#pragma once
#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
class TemporalMonitor {
 public:
  Assessment update(const FrameResult& frame, std::int64_t measurement_time_ns);
  Assessment on_timeout() const;
  void reset() {}  // Add history together with X03, not before.
};
}  // namespace metro_perception_core
