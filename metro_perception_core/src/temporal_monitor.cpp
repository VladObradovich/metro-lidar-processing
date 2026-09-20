#include "metro_perception_core/temporal_monitor.hpp"
namespace metro_perception_core {
Assessment TemporalMonitor::update(const FrameResult& frame, std::int64_t) {
  Assessment result;
  // R02/X03: do not infer NO_OBSTACLE_DETECTED from an empty candidate list.
  result.reason = frame.status == AnalysisStatus::OK ? "MONITOR_NOT_IMPLEMENTED" : frame.reason;
  return result;
}
Assessment TemporalMonitor::on_timeout() const {
  Assessment result;
  result.reason = "INPUT_PAUSED_OR_STOPPED";
  return result;
}
}  // namespace metro_perception_core
