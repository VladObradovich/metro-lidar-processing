#include "metro_perception_core/pipeline.hpp"
namespace metro_perception_core {
PerceptionPipeline::PerceptionPipeline(AlgorithmConfig config) : config_(config) {
  config_.validate();
}
FrameResult PerceptionPipeline::process(const FrameInput& frame) const {
  FrameResult result;
  if (frame.points.size() > config_.max_points) {
    result.status = AnalysisStatus::BAD_INPUT;
    result.reason = "POINT_LIMIT_EXCEEDED";
  } else if (frame.points.empty()) {
    result.status = AnalysisStatus::BAD_INPUT;
    result.reason = "EMPTY_INPUT";
  }
  // A03-A07: preprocessing in calibrated coordinates, ground, corridor,
  // candidates, clustering and raw-point validation. Until then NOT_IMPLEMENTED.
  return result;
}
}  // namespace metro_perception_core
