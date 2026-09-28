#pragma once
#include "metro_perception_core/config.hpp"
#include "metro_perception_core/detector.hpp"
#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
class PerceptionPipeline {
 public:
  explicit PerceptionPipeline(AlgorithmConfig config = {});
  FrameResult process(const FrameInput& frame);
  void reset() { detector_.reset(); }

 private:
  AlgorithmConfig config_;
  GeometricDetector detector_;
};
}  // namespace metro_perception_core
