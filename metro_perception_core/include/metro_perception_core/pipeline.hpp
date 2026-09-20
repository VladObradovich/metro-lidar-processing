#pragma once
#include "metro_perception_core/config.hpp"
#include "metro_perception_core/types.hpp"
namespace metro_perception_core {
class PerceptionPipeline {
 public:
  explicit PerceptionPipeline(AlgorithmConfig config = {});
  FrameResult process(const FrameInput& frame) const;
  void reset() {}  // No history until geometry is implemented.
 private:
  AlgorithmConfig config_;
};
}  // namespace metro_perception_core
