#pragma once
#include <cstddef>
#include <stdexcept>
namespace metro_perception_core {
struct AlgorithmConfig {
  std::size_t max_points{2000000};
  void validate() const {
    if (max_points == 0 || max_points > 10000000) {
      throw std::invalid_argument("max_points must be in [1, 10000000]");
    }
  }
};
}  // namespace metro_perception_core
