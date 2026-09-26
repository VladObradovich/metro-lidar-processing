#pragma once
#include <cstdint>
#include <vector>

#include "metro_perception_core/config.hpp"
#include "metro_perception_core/types.hpp"

namespace metro_perception_core {
// Per-point labels of FrameResult::preprocessed.geometry_points for display.
enum PointLabel : std::uint8_t { kBackgroundPoint = 0, kCorridorPoint = 1, kObstaclePoint = 2 };

// Obstacle: inside the box of a candidate (with a 0.1 m margin). Corridor: within the vehicle
// envelope's width of the route (corridor_half_width_m) from just below the floor up to
// corridor_height_m, up to the end of the published corridor, so the track bed and rails show
// the path ahead. Without a floor estimate only obstacles are labelled.
std::vector<std::uint8_t> label_points(const AlgorithmConfig& config, const FrameResult& result);
}  // namespace metro_perception_core
