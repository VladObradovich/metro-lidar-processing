#pragma once
#include <cstdint>
#include <vector>

#include "metro_perception_core/types.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace metro_perception_ros {
// Display cloud of the analysed points (target frame): x, y, z, a packed rgb for RViz's RGB8
// colour transformer and the label of metro_perception_core::label_points. Background points
// are grey, lighter when nearer; corridor points are green and obstacle points red.
sensor_msgs::msg::PointCloud2 make_labelled_cloud(const std_msgs::msg::Header& header,
                                                  const metro_perception_core::FrameResult& frame,
                                                  const std::vector<std::uint8_t>& labels);
}  // namespace metro_perception_ros
