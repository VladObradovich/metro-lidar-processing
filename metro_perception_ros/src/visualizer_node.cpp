#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>

#include "metro_perception_core/temporal_monitor.hpp"
#include "metro_perception_interfaces/msg/path_assessment.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
using metro_perception_interfaces::msg::CorridorSegment;
using metro_perception_interfaces::msg::ObstacleCandidate;
using metro_perception_interfaces::msg::PathAssessment;
using visualization_msgs::msg::Marker;

namespace {
geometry_msgs::msg::Point point(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// Wireframe box following the corridor centreline from start to end on the ground.
bool corridor_edges(const CorridorSegment& segment, Marker& marker) {
  const double dx = segment.end.x - segment.start.x, dy = segment.end.y - segment.start.y;
  const double length = std::hypot(dx, dy);
  if (!std::isfinite(length) || length <= 0 || !std::isfinite(segment.width_m) ||
      !std::isfinite(segment.height_m) || segment.width_m <= 0 || segment.height_m <= 0) {
    return false;
  }
  const double ly = dx / length * segment.width_m / 2, lx = -dy / length * segment.width_m / 2;
  geometry_msgs::msg::Point c[8];
  const geometry_msgs::msg::Point* ends[2] = {&segment.start, &segment.end};
  for (int e = 0; e < 2; ++e) {
    for (int side = 0; side < 2; ++side) {
      const double s = side ? 1.0 : -1.0;
      for (int top = 0; top < 2; ++top) {
        c[e * 4 + side * 2 + top] = point(ends[e]->x + s * lx, ends[e]->y + s * ly,
                                          ends[e]->z + (top ? segment.height_m : 0.0));
      }
    }
  }
  const int edges[12][2] = {{0, 4}, {1, 5}, {2, 6}, {3, 7}, {0, 2}, {4, 6},
                            {1, 3}, {5, 7}, {0, 1}, {2, 3}, {4, 5}, {6, 7}};
  for (const auto& edge : edges) {
    marker.points.push_back(c[edge[0]]);
    marker.points.push_back(c[edge[1]]);
  }
  return true;
}

// Same rule as the monitor decision: the nearest candidate with a usable distance.
const ObstacleCandidate* nearest_candidate(const PathAssessment& assessment) {
  const ObstacleCandidate* nearest = nullptr;
  for (const auto& object : assessment.reported_objects) {
    if (!metro_perception_core::usable_distance(object.distance_valid, object.distance_m)) continue;
    if (!nearest || object.distance_m < nearest->distance_m) nearest = &object;
  }
  return nearest;
}
}  // namespace

class Visualizer : public rclcpp::Node {
 public:
  Visualizer() : Node("visualizer") {
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);
    subscription_ = create_subscription<PathAssessment>(
        "~/input/assessment", 1,
        [this](PathAssessment::ConstSharedPtr assessment) { on_assessment(*assessment); });
  }

 private:
  void on_assessment(const PathAssessment& assessment) {
    if (assessment.header.frame_id.empty()) return;
    visualization_msgs::msg::MarkerArray output;
    Marker clear;
    clear.action = Marker::DELETEALL;
    output.markers.push_back(clear);

    Marker status = base(assessment, "status", 0, Marker::TEXT_VIEW_FACING);
    // Ahead of the train and above the track, readable from a camera behind the lidar.
    status.pose.position.x = 12.0;
    status.pose.position.z = 2.5;
    status.scale.z = 0.5;
    status.color.r = assessment.state == PathAssessment::NO_OBSTACLE_DETECTED ? 0.2 : 1.0;
    status.color.g = assessment.state == PathAssessment::OBSTACLE ? 0.2 : 0.8;
    status.color.a = 1.0;
    std::ostringstream label;
    label << (assessment.state == PathAssessment::OBSTACLE               ? "OBSTACLE"
              : assessment.state == PathAssessment::NO_OBSTACLE_DETECTED ? "NO_OBSTACLE_DETECTED"
                                                                         : "UNKNOWN");
    if (assessment.distance_valid && std::isfinite(assessment.distance_m))
      label << " " << std::fixed << std::setprecision(1) << assessment.distance_m << " m";
    label << ": " << assessment.reason;
    if (assessment.stale) label << " (stale)";
    status.text = label.str();
    output.markers.push_back(status);

    // A stale result carries no objects or corridor; never redraw old boxes.
    if (!assessment.stale) {
      std::int32_t id = 0;
      for (const auto& segment : assessment.corridor) {
        Marker corridor = base(assessment, "corridor", id++, Marker::LINE_LIST);
        corridor.scale.x = 0.1;
        const bool usable = segment.geometry_valid && segment.coverage_valid;
        corridor.color.r = usable ? 0.2 : 1.0;
        corridor.color.g = usable ? 0.9 : 0.8;
        corridor.color.b = 0.2;
        corridor.color.a = 0.8;
        if (corridor_edges(segment, corridor)) output.markers.push_back(corridor);
      }
      const auto* nearest = nearest_candidate(assessment);
      for (const auto& object : assessment.reported_objects) {
        const bool is_nearest = &object == nearest;
        Marker box = base(assessment, "candidate_bbox",
                          static_cast<std::int32_t>(object.candidate_id), Marker::CUBE);
        box.pose = object.bbox.center;
        box.scale = object.bbox.size;
        box.scale.x = std::max(box.scale.x, 0.05);
        box.scale.y = std::max(box.scale.y, 0.05);
        box.scale.z = std::max(box.scale.z, 0.05);
        // Gauge-only evidence (object fixed in the route) is blue, motion evidence red.
        const auto has = [&](const char* reason) {
          return std::find(object.reasons.begin(), object.reasons.end(), reason) !=
                 object.reasons.end();
        };
        const bool gauge_only = has("GAUGE") && !has("MOTION");
        box.color.r = gauge_only ? 0.2 : 1.0;
        box.color.g = is_nearest ? 0.1 : 0.6;
        box.color.b = gauge_only ? 1.0 : 0.0;
        box.color.a = is_nearest ? 0.7 : 0.35;
        output.markers.push_back(box);
      }
      if (nearest) {
        Marker dot = base(assessment, "nearest_point", 0, Marker::SPHERE);
        dot.pose.position = nearest->nearest_point;
        dot.scale.x = dot.scale.y = dot.scale.z = 0.4;
        dot.color.r = 1.0;
        dot.color.a = 1.0;
        output.markers.push_back(dot);
      }
      // Confirmed tracks (G4): label with id and distance; a coasting track, not measured in
      // this frame, is drawn as a faint box at its predicted place.
      for (const auto& track : assessment.tracks) {
        if (!track.confirmed) continue;
        const auto id = static_cast<std::int32_t>(track.track_id);
        Marker text = base(assessment, "track_label", id, Marker::TEXT_VIEW_FACING);
        text.pose.position = track.bbox.center.position;
        text.pose.position.z += track.bbox.size.z / 2 + 0.4;
        text.scale.z = 0.6;
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
        std::ostringstream name;
        name << '#' << track.track_id << ' ' << std::fixed << std::setprecision(1)
             << track.distance_m << " m" << (track.coasting ? " (predicted)" : "");
        text.text = name.str();
        output.markers.push_back(text);
        if (!track.coasting) continue;
        Marker ghost = base(assessment, "track_coasting", id, Marker::CUBE);
        ghost.pose = track.bbox.center;
        ghost.scale = track.bbox.size;
        ghost.scale.x = std::max(ghost.scale.x, 0.05);
        ghost.scale.y = std::max(ghost.scale.y, 0.05);
        ghost.scale.z = std::max(ghost.scale.z, 0.05);
        ghost.color.r = 1.0;
        ghost.color.g = 0.5;
        ghost.color.a = 0.2;
        output.markers.push_back(ghost);
      }
    }
    publisher_->publish(output);
  }

  static Marker base(const PathAssessment& assessment, const char* ns, std::int32_t id,
                     std::int32_t type) {
    Marker marker;
    // UI overlay uses latest transform, observation stamp remains in assessment.
    marker.header = assessment.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime.sec = 1;
    return marker;
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<PathAssessment>::SharedPtr subscription_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualizer>());
  rclcpp::shutdown();
  return 0;
}
