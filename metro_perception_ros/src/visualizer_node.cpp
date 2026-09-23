#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>

#include "metro_perception_interfaces/msg/path_assessment.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
using metro_perception_interfaces::msg::PathAssessment;
class Visualizer : public rclcpp::Node {
 public:
  Visualizer() : Node("visualizer") {
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);
    subscription_ = create_subscription<PathAssessment>(
        "~/input/assessment", 1, [this](PathAssessment::ConstSharedPtr assessment) {
          if (assessment->header.frame_id.empty()) {
            return;
          }
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = assessment->header.frame_id;
          marker.header.stamp = assessment->header.stamp;
          // UI overlay uses latest transform, observation stamp remains in assessment.
          marker.ns = "status";
          marker.id = 0;
          marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.orientation.w = 1.0;
          marker.pose.position.z = 1.0;
          marker.scale.z = 0.35;
          marker.color.r = assessment->state == PathAssessment::NO_OBSTACLE_DETECTED ? 0.2 : 1.0;
          marker.color.g = assessment->state == PathAssessment::OBSTACLE ? 0.2 : 0.8;
          marker.color.a = 1.0;
          marker.lifetime.sec = 1;
          std::ostringstream label;
          label << (assessment->state == PathAssessment::OBSTACLE ? "OBSTACLE"
                    : assessment->state == PathAssessment::NO_OBSTACLE_DETECTED
                        ? "NO_OBSTACLE_DETECTED"
                        : "UNKNOWN");
          if (assessment->distance_valid)
            label << " " << std::fixed << std::setprecision(1) << assessment->distance_m << " m";
          label << ": " << assessment->reason;
          if (assessment->stale) label << " (stale)";
          marker.text = label.str();
          visualization_msgs::msg::MarkerArray output;
          visualization_msgs::msg::Marker clear;
          clear.action = visualization_msgs::msg::Marker::DELETEALL;
          output.markers.push_back(clear);
          output.markers.push_back(marker);
          if (!assessment->stale) {
            for (const auto& object : assessment->confirmed_objects) {
              visualization_msgs::msg::Marker box;
              box.header = marker.header;
              box.ns = "candidate_bbox";
              box.id = static_cast<std::int32_t>(object.candidate_id);
              box.type = visualization_msgs::msg::Marker::CUBE;
              box.action = visualization_msgs::msg::Marker::ADD;
              box.pose = object.bbox.center;
              box.scale = object.bbox.size;
              box.scale.x = std::max(box.scale.x, 0.05);
              box.scale.y = std::max(box.scale.y, 0.05);
              box.scale.z = std::max(box.scale.z, 0.05);
              box.color.r = 1.0;
              box.color.g = 0.2;
              box.color.a = 0.5;
              box.lifetime.sec = 1;
              output.markers.push_back(box);
            }
          }
          publisher_->publish(output);
        });
  }

 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<PathAssessment>::SharedPtr subscription_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualizer>());
  rclcpp::shutdown();
  return 0;
}
