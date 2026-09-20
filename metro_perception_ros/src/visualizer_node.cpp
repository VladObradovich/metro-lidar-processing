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
          // UI overlay uses latest transform, observation stamp remains in assessment.
          marker.ns = "status";
          marker.id = 0;
          marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.orientation.w = 1.0;
          marker.pose.position.z = 1.0;
          marker.scale.z = 0.35;
          marker.color.r = 1.0;
          marker.color.g = 0.7;
          marker.color.a = 1.0;
          marker.lifetime.sec = 1;
          marker.text = "UNKNOWN: " + assessment->reason;
          visualization_msgs::msg::MarkerArray output;
          output.markers.push_back(marker);
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
