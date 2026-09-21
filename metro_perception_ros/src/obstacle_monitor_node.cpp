#include <chrono>
#include <cmath>

#include "metro_perception_core/temporal_monitor.hpp"
#include "metro_perception_interfaces/msg/frame_analysis.hpp"
#include "metro_perception_interfaces/msg/path_assessment.hpp"
#include "metro_perception_ros/measurement_time.hpp"
#include "metro_perception_ros/wire_enum_decode.hpp"
#include "rclcpp/rclcpp.hpp"
using metro_perception_interfaces::msg::FrameAnalysis;
using metro_perception_interfaces::msg::PathAssessment;
class ObstacleMonitor : public rclcpp::Node {
 public:
  ObstacleMonitor() : Node("obstacle_monitor") {
    timeout_ = declare_parameter<double>("timeout_s", 0.5);
    if (!std::isfinite(timeout_) || timeout_ <= 0) {
      throw std::invalid_argument("timeout_s must be positive");
    }
    output_.state = PathAssessment::UNKNOWN;
    output_.reason = "WAITING_FOR_INPUT";
    output_.stale = true;
    publisher_ = create_publisher<PathAssessment>("~/output/assessment", 1);
    subscription_ = create_subscription<FrameAnalysis>(
        "~/input/analysis", 1, [this](FrameAnalysis::ConstSharedPtr frame) {
          // R02: support explicit reset/source retirement before production use.
          if (seen_ && frame->source_instance_id == output_.source_instance_id &&
              (frame->session_id < output_.session_id ||
               (frame->session_id == output_.session_id &&
                frame->frame_sequence <= output_.frame_sequence))) {
            return;
          }
          if (!seen_ || frame->source_instance_id != output_.source_instance_id ||
              frame->session_id != output_.session_id) {
            monitor_.reset();
          }
          metro_perception_core::FrameResult result;
          result.status = metro_perception_ros::decode_analysis_status(frame->processing_status);
          result.reason = frame->reason;
          if (result.status == metro_perception_core::AnalysisStatus::BAD_INPUT &&
              frame->processing_status != FrameAnalysis::BAD_INPUT) {
            result.reason = "INVALID_PROCESSING_STATUS";
          }
          result.calibration_trust =
              metro_perception_ros::decode_calibration_trust(frame->calibration_trust);
          const auto stamp =
              metro_perception_ros::decode_measurement_time_ns(frame->header.stamp).value_or(0);
          const auto assessment = monitor_.update(result, stamp);
          output_.header = frame->header;
          output_.source_instance_id = frame->source_instance_id;
          output_.session_id = frame->session_id;
          output_.frame_sequence = frame->frame_sequence;
          output_.state = static_cast<std::uint8_t>(assessment.state);
          output_.reason = assessment.reason;
          output_.calibration_trust = static_cast<std::uint8_t>(assessment.calibration_trust);
          output_.distance_valid = false;
          output_.stale = false;
          processing_age_ms_ = frame->processing_age_ms;
          received_at_ = std::chrono::steady_clock::now();
          seen_ = true;
          publish();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish(); });
  }

 private:
  void publish() {
    if (seen_) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - received_at_).count();
      output_.result_age_ms = elapsed * 1000 + processing_age_ms_;
      if (output_.result_age_ms > timeout_ * 1000) {
        output_.state = PathAssessment::UNKNOWN;
        output_.reason = monitor_.on_timeout().reason;
        output_.stale = true;
        output_.distance_valid = false;
      }
    }
    publisher_->publish(output_);
  }
  bool seen_{false};
  double timeout_{0.5}, processing_age_ms_{0};
  std::chrono::steady_clock::time_point received_at_;
  metro_perception_core::TemporalMonitor monitor_;
  PathAssessment output_;
  rclcpp::Publisher<PathAssessment>::SharedPtr publisher_;
  rclcpp::Subscription<FrameAnalysis>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObstacleMonitor>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("monitor"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
