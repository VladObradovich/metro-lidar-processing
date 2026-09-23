#include <chrono>
#include <memory>

#include "metro_perception_ros/assessment_monitor.hpp"
#include "rclcpp/rclcpp.hpp"
using metro_perception_interfaces::msg::FrameAnalysis;
using metro_perception_interfaces::msg::PathAssessment;
using metro_perception_ros::AssessmentMonitor;
class ObstacleMonitor : public rclcpp::Node {
 public:
  ObstacleMonitor() : Node("obstacle_monitor") {
    monitor_ = std::make_unique<AssessmentMonitor>(declare_parameter<double>("timeout_s", 0.5));
    publisher_ = create_publisher<PathAssessment>("~/output/assessment", 1);
    subscription_ = create_subscription<FrameAnalysis>(
        "~/input/analysis", 1, [this](FrameAnalysis::ConstSharedPtr frame) {
          const auto decision = monitor_->on_analysis(*frame, AssessmentMonitor::Clock::now());
          if (!decision.accepted) {
            // The last valid result and its age stay untouched; the watchdog still applies.
            RCLCPP_WARN_THROTTLE(
                get_logger(), steady_clock_, 2000, "Rejected analysis %s/%lu/%lu: %s",
                frame->source_instance_id.c_str(), static_cast<unsigned long>(frame->session_id),
                static_cast<unsigned long>(frame->frame_sequence), decision.reason);
            return;
          }
          if (decision.reset) {
            RCLCPP_INFO(get_logger(), "Monitor reset (%s): source %s session %lu", decision.reason,
                        frame->source_instance_id.c_str(),
                        static_cast<unsigned long>(frame->session_id));
          }
          publish();
        });
    // Steady-clock heartbeat: works without /clock and never advances the observation stamp.
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish(); });
  }

 private:
  void publish() { publisher_->publish(monitor_->output(AssessmentMonitor::Clock::now())); }

  std::unique_ptr<AssessmentMonitor> monitor_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // Log throttling must not depend on /clock.
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
