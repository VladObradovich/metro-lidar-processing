#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>

#include "metro_perception_interfaces/msg/frame_analysis.hpp"
#include "metro_perception_ros/pointcloud_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
using metro_perception_interfaces::msg::FrameAnalysis;
class PerceptionNode : public rclcpp::Node {
 public:
  explicit PerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("perception", options) {
    const auto limit = declare_parameter<int>("max_points", 2000000);
    if (limit <= 0 || limit > 10000000) {
      throw std::invalid_argument("Invalid max_points");
    }
    max_points_ = static_cast<std::size_t>(limit);
    pipeline_ = std::make_unique<metro_perception_core::PerceptionPipeline>(
        metro_perception_core::AlgorithmConfig{max_points_});
    std::random_device random;
    std::ostringstream id;
    for (int i = 0; i < 4; ++i) {
      id << std::hex << std::setw(8) << std::setfill('0') << random();
    }
    source_ = id.str();
    const auto reliability = declare_parameter<std::string>("input_reliability", "best_effort");
    if (reliability != "best_effort" && reliability != "reliable") {
      throw std::invalid_argument("input_reliability: best_effort or reliable");
    }
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (reliability == "reliable") {
      qos.reliable();
    }
    publisher_ = create_publisher<FrameAnalysis>("~/output/analysis", 1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input/points", qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          const auto start = std::chrono::steady_clock::now();
          const auto stamp = std::int64_t(message->header.stamp.sec) * 1000000000LL +
                             message->header.stamp.nanosec;
          if (received_ > 0 && stamp < previous_stamp_) {
            ++session_;
            pipeline_->reset();
          }
          previous_stamp_ = stamp;
          ++received_;
          const auto frame = metro_perception_ros::process_cloud(*message, *pipeline_, max_points_);
          if (frame.status == metro_perception_core::AnalysisStatus::BAD_INPUT) {
            ++rejected_;
          }
          FrameAnalysis output;
          output.header = message->header;  // No TF applied in scaffold.
          output.source_instance_id = source_;
          output.session_id = session_;
          output.frame_sequence = received_;
          output.processing_status = static_cast<std::uint8_t>(frame.status);
          output.reason = frame.reason;
          output.received_frames = received_;
          output.processed_frames = received_ - rejected_;
          output.rejected_frames = rejected_;
          output.processing_age_ms =
              std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                  .count();
          publisher_->publish(output);
        });
    RCLCPP_WARN(get_logger(), "Scaffold: decoder active, detector/TF/worker NOT_IMPLEMENTED");
  }

 private:
  std::size_t max_points_;
  std::unique_ptr<metro_perception_core::PerceptionPipeline> pipeline_;
  std::string source_;
  std::uint64_t session_{0}, received_{0}, rejected_{0};
  std::int64_t previous_stamp_{0};
  rclcpp::Publisher<FrameAnalysis>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PerceptionNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("perception"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
