#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>

#include "metro_perception_interfaces/msg/frame_analysis.hpp"
#include "metro_perception_ros/pointcloud_adapter.hpp"
#include "metro_perception_ros/preprocessing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
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
    config_ = metro_perception_ros::load_preprocessing(
        declare_parameter<std::string>("sensor_profile", ""));
    config_.algorithm.max_points = max_points_;
    pipeline_ = std::make_unique<metro_perception_core::PerceptionPipeline>(config_.algorithm);
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    if (config_.has_static_transform) {
      broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    }
    if (config_.has_static_transform && config_.source_frame != "*") {
      buffer_->setTransform(config_.static_transform, "sensor_profile", true);
      broadcaster_->sendTransform(config_.static_transform);
    }
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
          const bool unbound = config_.source_frame == "*";
          metro_perception_ros::bind_source_frame(config_, message->header.frame_id);
          if (unbound && config_.source_frame != "*")
            broadcaster_->sendTransform(config_.static_transform);
          // A simulation-clock reset may clear the TF buffer, including static entries.
          if (config_.has_static_transform && config_.source_frame != "*")
            buffer_->setTransform(config_.static_transform, "sensor_profile", true);
          const auto context =
              metro_perception_ros::resolve_context(message->header, config_, *buffer_);
          const auto frame =
              metro_perception_ros::process_cloud(*message, *pipeline_, max_points_, context);
          if (frame.status != metro_perception_core::AnalysisStatus::OK &&
              frame.status != metro_perception_core::AnalysisStatus::NOT_IMPLEMENTED) {
            ++rejected_;
          }
          FrameAnalysis output;
          output.header = message->header;
          if (frame.preprocessed.transform_applied) output.header.frame_id = config_.target_frame;
          output.transform_applied = frame.preprocessed.transform_applied;
          output.calibration_trust = static_cast<std::uint8_t>(frame.calibration_trust);
          output.calibration_verified = config_.calibration_verified;
          output.calibration_assumed =
              config_.allow_unverified_calibration && !config_.calibration_verified;
          output.geometry_point_count = frame.preprocessed.geometry_points.size();
          output.detection_point_count = frame.preprocessed.detection_indices.size();
          output.invalid_point_count = frame.preprocessed.invalid_points;
          output.blind_point_count = frame.preprocessed.blind_points;
          output.outside_roi_point_count = frame.preprocessed.outside_roi_points;
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
    RCLCPP_WARN(get_logger(),
                "A02 active; calibration_verified=%s. Detector/worker remain NOT_IMPLEMENTED",
                config_.calibration_verified ? "true" : "false");
  }

 private:
  metro_perception_ros::PreprocessingConfig config_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
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
