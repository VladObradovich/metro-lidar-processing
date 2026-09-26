#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <thread>

#include "metro_perception_core/labels.hpp"
#include "metro_perception_interfaces/msg/frame_analysis.hpp"
#include "metro_perception_ros/labelled_cloud.hpp"
#include "metro_perception_ros/latest_frame_slot.hpp"
#include "metro_perception_ros/measurement_time.hpp"
#include "metro_perception_ros/pointcloud_adapter.hpp"
#include "metro_perception_ros/preprocessing.hpp"
#include "metro_perception_ros/session_gate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using metro_perception_interfaces::msg::FrameAnalysis;
using metro_perception_ros::resolve_context;

class PerceptionNode : public rclcpp::Node {
 public:
  explicit PerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("perception", options) {
    const auto point_limit = declare_parameter<std::int64_t>(
        "max_points", static_cast<std::int64_t>(metro_perception_ros::kDefaultMaxPoints));
    const auto byte_limit = declare_parameter<std::int64_t>(
        "max_cloud_bytes", static_cast<std::int64_t>(metro_perception_ros::kDefaultMaxCloudBytes));
    if (point_limit <= 0 ||
        point_limit > static_cast<std::int64_t>(metro_perception_ros::kMaxPointLimit)) {
      throw std::invalid_argument("Invalid max_points");
    }
    if (byte_limit <= 0 ||
        byte_limit > static_cast<std::int64_t>(metro_perception_ros::kMaxCloudByteLimit)) {
      throw std::invalid_argument("Invalid max_cloud_bytes");
    }
    max_points_ = static_cast<std::size_t>(point_limit);
    max_cloud_bytes_ = static_cast<std::size_t>(byte_limit);
    metro_perception_ros::validate_pointcloud_limits(max_points_, max_cloud_bytes_);

    const auto max_age_s = declare_parameter<double>("max_processing_age_s", 0.30);
    if (!std::isfinite(max_age_s) || max_age_s <= 0.0 || max_age_s > 10.0) {
      throw std::invalid_argument("max_processing_age_s must be in (0, 10]");
    }
    max_processing_age_ = std::chrono::duration<double>(max_age_s);

    const auto tf_wait_s = declare_parameter<double>("tf_wait_timeout_s", 0.05);
    if (!std::isfinite(tf_wait_s) || tf_wait_s < 0.0 || tf_wait_s > 1.0) {
      throw std::invalid_argument("tf_wait_timeout_s must be in [0, 1]");
    }
    tf_wait_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(tf_wait_s));

    config_ = metro_perception_ros::load_preprocessing(
        declare_parameter<std::string>("sensor_profile", ""));
    config_.algorithm.max_points = max_points_;
    pipeline_ = std::make_unique<metro_perception_core::PerceptionPipeline>(config_.algorithm);

    // Off by default: the profile TF stays private to perception. The demo turns it on so that
    // RViz can draw the input cloud in the target frame whatever the source frame is named.
    if (declare_parameter<bool>("publish_bound_transform", false))
      static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    if (const auto transform =
            metro_perception_ros::resolved_static_transform(config_, source_binding_)) {
      // Keep profile TF private to perception. Bringup owns optional publication to /tf_static.
      buffer_->setTransform(*transform, "sensor_profile", true);
    }

    std::random_device random;
    std::ostringstream id;
    for (int i = 0; i < 4; ++i) {
      id << std::hex << std::setw(8) << std::setfill('0') << random();
    }
    source_ = id.str();

    const auto reliability = declare_parameter<std::string>("input_reliability", "auto");
    if (reliability != "auto" && reliability != "best_effort" && reliability != "reliable") {
      throw std::invalid_argument("input_reliability: auto, best_effort or reliable");
    }

    publisher_ = create_publisher<FrameAnalysis>("~/output/analysis", 1);
    // Display only: the analysed points coloured by corridor and obstacle membership.
    if (declare_parameter<bool>("publish_labelled_cloud", false))
      labelled_publisher_ =
          create_publisher<sensor_msgs::msg::PointCloud2>("~/output/labelled_points", 1);
    // Clouds of several megabytes rarely arrive whole over best-effort DDS (one lost fragment
    // drops the message), so auto subscribes reliably, which also matches rosbag2 play. A
    // reliable reader cannot match a best-effort writer (a live driver): auto switches to best
    // effort while no cloud has arrived and every publisher of the topic is best effort.
    subscribe(reliability != "best_effort");
    if (reliability == "auto") {
      qos_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { adapt_reliability(); });
    }

    worker_ = std::thread([this] { worker_loop(); });

    RCLCPP_WARN(get_logger(),
                "Experimental geometric detector active; calibration_verified=%s. "
                "Ground support and straight corridor are required.",
                config_.calibration_verified ? "true" : "false");
  }

  ~PerceptionNode() override {
    pending_.stop();
    if (worker_.joinable()) worker_.join();
  }

 private:
  struct PendingFrame {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    std::uint64_t frame_sequence{0};
    std::uint64_t session_id{0};
    std::chrono::steady_clock::time_point received_at;
  };

  void subscribe(bool reliable) {
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (reliable) qos.reliable();
    reliable_input_ = reliable;
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input/points", qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          on_cloud(std::move(message));
        });
  }

  void adapt_reliability() {
    if (received_.load() > 0 || !reliable_input_) {
      qos_timer_->cancel();
      return;
    }
    const auto publishers = get_publishers_info_by_topic(subscription_->get_topic_name());
    if (publishers.empty()) return;
    for (const auto& info : publishers)
      if (info.qos_profile().reliability() != rclcpp::ReliabilityPolicy::BestEffort) return;
    RCLCPP_WARN(get_logger(),
                "Every publisher of %s is best effort; subscribing best effort. Large clouds "
                "may be lost in transport.",
                subscription_->get_topic_name());
    subscribe(false);
    qos_timer_->cancel();
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    const auto received_at = std::chrono::steady_clock::now();
    const auto stamp = metro_perception_ros::decode_measurement_time_ns(message->header.stamp);
    const auto sequence = received_.fetch_add(1) + 1;

    if (stamp && previous_stamp_ && *stamp < *previous_stamp_) {
      session_.advance();
      if (pending_.clear()) overwritten_.fetch_add(1);
    }
    if (stamp) previous_stamp_ = *stamp;

    PendingFrame next{std::move(message), sequence, session_.current(), received_at};
    const auto submit = pending_.submit(std::move(next));
    if (submit == metro_perception_ros::SubmitResult::OVERWROTE) {
      overwritten_.fetch_add(1);
    }
  }

  bool expired(const PendingFrame& frame) const {
    return std::chrono::steady_clock::now() - frame.received_at > max_processing_age_;
  }

  void worker_loop() {
    while (true) {
      auto item = pending_.wait_take();
      if (!item) return;
      process_frame(std::move(*item));
    }
  }

  void process_frame(PendingFrame work) {
    if (!worker_session_initialized_ || worker_session_ != work.session_id) {
      pipeline_->reset();
      source_binding_.reset();
      worker_session_ = work.session_id;
      worker_session_initialized_ = true;
    }

    const auto work_started = std::chrono::steady_clock::now();
    const double queue_age_ms =
        std::chrono::duration<double, std::milli>(work_started - work.received_at).count();
    if (expired(work)) {
      rejected_.fetch_add(1);
      return;
    }

    double tf_wait_ms = 0.0;
    const auto frame = metro_perception_ros::process_cloud_with_context(
        *work.message, *pipeline_, max_points_,
        [&] {
          metro_perception_ros::bind_source_frame(config_, source_binding_,
                                                  work.message->header.frame_id);
          // Simulation-clock resets can clear static entries in the private buffer.
          if (const auto transform =
                  metro_perception_ros::resolved_static_transform(config_, source_binding_)) {
            buffer_->setTransform(*transform, "sensor_profile", true);
            if (static_broadcaster_ && transform->child_frame_id != published_frame_) {
              static_broadcaster_->sendTransform(*transform);
              published_frame_ = transform->child_frame_id;
            }
          }
          const auto tf_wait_started = std::chrono::steady_clock::now();
          auto context = resolve_context(work.message->header, config_, source_binding_, *buffer_,
                                         tf_wait_timeout_);
          tf_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                                 tf_wait_started)
                           .count();
          return context;
        },
        max_cloud_bytes_);
    processed_.fetch_add(1);

    // A reset may happen while an old frame is being processed. Never publish it into the new
    // session, and never publish a result which is already too old to be actionable.
    if (work.session_id != session_.current() || expired(work)) {
      rejected_.fetch_add(1);
      return;
    }

    if (frame.status != metro_perception_core::AnalysisStatus::OK &&
        frame.status != metro_perception_core::AnalysisStatus::NOT_IMPLEMENTED) {
      rejected_.fetch_add(1);
    }

    FrameAnalysis output;
    output.header = work.message->header;
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
    output.session_id = work.session_id;
    output.frame_sequence = work.frame_sequence;
    output.processing_status = static_cast<std::uint8_t>(frame.status);
    output.reason = frame.reason;
    output.evaluation_region_valid = frame.evaluation_region_valid;
    output.evaluated_range_m = frame.evaluated_range_m;
    output.ego_motion_valid = frame.ego_motion_valid;
    output.ego_speed_mps = frame.ego_speed_mps;
    for (const auto& candidate : frame.candidates) {
      auto& item = output.candidates.emplace_back();
      item.candidate_id = candidate.id;
      item.bbox.center.position.x = candidate.center.x;
      item.bbox.center.position.y = candidate.center.y;
      item.bbox.center.position.z = candidate.center.z;
      item.bbox.center.orientation.w = 1.0;
      item.bbox.size.x = candidate.size.x;
      item.bbox.size.y = candidate.size.y;
      item.bbox.size.z = candidate.size.z;
      item.nearest_point.x = candidate.nearest_point.x;
      item.nearest_point.y = candidate.nearest_point.y;
      item.nearest_point.z = candidate.nearest_point.z;
      item.distance_m = candidate.distance_m;
      item.distance_valid = candidate.distance_valid;
      item.support_points = candidate.support_points;
      if (candidate.channels & metro_perception_core::ObstacleCandidate::kMotion)
        item.reasons.push_back("MOTION");
      if (candidate.channels & metro_perception_core::ObstacleCandidate::kGauge)
        item.reasons.push_back("GAUGE");
      if (candidate.edge) item.reasons.push_back("EDGE");
      item.reasons.push_back("OFFSET=" + std::to_string(candidate.closest_offset_m));
    }
    for (const auto& segment : frame.corridor) {
      auto& item = output.corridor.emplace_back();
      item.start.x = segment.start.x;
      item.start.y = segment.start.y;
      item.start.z = segment.start.z;
      item.end.x = segment.end.x;
      item.end.y = segment.end.y;
      item.end.z = segment.end.z;
      item.width_m = segment.width_m;
      item.height_m = segment.height_m;
      item.ground_plane = segment.ground_plane;
      item.geometry_valid = segment.geometry_valid;
      item.coverage_valid = segment.coverage_valid;
    }
    output.received_frames = received_.load();
    output.processed_frames = processed_.load();
    output.rejected_frames = rejected_.load();
    output.overwritten_frames = overwritten_.load();
    output.queue_age_ms = queue_age_ms;
    output.tf_wait_ms = tf_wait_ms;
    output.processing_age_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - work.received_at)
                                   .count();
    if (!session_.publish_if_current(work.session_id, [&] {
          if (expired(work)) {
            rejected_.fetch_add(1);
            return;
          }
          publisher_->publish(output);
          if (labelled_publisher_ && frame.preprocessed.transform_applied)
            labelled_publisher_->publish(metro_perception_ros::make_labelled_cloud(
                output.header, frame,
                metro_perception_core::label_points(config_.algorithm, frame)));
        })) {
      rejected_.fetch_add(1);
    }
  }

  metro_perception_ros::PreprocessingConfig config_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::size_t max_points_{0};
  std::size_t max_cloud_bytes_{0};
  std::chrono::duration<double> max_processing_age_{0.30};
  std::chrono::nanoseconds tf_wait_timeout_{std::chrono::milliseconds(50)};
  std::unique_ptr<metro_perception_core::PerceptionPipeline> pipeline_;
  metro_perception_ros::SourceFrameBinding source_binding_;

  std::string source_;
  metro_perception_ros::SessionGate session_;
  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> processed_{0};
  std::atomic<std::uint64_t> rejected_{0};
  std::atomic<std::uint64_t> overwritten_{0};
  std::optional<std::int64_t> previous_stamp_;

  metro_perception_ros::LatestFrameSlot<PendingFrame> pending_;
  std::thread worker_;
  std::uint64_t worker_session_{0};
  bool worker_session_initialized_{false};

  rclcpp::Publisher<FrameAnalysis>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr labelled_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr qos_timer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::string published_frame_;  // Source frame last sent to /tf_static; worker thread only.
  bool reliable_input_{true};
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
