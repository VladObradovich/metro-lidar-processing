#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "metro_perception_core/temporal_monitor.hpp"
#include "metro_perception_ros/measurement_time.hpp"
#include "metro_perception_ros/pointcloud_adapter.hpp"
#include "metro_perception_ros/preprocessing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace {

std::string json_string(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char ch : value) {
    if (ch == '"' || ch == '\\') {
      out << '\\' << ch;
    } else if (ch < 32) {
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch);
    } else {
      out << ch;
    }
  }
  out << '"';
  return out.str();
}

const char* state_name(metro_perception_core::State state) {
  switch (state) {
    case metro_perception_core::State::UNKNOWN:
      return "UNKNOWN";
    case metro_perception_core::State::OBSTACLE:
      return "OBSTACLE";
    case metro_perception_core::State::NO_OBSTACLE_DETECTED:
      return "NO_OBSTACLE_DETECTED";
  }
  return "UNKNOWN";
}

void restore_static_transforms(
    tf2_ros::Buffer& buffer,
    const std::vector<geometry_msgs::msg::TransformStamped>& bag_static_transforms,
    const metro_perception_ros::PreprocessingConfig& config,
    const metro_perception_ros::SourceFrameBinding& source_binding) {
  for (const auto& transform : bag_static_transforms) {
    buffer.setTransform(transform, "bag_tf_static", true);
  }
  if (const auto transform =
          metro_perception_ros::resolved_static_transform(config, source_binding)) {
    buffer.setTransform(*transform, "sensor_profile", true);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 6) {
    std::cerr << "Usage: evaluate_bag BAG TOPIC OUTPUT.jsonl [SENSOR_PROFILE.yaml] [MAX_POINTS]\n";
    return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
  }

  try {
    if (std::filesystem::exists(argv[3])) {
      throw std::runtime_error("Output already exists");
    }

    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    bool found_points = false;
    bool has_tf = false;
    bool has_tf_static = false;
    for (const auto& topic : reader.get_all_topics_and_types()) {
      if (topic.name == argv[2] && topic.type == "sensor_msgs/msg/PointCloud2") {
        found_points = true;
      } else if (topic.name == "/tf" && topic.type == "tf2_msgs/msg/TFMessage") {
        has_tf = true;
      } else if (topic.name == "/tf_static" && topic.type == "tf2_msgs/msg/TFMessage") {
        has_tf_static = true;
      }
    }
    if (!found_points) {
      throw std::runtime_error("Requested PointCloud2 topic not found");
    }

    rosbag2_storage::StorageFilter filter;
    filter.topics = {argv[2]};
    if (has_tf) filter.topics.push_back("/tf");
    if (has_tf_static) filter.topics.push_back("/tf_static");
    reader.set_filter(filter);

    std::ofstream output(argv[3]);
    if (!output) {
      throw std::runtime_error("Cannot create output");
    }

    auto config = metro_perception_ros::load_preprocessing(argc >= 5 ? argv[4] : "");
    if (argc == 6) {
      const std::string value(argv[5]);
      std::size_t consumed = 0;
      const auto max_points = std::stoull(value, &consumed);
      if (consumed != value.size() || max_points == 0 || max_points > 10000000) {
        throw std::invalid_argument("MAX_POINTS must be in [1, 10000000]");
      }
      config.algorithm.max_points = static_cast<std::size_t>(max_points);
      config.algorithm.validate();
    }

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> point_serializer;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer;
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    metro_perception_ros::SourceFrameBinding source_binding;
    std::vector<geometry_msgs::msg::TransformStamped> bag_static_transforms;
    restore_static_transforms(buffer, bag_static_transforms, config, source_binding);

    metro_perception_core::PerceptionPipeline pipeline(config.algorithm);
    metro_perception_core::TemporalMonitor monitor;
    std::uint64_t sequence = 0;
    std::uint64_t session = 0;
    std::optional<std::int64_t> previous_stamp;

    while (reader.has_next()) {
      const auto bag_message = reader.read_next();

      if (bag_message->topic_name == "/tf" || bag_message->topic_name == "/tf_static") {
        tf2_msgs::msg::TFMessage tf_message;
        rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
        tf_serializer.deserialize_message(&serialized, &tf_message);
        const bool is_static = bag_message->topic_name == "/tf_static";
        for (const auto& transform : tf_message.transforms) {
          buffer.setTransform(transform, is_static ? "bag_tf_static" : "bag_tf", is_static);
          if (is_static) bag_static_transforms.push_back(transform);
        }
        continue;
      }

      if (bag_message->topic_name != argv[2]) continue;

      const auto start = std::chrono::steady_clock::now();
      std::int64_t stamp = 0;
      metro_perception_core::FrameResult result;
      try {
        sensor_msgs::msg::PointCloud2 cloud;
        rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
        point_serializer.deserialize_message(&serialized, &cloud);
        const auto measurement_time =
            metro_perception_ros::decode_measurement_time_ns(cloud.header.stamp);
        stamp = measurement_time.value_or(0);

        metro_perception_core::FrameContext context;
        if (measurement_time) {
          if (previous_stamp && stamp < *previous_stamp) {
            ++session;
            pipeline.reset();
            monitor.reset();
            source_binding.reset();
            buffer.clear();
            restore_static_transforms(buffer, bag_static_transforms, config, source_binding);
          }
          previous_stamp = stamp;

          metro_perception_ros::bind_source_frame(config, source_binding, cloud.header.frame_id);
          if (const auto transform =
                  metro_perception_ros::resolved_static_transform(config, source_binding)) {
            buffer.setTransform(*transform, "sensor_profile", true);
          }
          context = metro_perception_ros::resolve_context(cloud.header, config, source_binding,
                                                          buffer);
        }

        result = metro_perception_ros::process_cloud(cloud, pipeline, config.algorithm.max_points,
                                                     context);
      } catch (const std::exception&) {
        result.status = metro_perception_core::AnalysisStatus::BAD_INPUT;
        result.reason = "DESERIALIZATION_ERROR";
      }

      const auto assessment = monitor.update(result, stamp);
      const double elapsed =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
              .count();

      output << "{\"schema_version\":1,\"mode\":\"a02\",\"bag_id\":";
      output << json_string(argv[1]);
      output << ",\"session_id\":" << session;
      output << ",\"frame_sequence\":" << ++sequence;
      output << ",\"measurement_stamp_ns\":" << stamp;
      output << ",\"bag_stamp_ns\":" << bag_message->time_stamp;
      output << ",\"state\":" << json_string(state_name(assessment.state));
      output << ",\"reason\":" << json_string(assessment.reason);
      output << ",\"distance_m\":";
      if (assessment.distance_valid && std::isfinite(assessment.distance_m)) {
        output << std::setprecision(17) << assessment.distance_m;
      } else {
        output << "null";
      }
      output << ",\"distance_valid\":" << (assessment.distance_valid ? "true" : "false")
             << ",\"candidate_count\":" << result.candidates.size()
             << ",\"evaluation_region_valid\":"
             << (result.evaluation_region_valid ? "true" : "false")
             << ",\"processing_status\":" << static_cast<unsigned>(result.status)
             << ",\"calibration_trust\":" << static_cast<unsigned>(result.calibration_trust)
             << ",\"transform_applied\":"
             << (result.preprocessed.transform_applied ? "true" : "false")
             << ",\"calibration_verified\":" << (config.calibration_verified ? "true" : "false")
             << ",\"calibration_assumed\":"
             << (config.allow_unverified_calibration && !config.calibration_verified ? "true"
                                                                                     : "false")
             << ",\"geometry_point_count\":" << result.preprocessed.geometry_points.size()
             << ",\"detection_point_count\":" << result.preprocessed.detection_indices.size()
             << ",\"invalid_point_count\":" << result.preprocessed.invalid_points
             << ",\"blind_point_count\":" << result.preprocessed.blind_points
             << ",\"outside_roi_point_count\":" << result.preprocessed.outside_roi_points
             << ",\"processing_ms\":" << elapsed << "}\n";
      if (!output) {
        throw std::runtime_error("Failed to write results");
      }
    }

    output.close();
    if (!output) {
      throw std::runtime_error("Failed to close results");
    }
    if (sequence == 0) {
      throw std::runtime_error("No frames processed");
    }

    std::cout << "A02 exported " << sequence << " frames\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
