#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "metro_perception_core/temporal_monitor.hpp"
#include "metro_perception_ros/bag_tf_replay.hpp"
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

std::size_t parse_limit(const char* text, const char* name) {
  const std::string value(text);
  std::size_t consumed = 0;
  const auto parsed = std::stoull(value, &consumed);
  if (consumed != value.size() || parsed > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(std::string(name) + " is not a valid size");
  }
  return static_cast<std::size_t>(parsed);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 8) {
    std::cerr << "Usage: evaluate_bag BAG TOPIC OUTPUT.jsonl [SENSOR_PROFILE.yaml] [MAX_POINTS] "
                 "[MAX_CLOUD_BYTES] [TF_LOOKAHEAD_S]\n";
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
    reader.set_filter(filter);

    std::ofstream output(argv[3]);
    if (!output) {
      throw std::runtime_error("Cannot create output");
    }

    auto config = metro_perception_ros::load_preprocessing(argc >= 5 ? argv[4] : "");
    std::size_t max_cloud_bytes = metro_perception_ros::kDefaultMaxCloudBytes;
    if (argc >= 6) config.algorithm.max_points = parse_limit(argv[5], "MAX_POINTS");
    if (argc >= 7) max_cloud_bytes = parse_limit(argv[6], "MAX_CLOUD_BYTES");
    metro_perception_ros::validate_pointcloud_limits(config.algorithm.max_points, max_cloud_bytes);
    // Diagnostics: rejected components and their reasons in every row.
    config.algorithm.record_rejected = std::getenv("METRO_DEBUG_COMPONENTS") != nullptr;
    // Diagnostics: odometry shift errors and speed tracker state in every row.
    config.algorithm.record_motion = std::getenv("METRO_DEBUG_MOTION") != nullptr;
    config.algorithm.validate();

    double lookahead_s = 0.05;
    if (argc == 8) {
      std::size_t consumed = 0;
      const std::string text(argv[7]);
      lookahead_s = std::stod(text, &consumed);
      if (consumed != text.size() || !std::isfinite(lookahead_s) || lookahead_s < 0.0 ||
          lookahead_s > 1.0) {
        throw std::invalid_argument("TF_LOOKAHEAD_S must be in [0, 1]");
      }
    }
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> point_serializer;
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    metro_perception_ros::SourceFrameBinding source_binding;
    std::vector<std::string> tf_topics;
    if (has_tf) tf_topics.push_back("/tf");
    if (has_tf_static) tf_topics.push_back("/tf_static");
    metro_perception_ros::BagTfReplay tf_replay(argv[1], tf_topics, buffer,
                                                static_cast<std::int64_t>(lookahead_s * 1e9));

    metro_perception_core::PerceptionPipeline pipeline(config.algorithm);
    metro_perception_core::TemporalMonitor monitor(
        metro_perception_ros::load_temporal_config(argc >= 5 ? argv[4] : ""));
    std::uint64_t sequence = 0;
    std::uint64_t session = 0;
    std::optional<std::int64_t> previous_stamp;

    while (reader.has_next()) {
      const auto bag_message = reader.read_next();

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

        if (measurement_time) {
          const bool new_session = previous_stamp && stamp < *previous_stamp;
          tf_replay.advance(bag_message->time_stamp, new_session);
          if (new_session) {
            ++session;
            pipeline.reset();
            monitor.reset();
            source_binding.reset();
          }
          previous_stamp = stamp;
        }

        result = metro_perception_ros::process_cloud_with_context(
            cloud, pipeline, config.algorithm.max_points,
            [&] {
              metro_perception_ros::bind_source_frame(config, source_binding,
                                                      cloud.header.frame_id);
              if (const auto transform =
                      metro_perception_ros::resolved_static_transform(config, source_binding)) {
                buffer.setTransform(*transform, "sensor_profile", true);
              }
              return metro_perception_ros::resolve_context(cloud.header, config, source_binding,
                                                           buffer);
            },
            max_cloud_bytes);
      } catch (const metro_perception_ros::TfReplayError&) {
        throw;  // Incomplete TF history must fail the export, not produce a success manifest.
      } catch (const std::exception&) {
        result.status = metro_perception_core::AnalysisStatus::BAD_INPUT;
        result.reason = "DESERIALIZATION_ERROR";
      }

      const auto assessment = monitor.update(result, stamp);
      const double elapsed =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
              .count();

      output << "{\"schema_version\":1,\"mode\":\""
             << (config.algorithm.background_history_frames ? "geometric_rolling" : "geometric_b0")
             << "\",\"bag_id\":";
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
             << ",\"candidate_count\":" << result.candidates.size();
      output << ",\"candidates\":[";
      for (std::size_t i = 0; i < result.candidates.size(); ++i) {
        if (i) output << ',';
        const auto& candidate = result.candidates[i];
        output << "{\"id\":" << candidate.id << ",\"distance_m\":" << std::setprecision(17)
               << candidate.distance_m << ",\"support_points\":" << candidate.support_points
               << ",\"center\":[" << candidate.center.x << ',' << candidate.center.y << ','
               << candidate.center.z << "],\"size\":[" << candidate.size.x << ','
               << candidate.size.y << ',' << candidate.size.z
               << "],\"channels\":" << static_cast<unsigned>(candidate.channels) << '}';
      }
      output << "]";
      output << ",\"evaluation_region_valid\":"
             << (result.evaluation_region_valid ? "true" : "false")
             << ",\"evaluated_range_m\":" << result.evaluated_range_m
             << ",\"ego_motion_valid\":" << (result.ego_motion_valid ? "true" : "false")
             << ",\"ego_speed_mps\":" << result.ego_speed_mps << ",\"ground_inliers\":"
             << (result.corridor.empty() ? 0u : result.corridor.front().ground_inliers)
             << ",\"ground_plane\":";
      if (result.corridor.empty()) {
        output << "null";
      } else {
        const auto& plane = result.corridor.front().ground_plane;
        output << '[' << plane[0] << ',' << plane[1] << ',' << plane[2] << ',' << plane[3] << ']';
      }
      output << ",\"tracks\":[";
      for (std::size_t i = 0; i < assessment.tracks.size(); ++i) {
        const auto& track = assessment.tracks[i];
        if (i) output << ',';
        output << "{\"id\":" << track.id
               << ",\"confirmed\":" << (track.confirmed ? "true" : "false")
               << ",\"coasting\":" << (track.coasting ? "true" : "false")
               << ",\"hits\":" << track.hits << ",\"age\":" << track.age_frames
               << ",\"distance_m\":" << std::setprecision(17) << track.distance_m << ",\"center\":["
               << track.center.x << ',' << track.center.y << ',' << track.center.z << "],\"size\":["
               << track.size.x << ',' << track.size.y << ',' << track.size.z
               << "],\"channels\":" << static_cast<unsigned>(track.channels) << '}';
      }
      output << ']';
      if (config.algorithm.record_rejected) {
        output << ",\"rejected\":[";
        for (std::size_t i = 0; i < result.rejected.size(); ++i) {
          const auto& item = result.rejected[i];
          output << (i ? "," : "") << "{\"reason\":" << json_string(item.reason) << ",\"center\":["
                 << item.center.x << ',' << item.center.y << ',' << item.center.z << "],\"size\":["
                 << item.size.x << ',' << item.size.y << ',' << item.size.z
                 << "],\"cells\":" << item.cells << ",\"points\":" << item.points
                 << ",\"channels\":" << static_cast<unsigned>(item.channels) << '}';
        }
        output << ']';
      }
      if (config.algorithm.record_motion) {
        const auto& motion = result.motion;
        const auto number = [&output](double value) {
          if (std::isfinite(value))
            output << value;
          else
            output << "null";
        };
        output << ",\"motion\":";
        if (!motion.recorded) {
          output << "null";
        } else {
          output << "{\"has_previous\":" << (motion.has_previous ? "true" : "false")
                 << ",\"dt_s\":";
          number(motion.dt_s);
          output << ",\"errors\":[";
          for (std::size_t i = 0; i < motion.errors.size(); ++i) {
            if (i) output << ',';
            number(motion.errors[i]);
          }
          output << "],\"median\":";
          number(motion.median);
          output << ",\"tracked\":" << motion.tracked << ",\"global\":" << motion.global
                 << ",\"candidate\":" << motion.candidate
                 << ",\"adopted\":" << (motion.adopted ? "true" : "false")
                 << ",\"escaped\":" << (motion.escaped ? "true" : "false")
                 << ",\"speed_known\":" << (motion.speed_known ? "true" : "false")
                 << ",\"speed_mps\":";
          number(motion.speed_mps);
          output << ",\"unconfirmed_s\":";
          number(motion.unconfirmed_s);
          output << '}';
        }
      }
      output << ",\"route\":[" << result.route.c1 << ',' << result.route.c2 << ','
             << (result.route.valid ? "true" : "false") << ',' << result.route.max_x << ']';
      output << ",\"processing_status\":" << static_cast<unsigned>(result.status)
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

    std::cout << "Geometric detector exported " << sequence << " frames\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
