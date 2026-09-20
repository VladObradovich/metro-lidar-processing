#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "metro_perception_core/temporal_monitor.hpp"
#include "metro_perception_ros/pointcloud_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
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
}  // namespace
int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: evaluate_bag BAG TOPIC OUTPUT.jsonl\n"
              << "Scaffold: all results are UNKNOWN; geometry and TF are not implemented.\n";
    return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
  }
  try {
    if (std::filesystem::exists(argv[3])) {
      throw std::runtime_error("Output already exists");
    }
    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    bool found = false;
    for (const auto& topic : reader.get_all_topics_and_types()) {
      if (topic.name == argv[2] && topic.type == "sensor_msgs/msg/PointCloud2") {
        found = true;
      }
    }
    if (!found) {
      throw std::runtime_error("Requested PointCloud2 topic not found");
    }
    rosbag2_storage::StorageFilter filter;
    filter.topics = {argv[2]};
    reader.set_filter(filter);
    std::ofstream output(argv[3]);
    if (!output) {
      throw std::runtime_error("Cannot create output");
    }
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    metro_perception_core::PerceptionPipeline pipeline;
    metro_perception_core::TemporalMonitor monitor;
    std::uint64_t sequence = 0, session = 0;
    std::int64_t previous_stamp = 0;
    while (reader.has_next()) {
      const auto bag_message = reader.read_next();
      const auto start = std::chrono::steady_clock::now();
      std::int64_t stamp = 0;
      metro_perception_core::FrameResult result;
      try {
        sensor_msgs::msg::PointCloud2 cloud;
        rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
        serializer.deserialize_message(&serialized, &cloud);
        stamp = std::int64_t(cloud.header.stamp.sec) * 1000000000LL + cloud.header.stamp.nanosec;
        if (sequence > 0 && stamp < previous_stamp) {
          ++session;
          pipeline.reset();
          monitor.reset();
        }
        previous_stamp = stamp;
        result = metro_perception_ros::process_cloud(cloud, pipeline);
      } catch (const std::exception&) {
        result.status = metro_perception_core::AnalysisStatus::BAD_INPUT;
        result.reason = "DESERIALIZATION_ERROR";
      }
      const auto assessment = monitor.update(result, stamp);
      const double elapsed =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
              .count();
      output << "{\"schema_version\":1,\"mode\":\"scaffold\",\"bag_id\":" << json_string(argv[1])
             << ",\"session_id\":" << session << ",\"frame_sequence\":" << ++sequence
             << ",\"measurement_stamp_ns\":" << stamp
             << ",\"bag_stamp_ns\":" << bag_message->time_stamp
             << ",\"state\":\"UNKNOWN\",\"reason\":" << json_string(assessment.reason)
             << ",\"distance_m\":null,\"candidate_count\":0,\"evaluation_region_valid\":false"
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
    std::cout << "Scaffold exported " << sequence << " UNKNOWN frames\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
