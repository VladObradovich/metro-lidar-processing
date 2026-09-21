#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"

namespace metro_perception_ros {

class TfReplayError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Read only TF ahead of the cloud reader: no queue of large PointCloud2 messages.
// The lookahead uses bag record time, while lookup still uses measurement time.
class BagTfReplay {
 public:
  BagTfReplay(const std::string& bag, const std::vector<std::string>& topics,
              tf2_ros::Buffer& buffer, std::int64_t lookahead_ns)
      : buffer_(buffer), lookahead_ns_(lookahead_ns), enabled_(!topics.empty()) {
    if (lookahead_ns < 0 || lookahead_ns > 1000000000LL) {
      throw std::invalid_argument("TF lookahead must be in [0, 1] seconds");
    }
    if (enabled_) {
      reader_.open(bag);
      rosbag2_storage::StorageFilter filter;
      filter.topics = topics;
      reader_.set_filter(filter);
    }
  }

  void advance(std::int64_t cloud_record_ns, bool new_session) {
    // TF between the last old cloud and this cloud may already belong to the new
    // session. Keep that interval, including records prefetched for the last cloud.
    const auto floor = previous_cloud_record_.value_or(cloud_record_ns);
    while (!recent_dynamic_.empty() && recent_dynamic_.front().first <= floor) {
      recent_dynamic_.pop_front();
    }
    const auto cutoff = cloud_record_ns > std::numeric_limits<std::int64_t>::max() - lookahead_ns_
                            ? std::numeric_limits<std::int64_t>::max()
                            : cloud_record_ns + lookahead_ns_;
    while (enabled_) {
      if (!pending_) {
        if (!reader_.has_next()) break;
        pending_ = reader_.read_next();
      }
      if (pending_->time_stamp > cutoff) break;
      tf2_msgs::msg::TFMessage message;
      rclcpp::SerializedMessage serialized(*pending_->serialized_data);
      serializer_.deserialize_message(&serialized, &message);
      const bool is_static = pending_->topic_name == "/tf_static";
      for (const auto& transform : message.transforms) {
        // An old-session buffer may reject a new epoch as TF_OLD_DATA. Retain the
        // record anyway: after clear() it must be validated/inserted again.
        buffer_.setTransform(transform, "bag_tf", is_static);
        if (is_static) {
          static_transforms_[transform.child_frame_id] = transform;
        } else if (pending_->time_stamp > floor) {
          recent_dynamic_.emplace_back(pending_->time_stamp, transform);
        }
        if (recent_dynamic_.size() + static_transforms_.size() > 100000) {
          throw TfReplayError("TF replay history limit exceeded");
        }
      }
      pending_.reset();
    }
    if (new_session) {
      buffer_.clear();
      for (const auto& entry : static_transforms_) {
        buffer_.setTransform(entry.second, "bag_tf_static", true);
      }
      for (const auto& entry : recent_dynamic_) {
        buffer_.setTransform(entry.second, "bag_tf", false);
      }
    }
    previous_cloud_record_ = cloud_record_ns;
  }

 private:
  rosbag2_cpp::Reader reader_;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer_;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> pending_;
  tf2_ros::Buffer& buffer_;
  std::int64_t lookahead_ns_;
  bool enabled_;
  std::optional<std::int64_t> previous_cloud_record_;
  std::deque<std::pair<std::int64_t, geometry_msgs::msg::TransformStamped>> recent_dynamic_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> static_transforms_;
};

}  // namespace metro_perception_ros
