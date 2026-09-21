#pragma once

#include <cstdint>
#include <mutex>

namespace metro_perception_ros {

// A reset and publication are linearized by the same mutex. A result either publishes
// before advance() completes, or observes the new session and is discarded.
class SessionGate {
 public:
  std::uint64_t current() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_;
  }

  void advance() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++session_;
  }

  template <typename Publish>
  bool publish_if_current(std::uint64_t expected, Publish publish) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (expected != session_) return false;
    publish();
    return true;
  }

 private:
  mutable std::mutex mutex_;
  std::uint64_t session_{0};
};

}  // namespace metro_perception_ros
