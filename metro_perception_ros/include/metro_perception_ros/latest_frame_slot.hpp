#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <utility>

namespace metro_perception_ros {

enum class SubmitResult { STORED, OVERWROTE, STOPPED };

template <typename T>
class LatestFrameSlot {
 public:
  SubmitResult submit(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) return SubmitResult::STOPPED;
    const bool overwritten = pending_.has_value();
    pending_ = std::move(value);
    condition_.notify_one();
    return overwritten ? SubmitResult::OVERWROTE : SubmitResult::STORED;
  }

  std::optional<T> wait_take() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return stopped_ || pending_.has_value(); });
    if (stopped_) return std::nullopt;
    auto value = std::move(pending_);
    pending_.reset();
    return value;
  }

  bool clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool had_pending = pending_.has_value();
    pending_.reset();
    return had_pending;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
      pending_.reset();
    }
    condition_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<T> pending_;
  bool stopped_{false};
};

}  // namespace metro_perception_ros
