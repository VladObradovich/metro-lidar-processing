#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>

namespace metro_perception_ros {

// Identity and order of one FrameAnalysis. stamp_ns is nullopt when header.stamp is invalid.
struct FrameKey {
  std::string source;
  std::uint64_t session{0};
  std::uint64_t sequence{0};
  std::optional<std::int64_t> stamp_ns;
};

struct GateDecision {
  bool accepted{false};
  // Accepted into a new source or session: per-source monitor state must be reset.
  bool reset{false};
  const char* reason{""};
};

// Admits analyses of one active source in order. A new source replaces the active one and
// retires it until restart; a newer session of the active source resets it. Rejected keys
// never change the gate, so a malformed or late message cannot displace the valid stream.
class FrameKeyGate {
 public:
  GateDecision admit(const FrameKey& key) {
    if (key.source.empty()) return reject("INVALID_SOURCE_ID");
    if (!key.stamp_ns) return reject("INVALID_STAMP");
    if (key.sequence == 0) return reject("INVALID_SEQUENCE");
    if (retired_.count(key.source)) return reject("RETIRED_SOURCE");
    if (!active_) return accept(key, "FIRST_SOURCE");
    if (key.source != active_->source) {
      retired_.insert(active_->source);
      return accept(key, "NEW_SOURCE");
    }
    if (key.session < active_->session) return reject("OLD_SESSION");
    if (key.session > active_->session) return accept(key, "NEW_SESSION");
    if (key.sequence <= active_->sequence) return reject("DUPLICATE_OR_LATE_SEQUENCE");
    if (*key.stamp_ns < *active_->stamp_ns) return reject("STAMP_WENT_BACKWARDS");
    active_ = key;
    return {true, false, "IN_ORDER"};
  }

  const std::optional<FrameKey>& active() const { return active_; }
  std::size_t retired_sources() const { return retired_.size(); }

 private:
  static GateDecision reject(const char* reason) { return {false, false, reason}; }
  GateDecision accept(const FrameKey& key, const char* reason) {
    active_ = key;
    return {true, true, reason};
  }

  std::optional<FrameKey> active_;
  std::unordered_set<std::string> retired_;
};

}  // namespace metro_perception_ros
