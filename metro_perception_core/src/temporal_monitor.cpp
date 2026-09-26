#include "metro_perception_core/temporal_monitor.hpp"

#include <algorithm>
#include <bitset>
#include <tuple>

namespace metro_perception_core {
namespace {
std::uint32_t window_mask(std::size_t window) {
  return window >= 32 ? 0xffffffffu : (1u << window) - 1u;
}
std::uint32_t hits_in(std::uint32_t history, std::size_t window) {
  return static_cast<std::uint32_t>(std::bitset<32>(history & window_mask(window)).count());
}
std::string obstacle_reason(CalibrationTrust trust, bool coasting) {
  const std::string base = coasting ? "OBSTACLE_COASTING" : "OBSTACLE";
  switch (trust) {
    case CalibrationTrust::ASSUMED:
      return base + "_WITH_ASSUMED_CALIBRATION";
    case CalibrationTrust::VERIFIED:
      return coasting ? base : "OBSTACLE_CANDIDATE";
    default:
      return base + "_WITH_UNKNOWN_CALIBRATION";
  }
}
}  // namespace

TemporalMonitor::TemporalMonitor(TemporalConfig config) : config_(config) { config_.validate(); }

void TemporalMonitor::reset() {
  tracks_.clear();
  last_time_ns_ = 0;
}

bool TemporalMonitor::update_tracks(const FrameResult& frame, std::int64_t time_ns) {
  auto any_tentative = [&] {
    return std::any_of(tracks_.begin(), tracks_.end(), [](const Track& t) { return !t.confirmed; });
  };
  // The same measurement again (a re-delivered or re-analysed cloud) is no new evidence.
  if (time_ns && time_ns == last_time_ns_) return any_tentative();
  // Time: a step back or a long pause restarts every track (seek, loop, stop).
  double dt = 0;
  if (last_time_ns_ && time_ns) {
    if (time_ns < last_time_ns_ || (time_ns - last_time_ns_) * 1e-9 > config_.max_gap_s) {
      tracks_.clear();
    } else {
      dt = (time_ns - last_time_ns_) * 1e-9;
    }
  }
  if (time_ns) last_time_ns_ = time_ns;

  // Prediction: fixed in the world, so the train's own travel brings it closer.
  const bool ego = frame.ego_motion_valid && std::isfinite(frame.ego_speed_mps);
  const bool still = ego && std::abs(frame.ego_speed_mps) < config_.still_speed_mps;
  const double travel = ego ? std::max(0.0, frame.ego_speed_mps) * dt : 0.0;
  for (auto& track : tracks_) {
    track.center.x -= travel;
    track.distance_m -= travel;
  }

  std::vector<std::size_t> usable;
  for (std::size_t i = 0; i < frame.candidates.size(); ++i)
    if (usable_distance(frame.candidates[i].distance_valid, frame.candidates[i].distance_m))
      usable.push_back(i);

  // Greedy one-to-one association by centre distance inside a range-dependent gate.
  std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
  const double motion =
      config_.object_max_speed_mps * dt + (ego ? 0.0 : config_.unknown_ego_speed_mps * dt);
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    const auto& track = tracks_[t];
    const double gate =
        config_.gate_base_m + config_.gate_range_fraction * std::max(0.0, track.center.x) + motion;
    for (const auto c : usable) {
      const auto& centre = frame.candidates[c].center;
      const double d = std::hypot(centre.x - track.center.x, centre.y - track.center.y);
      if (d <= gate) pairs.emplace_back(d, t, c);
    }
  }
  std::sort(pairs.begin(), pairs.end());
  std::vector<bool> track_used(tracks_.size(), false), candidate_used(frame.candidates.size());
  // An edge measurement keeps the track alive but is no evidence for confirmation.
  auto measure = [](Track& track, const ObstacleCandidate& candidate) {
    track.center = candidate.center;
    track.size = candidate.size;
    track.distance_m = candidate.distance_m;
    track.channels |= candidate.channels;
    track.support = candidate.support_points;
    if (!candidate.edge) track.history |= 1u;
    track.misses = 0;
  };
  for (auto& track : tracks_) {
    track.history <<= 1;
    ++track.age_frames;
  }
  for (const auto& [d, t, c] : pairs) {
    if (track_used[t] || candidate_used[c]) continue;
    track_used[t] = candidate_used[c] = true;
    measure(tracks_[t], frame.candidates[c]);
  }
  for (std::size_t t = 0; t < tracks_.size(); ++t)
    if (!track_used[t]) ++tracks_[t].misses;
  for (const auto c : usable) {
    if (candidate_used[c]) continue;
    Track track;
    track.id = next_id_++;
    track.age_frames = 1;
    measure(track, frame.candidates[c]);
    tracks_.push_back(track);
  }

  // Confirmation, then removal of released or stale tracks.
  const std::size_t longest =
      std::max({config_.confirm_window, config_.gauge_confirm_window, config_.far_confirm_window});
  const double evaluated = frame.evaluation_region_valid && std::isfinite(frame.evaluated_range_m)
                               ? frame.evaluated_range_m
                               : 0.0;
  for (auto& track : tracks_) {
    const bool gauge_only = track.channels == ObstacleCandidate::kGauge;
    const auto window = gauge_only ? config_.gauge_confirm_window : config_.confirm_window;
    const auto needed = gauge_only ? config_.gauge_confirm_hits : config_.confirm_hits;
    const bool far =
        (evaluated > 0 && track.distance_m > evaluated) ||
        (config_.far_confirm_from_m > 0 && track.distance_m > config_.far_confirm_from_m);
    if (!track.confirmed && hits_in(track.history, window) >= needed &&
        (!far || hits_in(track.history, config_.far_confirm_window) >= config_.far_confirm_hits))
      track.confirmed = true;
    // Standing still, the background difference has no odometry or disocclusion error:
    // new MOTION evidence is a real change and is confirmed when it is first seen.
    if (!track.confirmed && still && (track.history & 1u) &&
        (track.channels & ObstacleCandidate::kMotion) && track.support >= config_.still_min_points)
      track.confirmed = true;
  }
  auto finished = [&](const Track& track) {
    return !(track.distance_m > 0) ||
           track.misses >= (track.confirmed ? config_.release_misses : longest);
  };
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), finished), tracks_.end());
  if (tracks_.size() > config_.max_tracks) {
    // Keep confirmed tracks; drop the weakest, farthest tentative ones first.
    std::stable_sort(tracks_.begin(), tracks_.end(), [&](const Track& a, const Track& b) {
      if (a.confirmed != b.confirmed) return a.confirmed;
      const auto ha = hits_in(a.history, longest), hb = hits_in(b.history, longest);
      return ha != hb ? ha > hb : a.distance_m < b.distance_m;
    });
    while (tracks_.size() > config_.max_tracks && !tracks_.back().confirmed) tracks_.pop_back();
  }
  return any_tentative();
}

std::vector<TrackSummary> TemporalMonitor::summaries() const {
  const std::size_t longest =
      std::max({config_.confirm_window, config_.gauge_confirm_window, config_.far_confirm_window});
  std::vector<TrackSummary> out;
  for (const auto& track : tracks_) {
    TrackSummary summary;
    summary.id = track.id;
    summary.confirmed = track.confirmed;
    summary.coasting = track.misses > 0;
    summary.hits = hits_in(track.history, longest);
    summary.age_frames = track.age_frames;
    summary.distance_m = track.distance_m;
    summary.center = track.center;
    summary.size = track.size;
    summary.channels = track.channels;
    out.push_back(summary);
  }
  return out;
}

Assessment TemporalMonitor::update(const FrameResult& frame, std::int64_t measurement_time_ns) {
  Assessment result;
  result.calibration_trust = frame.calibration_trust;
  if (frame.status != AnalysisStatus::OK) {
    // A failed frame carries no evidence either way: tracks are neither updated nor aged.
    result.reason = frame.reason.empty() ? "ANALYSIS_FAILED" : frame.reason;
    result.tracks = summaries();
    return result;
  }
  const bool tentative = update_tracks(frame, measurement_time_ns);
  result.tracks = summaries();
  const TrackSummary* nearest = nullptr;
  for (const auto& track : result.tracks) {
    if (!track.confirmed || !usable_distance(true, track.distance_m)) continue;
    if (!nearest || track.distance_m < nearest->distance_m) nearest = &track;
  }
  if (nearest) {
    result.state = State::OBSTACLE;
    result.reason = obstacle_reason(frame.calibration_trust, nearest->coasting);
    result.distance_m = nearest->distance_m;
    result.distance_valid = true;
    return result;
  }
  bool usable_now = false;
  for (const auto& candidate : frame.candidates)
    usable_now = usable_now || usable_distance(candidate.distance_valid, candidate.distance_m);
  // Something was found but is not confirmed or cannot be ranged: never a clear path.
  if (usable_now || (tentative && frame.candidates.empty())) {
    result.reason = "CANDIDATE_UNCONFIRMED";
    return result;
  }
  if (!frame.candidates.empty()) {
    result.reason = "CANDIDATE_DISTANCE_INVALID";
    return result;
  }
  const bool region_usable = frame.evaluation_region_valid &&
                             std::isfinite(frame.evaluated_range_m) && frame.evaluated_range_m > 0;
  const bool assumed = frame.calibration_trust == CalibrationTrust::ASSUMED;
  if (frame.calibration_trust == CalibrationTrust::UNKNOWN) {
    result.reason = "CALIBRATION_TRUST_UNKNOWN";
  } else if (assumed && !config_.assumed_clear) {
    result.reason = "ASSUMED_CALIBRATION_CANNOT_CONFIRM_CLEAR";
  } else if (region_usable && assumed &&
             frame.evaluated_range_m < config_.assumed_clear_min_range_m) {
    result.reason = "EVALUATED_RANGE_TOO_SHORT";
  } else if (region_usable) {
    result.state = State::NO_OBSTACLE_DETECTED;
    result.reason =
        assumed ? "NO_CANDIDATE_ASSUMED_CALIBRATION" : "NO_CANDIDATE_IN_EVALUATED_REGION";
  } else if (frame.reason == "BACKGROUND_CANNOT_CONFIRM_CLEAR") {
    result.reason = frame.reason;
  } else {
    result.reason =
        frame.evaluation_region_valid ? "EVALUATION_REGION_INVALID" : "CORRIDOR_UNOBSERVABLE";
  }
  return result;
}
Assessment TemporalMonitor::on_timeout() const {
  Assessment result;
  result.reason = "INPUT_PAUSED_OR_STOPPED";
  return result;
}
}  // namespace metro_perception_core
