#include "metro_perception_core/speed_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace metro_perception_core {
namespace {
// Periodic lining (rings about 1 m apart) produces aliases that can match better
// than the true shift for several frames, and a featureless stretch leaves only
// the sensor-fixed scan pattern, which matches at zero shift. The speed is
// therefore kept within physical reach of its last value and acquired anew only
// on a minimum that stands out.
//
// Metro traction and braking stay below about 1.5 m/s^2; 2 m/s^2 bounds every
// change of the estimated speed with a margin.
constexpr double kMaxAccelerationMps2 = 2.0;
// Standing still the whole scene matches at zero shift, so its error falls far
// below the median over all shifts (0.2-0.5 of it in stationary recordings, also
// at platforms with people); moving through a featureless tunnel only the scan
// pattern matches there (0.7-1.0). A shift of at most one step needs the sharp
// match.
constexpr double kStandstillMaxErrorShare = 0.6;
// A speed is acquired only on a minimum whose contrast to the median is this
// multiple of the best competing minimum more than three steps away. A true
// shift usually stands out by about 2; aliases and featureless stretches by
// about 1.2.
constexpr double kMinUniqueness = 1.5;
// Below this speed the 5 cm quantisation of the shift is a quarter of the speed or
// more: the estimate follows the shifts freely and may still adopt a clearly
// better distant minimum, and neither the standstill nor the rate rule applies,
// so that a train starting to creep is not held at zero.
constexpr double kLowSpeedMps = 2.0;
// A distant minimum that stands out as the global one for this many frames at a
// consistent speed replaces the speed even beyond physical reach: then the track
// itself is wrong, e.g. acquired on a transient minimum. Lining aliases stood out
// for up to 1.6 s in recordings, true shifts for 4 s and more.
constexpr std::size_t kEscapeFrames = 25;
// An unconfirmed speed drives the odometry for kMotionHoldS and centres the
// reacquisition window until kMotionForgetS.
constexpr double kMotionHoldS = 5.0, kMotionForgetS = 10.0;
constexpr std::size_t kMotionConfirmFrames = 3;
constexpr double kSpeedGain = 0.5;
}  // namespace

void SpeedTracker::reset() { *this = SpeedTracker(); }

SpeedTracker::Step SpeedTracker::update(const std::vector<double>& errors, double dt,
                                        double min_contrast, MotionDiagnostics* diagnostics) {
  Step step;
  const std::size_t max_shift = errors.empty() ? 0 : errors.size() - 1;
  if (dt > 0 && max_shift >= 4) {
    std::vector<double> finite;
    for (const double error : errors)
      if (std::isfinite(error)) finite.push_back(error);
    double median = INFINITY;
    if (finite.size() >= 4) {
      std::nth_element(finite.begin(), finite.begin() + finite.size() / 2, finite.end());
      median = finite[finite.size() / 2];
    }
    if (diagnostics) {
      diagnostics->errors = errors;
      diagnostics->median = median;
    }
    // At low speed the standstill and rate rules do not apply (see kLowSpeedMps).
    const bool guarded = !speed_known_ || speed_mps_ >= kLowSpeedMps;
    // Best clear shift within [predicted - window, predicted + window] steps.
    auto search = [&](double predicted, double window) -> std::optional<std::size_t> {
      const auto low = static_cast<std::size_t>(std::max(0.0, std::floor(predicted - window)));
      const auto high =
          std::min(max_shift, static_cast<std::size_t>(std::ceil(predicted + window)));
      if (low > high) return std::nullopt;
      std::size_t best = low;
      for (std::size_t shift = low; shift <= high; ++shift)
        if (errors[shift] < errors[best]) best = shift;
      // Without enough comparable shifts there is no median to stand out from.
      if (!std::isfinite(median) || !std::isfinite(errors[best]) ||
          median - errors[best] < min_contrast)
        return std::nullopt;
      if (guarded && best <= 1 && !(errors[0] <= kStandstillMaxErrorShare * median))
        return std::nullopt;
      return best;
    };
    auto stands_out = [&](std::size_t shift) {
      // Competitors are other minima of the curve taken as the least error over
      // three neighbouring shifts: neither the flank of a wide valley nor the
      // zigzag of returns sampled more coarsely than a profile step is one. A flat
      // run of equal values is one minimum when both neighbouring values are higher.
      const std::size_t n = errors.size();
      std::vector<double> low(errors);
      for (std::size_t i = 0; i < n; ++i) {
        if (i > 0) low[i] = std::min(low[i], errors[i - 1]);
        if (i + 1 < n) low[i] = std::min(low[i], errors[i + 1]);
      }
      double competitor = 0;
      for (std::size_t i = 0; i < n;) {
        std::size_t j = i;
        while (j + 1 < n && low[j + 1] == low[i]) ++j;
        const double value = low[i];
        if (std::isfinite(value) && (i == 0 || low[i - 1] > value) &&
            (j + 1 == n || low[j + 1] > value) && (i > shift + 3 || j + 3 < shift))
          competitor = std::max(competitor, median - value);
        i = j + 1;
      }
      return median - errors[shift] >= kMinUniqueness * competitor;
    };
    const double predicted = speed_mps_ * dt / kStepM;
    const auto global =
        static_cast<std::size_t>(std::min_element(errors.begin(), errors.end()) - errors.begin());
    // A distant global minimum that stands out (sharp if it is standstill) counts
    // towards replacing a known speed that it keeps contradicting.
    std::optional<double> escape;
    if (speed_known_) {
      const bool distant = std::abs(double(global) - predicted) > 2;
      const bool sharp = global > 1 || errors[0] <= kStandstillMaxErrorShare * median;
      if (distant && sharp && std::isfinite(median) && median - errors[global] >= min_contrast &&
          stands_out(global)) {
        const double speed = global * kStepM / dt;
        escape_frames_ = std::abs(speed - escape_speed_mps_) <= 1.0 ? escape_frames_ + 1 : 1;
        escape_speed_mps_ = speed;
        if (escape_frames_ >= kEscapeFrames) escape = speed;
      } else {
        escape_frames_ = 0;
      }
    }
    // A known speed is tracked within two 5 cm steps per frame. While it is
    // tracked at speed a distant better minimum is out of physical reach.
    std::optional<std::size_t> tracked;
    if (speed_known_) tracked = search(std::round(predicted), 2);
    // Without a tracked shift a speed is acquired on the best overall minimum,
    // within reach of the last speed when there is one, standing out and
    // consistent for kMotionConfirmFrames frames.
    std::optional<std::size_t> candidate;
    bool adopted = false;
    if (!tracked) {
      candidate = speed_known_
                      ? search(predicted, 1 + std::ceil(kMaxAccelerationMps2 *
                                                        (unconfirmed_s_ + dt) * dt / kStepM))
                      : search(0, double(max_shift));
      if (candidate &&
          (*candidate > global + 1 || *candidate + 1 < global || !stands_out(*candidate)))
        candidate.reset();
    } else if (!guarded && (global > *tracked + 1 || global + 1 < *tracked) &&
               errors[*tracked] - errors[global] > min_contrast) {
      // At low speed a clearly better distant minimum that stands out is adopted.
      candidate = search(double(global), 0);
      if (candidate && !stands_out(*candidate)) candidate.reset();
    }
    if (diagnostics) {
      diagnostics->tracked = tracked ? static_cast<int>(*tracked) : -1;
      diagnostics->global = static_cast<int>(global);
      diagnostics->candidate = candidate ? static_cast<int>(*candidate) : -1;
    }
    if (candidate) {
      const double speed = *candidate * kStepM / dt;
      pending_frames_ = std::abs(speed - pending_speed_mps_) <= 1.0 ? pending_frames_ + 1 : 1;
      pending_speed_mps_ = speed;
      if (pending_frames_ >= kMotionConfirmFrames) {
        tracked = candidate;
        adopted = true;
        pending_frames_ = 0;
      }
      if (diagnostics) diagnostics->adopted = adopted;
    } else {
      pending_frames_ = 0;
    }
    if (tracked) {
      // Shifts are quantised to 5 cm; a tracked speed follows them gradually and,
      // at speed, no faster than the train can change speed.
      const double measured = *tracked * kStepM / dt;
      if (!adopted && unconfirmed_s_ == 0 && speed_known_) {
        double change = kSpeedGain * (measured - speed_mps_);
        if (guarded)
          change = std::clamp(change, -kMaxAccelerationMps2 * dt, kMaxAccelerationMps2 * dt);
        speed_mps_ += change;
      } else {
        speed_mps_ = measured;
      }
      step.displacement_m = speed_mps_ * dt;
      unconfirmed_s_ = 0;
      speed_known_ = step.valid = true;
    } else if (speed_known_) {
      unconfirmed_s_ += dt;
      if (unconfirmed_s_ <= kMotionHoldS) {
        step.displacement_m = speed_mps_ * dt;
        step.valid = true;
      } else if (unconfirmed_s_ > kMotionForgetS) {
        speed_known_ = false;
      }
    }
    if (escape) {
      speed_mps_ = *escape;
      step.displacement_m = speed_mps_ * dt;
      unconfirmed_s_ = 0;
      speed_known_ = step.valid = true;
      pending_frames_ = 0;
      escape_frames_ = 0;
      if (diagnostics) diagnostics->escaped = true;
    }
  }
  if (diagnostics) {
    diagnostics->speed_known = speed_known_;
    diagnostics->speed_mps = speed_mps_;
    diagnostics->unconfirmed_s = unconfirmed_s_;
  }
  return step;
}
}  // namespace metro_perception_core
