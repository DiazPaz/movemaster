#include "movemaster_hardware/homing.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace movemaster {
namespace {
// Twice the time the move needs at speed, plus a second to start and settle.
double time_limit(double distance, double speed) { return 2 * distance / speed + 1.0; }
}
double ramp_target(double target, double goal, double speed, double dt, double position, double lead) {
  const double step = speed * dt;
  target = std::abs(goal - target) <= step ? goal : target + std::copysign(step, goal - target);
  return std::clamp(target, position - lead, position + lead);
}
StopSearch::StopSearch(const HomingConfig &config, int direction, double lead)
    : config_(config), direction_(direction), lead_(lead) {
  if (direction != 1 && direction != -1) throw std::invalid_argument("Stop search direction must be +1 or -1");
  if (!(lead > 0)) throw std::invalid_argument("Stop search lead must be positive");
}
double StopSearch::update(double position, double current, double dt) {
  if (!started_) {
    started_ = true;
    enter(phase_, position);
  }
  phase_time_ += dt;
  if (phase_ == Phase::kFastApproach || phase_ == Phase::kSlowApproach) {
    approach(position, current, dt);
  } else if (phase_ == Phase::kBackoff) {
    const double goal = contact_ - direction_ * config_.backoff_rad;
    target_ = ramp_target(target_, goal, config_.fast_speed_rad_s, dt, position, lead_);
    if (target_ == goal && std::abs(position - goal) <= 0.25 * lead_) enter(Phase::kSlowApproach, position);
    else if (phase_time_ > time_limit(config_.backoff_rad, config_.fast_speed_rad_s))
      fail("could not back off from the stop", position);
  }
  last_position_ = position;
  return target_;
}
void StopSearch::approach(double position, double current, double dt) {
  const bool fast = phase_ == Phase::kFastApproach;
  const double speed = fast ? config_.fast_speed_rad_s : config_.slow_speed_rad_s;
  // The slow approach must meet the stop the fast one found, just past the backoff.
  const double travel_limit = fast ? config_.max_travel_rad : 3 * config_.backoff_rad;
  target_ = std::clamp(target_ + direction_ * speed * dt, position - lead_, position + lead_);
  // Stalled: the joint no longer keeps up with its target while the motor pushes.
  const bool stalled = direction_ * (target_ - position) >= 0.8 * lead_ && current >= config_.stall_current_a;
  stalled_ = stalled ? stalled_ + dt : 0;
  if (stalled_ >= config_.stall_time_s) {
    if (fast) {
      contact_ = position;
      enter(Phase::kBackoff, position);
    } else {
      stop_ = position;
      stop_current_ = current;
      phase_ = Phase::kFound;
      target_ = position;  // Stop pushing.
    }
    return;
  }
  // Free motion: the joint advances at half the speed or more (gravity and friction, no stop).
  if (fast && direction_ * (position - last_position_) >= 0.5 * speed * dt) free_peak_ = std::max(free_peak_, current);
  if (direction_ * (position - phase_start_) > travel_limit)
    fail(fast ? "no stop within max_travel_rad"
              : "the slow approach went past the stop the fast one found: it was a false stop "
                "(raise stall_current_a or current_limit_a, or lower fast_speed_rad_s)", position);
  else if (phase_time_ > time_limit(travel_limit, speed))
    fail("the joint stopped advancing without reaching stall_current_a: if it is against a stop, lower "
         "stall_current_a", position);
}
void StopSearch::enter(Phase phase, double position) {
  phase_ = phase;
  last_position_ = position;
  phase_start_ = position;
  phase_time_ = 0;
  stalled_ = 0;
  target_ = position;
}
void StopSearch::fail(const std::string &message, double position) {
  phase_ = Phase::kFailed;
  error_ = message;
  target_ = position;
}
}  // namespace movemaster
