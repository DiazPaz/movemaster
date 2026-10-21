#pragma once
#include <limits>
#include <string>

namespace movemaster {
// How a joint looks for its mechanical stops (spark_home). Joint units: rad and rad/s, in the
// joint's DH sense (+ = counterclockwise about its z axis); current in A.
struct HomingConfig {
  int current_limit_a = 0;  // SPARK Smart Current Limit while searching, written to RAM only
  // A stop is the joint no longer following its target while the motor draws at least
  // stall_current_a, for stall_time_s.
  double stall_current_a = std::numeric_limits<double>::quiet_NaN();
  double stall_time_s = std::numeric_limits<double>::quiet_NaN();
  double fast_speed_rad_s = std::numeric_limits<double>::quiet_NaN();
  double slow_speed_rad_s = std::numeric_limits<double>::quiet_NaN();
  double backoff_rad = std::numeric_limits<double>::quiet_NaN();     // retreat between both approaches
  double max_travel_rad = std::numeric_limits<double>::quiet_NaN();  // no stop within it: abort
};

// Next target toward goal at speed, never farther than lead from the measured position.
double ramp_target(double target, double goal, double speed, double dt, double position, double lead);

// Finds one mechanical stop as CNC homing does: a fast approach, a backoff and a slow approach,
// whose stall is the stop. No I/O: the caller feeds the measured position and current every
// cycle and sends the returned target in Position mode. lead (how far the target may run ahead
// of the joint) must stay below the driver's max_following_error_rad.
class StopSearch {
 public:
  enum class Phase { kFastApproach, kBackoff, kSlowApproach, kFound, kFailed };
  StopSearch(const HomingConfig &config, int direction, double lead);
  double update(double position, double current, double dt);
  Phase phase() const { return phase_; }
  double stop() const { return stop_; }                     // once kFound
  const std::string &error() const { return error_; }       // once kFailed
  double free_current_peak() const { return free_peak_; }  // fast approach, at least half speed
  double stop_current() const { return stop_current_; }    // when the stop was confirmed
 private:
  HomingConfig config_;
  int direction_;
  double lead_;
  Phase phase_ = Phase::kFastApproach;
  bool started_ = false;
  double target_ = 0, last_position_ = 0, phase_start_ = 0, phase_time_ = 0, stalled_ = 0, contact_ = 0;
  double stop_ = std::numeric_limits<double>::quiet_NaN(), free_peak_ = 0, stop_current_ = 0;
  std::string error_;
  void approach(double position, double current, double dt);
  void enter(Phase phase, double position);
  void fail(const std::string &message, double position);
};
}  // namespace movemaster
