// Stop search on a simulated joint, spark_home's measurement on a simulated arm and its status
// against simulated SPARKs. No CAN.
#include "simulated_spark_bus.hpp"
#define MOVEMASTER_HOME_NO_MAIN
#include "../examples/spark_home.cpp"
#include <cmath>
#include <functional>

using namespace movemaster;
using namespace movemaster::testing;
using Phase = StopSearch::Phase;
namespace {
constexpr double kDt = 0.002;

HomingConfig homing() {
  HomingConfig h;
  h.current_limit_a = 6;
  h.stall_current_a = 4;
  h.stall_time_s = 0.02;
  h.fast_speed_rad_s = 2.0;
  h.slow_speed_rad_s = 0.5;
  h.backoff_rad = 0.05;
  h.max_travel_rad = 2.0;
  return h;
}
bool contains(const std::string &text, const std::string &part) { return text.find(part) != std::string::npos; }

// One joint between two mechanical stops. It reaches its target unless a stop (or a snag) is in
// the way; pushing on it draws more current the harder it pushes, up to the limit, as a PID would.
// Moving freely draws free_current.
struct Joint {
  double position = 0, lower = -0.4, upper = 0.6, snag = NAN, free_current = 1, limit = 6, current = 0;
  bool frozen = false;
  void follow(double target) {
    if (frozen) return;
    const double top = std::isnan(snag) ? upper : std::min(upper, snag);
    const double reach = std::clamp(target, lower, top);
    current = std::min(limit, free_current + 200 * std::abs(target - reach));
    position = reach;
  }
};
StopSearch run_search(Joint &joint, int direction, const std::function<void(const StopSearch &)> &each = {}) {
  StopSearch search(homing(), direction, 0.05);
  for (int i = 0; i < 100000 && search.phase() != Phase::kFound && search.phase() != Phase::kFailed; ++i) {
    joint.follow(search.update(joint.position, joint.current, kDt));
    if (each) each(search);
  }
  return search;
}

// Both stops are found exactly, with the currents an operator needs to tune the thresholds.
void finds_both_stops() {
  Joint joint;
  const auto up = run_search(joint, 1);
  check(up.phase() == Phase::kFound && std::abs(up.stop() - 0.6) < 1e-9, "Counterclockwise stop not found");
  // The moving peak includes the first touch, still well below the stall threshold.
  check(up.free_current_peak() >= 1 && up.free_current_peak() < 4 && up.stop_current() == 6, "Currents not reported");
  joint.position = 0;
  const auto down = run_search(joint, -1);
  check(down.phase() == Phase::kFound && std::abs(down.stop() + 0.4) < 1e-9, "Clockwise stop not found");
}
// Gravity can draw more than stall_current_a while moving: a joint that follows is no stop.
void moving_current_is_not_a_stop() {
  Joint joint;
  joint.free_current = 5;
  const auto search = run_search(joint, 1);
  check(search.phase() == Phase::kFound && std::abs(search.stop() - 0.6) < 1e-9, "High moving current read as a stop");
  check(search.free_current_peak() >= 5 && search.free_current_peak() < 6, "Moving current not reported");
}
// A snag that holds the fast approach and then lets go: the slow approach must not invent a stop.
void false_stop_fails() {
  Joint joint;
  joint.snag = 0.3;
  const auto search = run_search(joint, 1, [&](const StopSearch &s) {
    if (s.phase() == Phase::kBackoff) joint.snag = NAN;
  });
  check(search.phase() == Phase::kFailed && contains(search.error(), "false stop"), "Snag accepted as a stop");
}
void no_stop_within_travel_fails() {
  Joint joint;
  joint.upper = 5;
  const auto search = run_search(joint, 1);
  check(search.phase() == Phase::kFailed && contains(search.error(), "max_travel_rad"), "Unbounded search");
  check(joint.position < 2.0 + 0.1, "Searched past max_travel_rad");
}
void joint_that_does_not_move_fails() {
  Joint joint;
  joint.frozen = true;
  joint.current = 0;
  const auto search = run_search(joint, -1);
  check(search.phase() == Phase::kFailed && contains(search.error(), "stopped advancing"), "Frozen joint not reported");
}
void ramp() {
  check(ramp_target(0, 1, 2, 0.1, 0, 1) == 0.2, "Ramp step");
  check(ramp_target(0.95, 1, 2, 0.1, 0.95, 1) == 1, "Ramp does not land on the goal");
  check(ramp_target(0, 1, 100, 0.1, 0, 0.05) == 0.05, "Ramp ignores the lead");
}

// Driver-like arm for spark_home::measure: joint 0 is the measured one, joint 1 must hold.
struct FakeArm {
  Joint joint;
  std::vector<JointState> state{{0, 0, 0, false}, {1.0, 0, 0, false}};
  std::vector<JointControl> control{{ControlMode::kMAXMotionPosition, 0}, {ControlMode::kMAXMotionPosition, 0}};
  std::vector<int> limits;  // every set_current_limit
  bool enabled = false, fail_write = false;
  int writes = 0;
  void read() {
    state[0].position = joint.position;
    state[0].current = joint.current;
  }
  void activate() { enabled = true; }
  void deactivate() noexcept { enabled = false; }
  const std::vector<JointState> &states() const { return state; }
  void set_control(std::size_t index, ControlMode mode, int slot) { control.at(index) = {mode, slot}; }
  void set_current_limit(std::size_t index, int amps) {
    check(index == 0 && !enabled, "Current limit changed on the wrong joint or while active");
    limits.push_back(amps);
    joint.limit = amps;
  }
  void write(const std::vector<double> &targets) {
    check(enabled && control[0].mode == ControlMode::kPosition, "Measured joint not in Position mode");
    check(std::abs(targets.at(0) - state[0].position) <= 0.1, "Target beyond max_following_error_rad");
    check(targets.at(1) == 1.0, "The other joint did not hold");
    if (fail_write && ++writes > 10) throw std::runtime_error("Injected watchdog fault");
    joint.follow(targets[0]);
  }
};
DriverConfig arm_config(const char *spec) {
  auto config = config_for(spec, 2);
  config.period_s = kDt;
  config.joints[0].max_following_error_rad = 0.1;
  config.joints[0].homing = homing();
  return config;
}
// Both stops, back to the start after each, and the suggested limits; current limit restored.
void measures_an_arm(const char *spec) {
  const auto config = arm_config(spec);
  FakeArm arm;
  std::ostringstream out;
  check(spark_home::measure(arm, config, 0, {1, -1}, out) == 0, "Measurement failed: " + out.str());
  check(contains(out.str(), "tope +: 0.600 rad") && contains(out.str(), "tope -: -0.400 rad"), "Stops: " + out.str());
  check(contains(out.str(), "\"min_position_rad\": -0.350, \"max_position_rad\": 0.550"), "Limits: " + out.str());
  check(arm.limits == std::vector<int>{6, 40} && !arm.enabled, "Current limit not restored or still enabled");
  check(std::abs(arm.joint.position) <= 0.0125, "Did not return to the start");

  FakeArm one;
  std::ostringstream single;
  check(spark_home::measure(one, config, 0, {-1}, single) == 0, "One-sided measurement failed");
  check(contains(single.str(), "tope -: -0.400 rad") && !contains(single.str(), "min_position_rad"), single.str());
}
void measurement_failures(const char *spec) {
  const auto config = arm_config(spec);
  FakeArm far;
  far.joint.upper = 5;
  std::ostringstream out;
  check(spark_home::measure(far, config, 0, {1}, out) == 1 && contains(out.str(), "max_travel_rad") &&
      contains(out.str(), "al final 1.0 A"), out.str());
  check(far.limits == std::vector<int>{6, 40} && !far.enabled, "Not restored after a failed search");

  FakeArm stopped;
  std::ostringstream interrupted;
  spark_home::stop_requested = 1;
  const int code = spark_home::measure(stopped, config, 0, {1}, interrupted);
  spark_home::stop_requested = 0;
  check(code == 130 && stopped.limits == std::vector<int>{6, 40} && !stopped.enabled, "Interruption not handled");

  FakeArm faulty;
  faulty.fail_write = true;
  std::ostringstream fault;
  rejects([&] { spark_home::measure(faulty, config, 0, {1}, fault); }, "Driver fault swallowed");
  check(faulty.limits == std::vector<int>{6, 40} && !faulty.enabled, "Not restored after a driver fault");
}
void arguments(const char *spec) {
  auto config = config_for(spec, 2);
  config.joints[0].homing = homing();
  using Args = std::vector<std::string>;
  check(!spark_home::parse(config, {}).joint, "Status mode not parsed");
  const auto both = spark_home::parse(config, Args{"--measure", "joint_1"});
  check(both.joint == 0u && both.directions == std::vector<int>{1, -1}, "Both stops not requested");
  check(spark_home::parse(config, Args{"--measure", "joint_1", "-"}).directions == std::vector<int>{-1}, "One side");
  rejects_with([&] { spark_home::parse(config, Args{"--measure", "joint_9"}); }, "Joint desconocido");
  rejects_with([&] { spark_home::parse(config, Args{"--measure", "joint_2"}); }, "no tiene bloque homing");
  rejects_with([&] { spark_home::parse(config, Args{"--measure", "joint_1", "x"}); }, "El sentido es");
  rejects_with([&] { spark_home::parse(config, Args{"--home"}); }, "Se espera --measure");
}
// The status names the SPARKs that restarted, against the same driver the node uses.
void status(const char *spec) {
  const auto config = config_for(spec, 2);
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  bus->has_reset[2] = false;  // only joint_1's SPARK powered up since its last reference
  MoveMasterDriver driver(config, std::move(bus));
  const auto restarted = driver.configure();
  check(restarted == std::vector<std::string>{"joint_1"}, "Restarted SPARKs not reported");
  std::ostringstream out;
  check(spark_home::status(driver, config, restarted, out) == 0, "Status failed");
  check(contains(out.str(), "joint_1  CAN 1  reiniciado") && contains(out.str(), "joint_2  CAN 2  sin reinicio"),
      "Status: " + out.str());
}
}  // namespace
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  try {
    finds_both_stops();
    moving_current_is_not_a_stop();
    false_stop_fails();
    no_stop_within_travel_fails();
    joint_that_does_not_move_fails();
    ramp();
    measures_an_arm(argv[1]);
    measurement_failures(argv[1]);
    arguments(argv[1]);
    status(argv[1]);
    std::cout << "PASS: stop search (both stops, moving current, false stop, travel and time limits), "
        "spark_home measurement and restore, arguments, status.\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
