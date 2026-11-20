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
// A joint held back without a stall fails stall_time_s + kHeldMarginS after it stops following,
// not after the whole max_travel_rad at speed.
StopSearch timed_search(Joint &joint, int direction, double &seconds) {
  int steps = 0;
  const auto search = run_search(joint, direction, [&](const StopSearch &) { ++steps; });
  seconds = steps * kDt;
  return search;
}
void joint_that_does_not_move_fails() {
  Joint joint;
  joint.frozen = true;
  joint.current = 0;
  double seconds = 0;
  const auto search = timed_search(joint, -1, seconds);
  check(search.phase() == Phase::kFailed && contains(search.error(), "stopped advancing"), "Frozen joint not reported");
  // 0.8 * lead at fast speed (0.02 s), then stall_time_s + kHeldMarginS (2.02 s).
  const double expected = 0.8 * 0.05 / homing().fast_speed_rad_s + homing().stall_time_s + kHeldMarginS;
  check(std::abs(seconds - expected) < 0.01, "Frozen joint failed after " + std::to_string(seconds) + " s");
}
// Two holds of 1.1 s each (stiff stretches) add up to more than stall_time_s + kHeldMarginS,
// but each one is shorter: the search goes on to the stop.
void brief_holds_are_not_a_failure() {
  Joint joint;
  int step = 0;
  const auto search = run_search(joint, 1, [&](const StopSearch &) {
    ++step;
    joint.frozen = (step >= 20 && step < 570) || (step >= 600 && step < 1150);
  });
  check(step > 1150 && search.phase() == Phase::kFound && std::abs(search.stop() - 0.6) < 1e-9,
      "Brief holds failed the search: " + search.error());
}
// A stop the motor pushes with less than stall_current_a: reported soon, not as a stop.
void stop_below_stall_current_fails() {
  Joint joint;
  joint.limit = 3;
  double seconds = 0;
  const auto search = timed_search(joint, 1, seconds);
  check(search.phase() == Phase::kFailed && contains(search.error(), "lower stall_current_a"),
      "Weak push on a stop not reported");
  // 0.3 s to reach the stop at 0.6 rad, then the same as a frozen joint.
  check(seconds < 0.3 + 0.02 + homing().stall_time_s + kHeldMarginS + 0.01 && joint.position == 0.6,
      "Weak push on a stop failed after " + std::to_string(seconds) + " s");
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
  bool referenced(std::size_t) const { return true; }
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
// Three SPARK joints with a stop each, and the gripper wrist (joint_4), which has none.
DriverConfig home_config(const char *spec) {
  auto config = config_with_gripper(spec, 3);  // period_s 0.005, the shortest the driver takes
  const std::vector<std::pair<int, double>> stops{{-1, -1.0}, {-1, -2.0}, {1, 1.5}};
  for (std::size_t i = 0; i < stops.size(); ++i) {
    auto &joint = config.joints[i];
    joint.min_position_rad = -2.5;
    joint.max_position_rad = 2.5;
    joint.max_following_error_rad = 0.1;
    joint.homing = homing();
    joint.homing->stop = stops[i].first;
    joint.homing->stop_rad = stops[i].second;
  }
  config.homing_order = {"joint_2", "joint_3", "joint_1"};
  validate_driver_config(config);
  return config;
}
void arguments(const char *spec) {
  auto config = config_for(spec, 2);
  config.joints[0].homing = homing();
  using Args = std::vector<std::string>;
  check(!spark_home::parse(config, {}).joint && !spark_home::parse(config, {}).home, "Status mode not parsed");
  const auto both = spark_home::parse(config, Args{"--measure", "joint_1"});
  check(both.joint == 0u && both.directions == std::vector<int>{1, -1}, "Both stops not requested");
  check(spark_home::parse(config, Args{"--measure", "joint_1", "-"}).directions == std::vector<int>{-1}, "One side");
  rejects_with([&] { spark_home::parse(config, Args{"--measure", "joint_9"}); }, "Joint desconocido");
  rejects_with([&] { spark_home::parse(config, Args{"--measure", "joint_2"}); }, "no tiene bloque homing");
  rejects_with([&] { spark_home::parse(config, Args{"--measure", "joint_1", "x"}); }, "El sentido es");
  rejects_with([&] { spark_home::parse(config, Args{"--measur", "joint_1"}); }, "Se espera --measure");
  rejects_with([&] { spark_home::parse(config, Args{"--home"}); }, "no tiene homing_order");

  const auto homed = home_config(spec);
  const auto all = spark_home::parse(homed, Args{"--home"});
  check(all.home && all.named.empty() && all.pose.empty() && !all.joint, "--home not parsed");
  const auto some = spark_home::parse(homed, Args{"--home", "joint_3", "joint_1", "--pose", "joint_1=0,joint_4=-0.3"});
  check(some.named == std::vector<std::size_t>{2, 0} &&
      some.pose == std::map<std::size_t, double>{{0, 0.0}, {3, -0.3}}, "--home with joints and --pose not parsed");
  const auto broken = [&](const Args &args, const std::string &expected) {
    rejects_with([&] { spark_home::parse(homed, args); }, expected);
  };
  broken({"--home", "joint_4"}, "joint_4 no tiene homing.stop");
  broken({"--home", "joint_1", "joint_1"}, "joint_1 repetido");
  broken({"--home", "--pose"}, "Se espera una sola --pose");
  broken({"--home", "--pose", "joint_1=0", "--pose", "joint_2=0"}, "Se espera una sola --pose");
  broken({"--home", "--pose", "joint_1"}, "--pose espera joint_N=rad");
  broken({"--home", "--pose", "joint_1=x"}, "joint_1=x no es un angulo");
  broken({"--home", "--pose", "joint_1=0.5rad"}, "no es un angulo");
  broken({"--home", "--pose", "joint_1=nan"}, "no es un angulo");
  broken({"--home", "--pose", "joint_1=3"}, "joint_1 = 3 rad esta fuera de");
  broken({"--home", "--pose", "joint_1=0,joint_1=0.1"}, "joint_1 repetido");
  broken({"--home", "--pose", "joint_7=0"}, "Joint desconocido en JOINTS.json: joint_7");
}

// Driver-like arm for spark_home::home: each joint follows its target between its stops and
// counts from where it powered up until shift_reference, which moves its whole scale.
struct FakeRobot {
  std::vector<Joint> joints;
  std::vector<JointState> state;
  std::vector<JointControl> control;
  std::vector<bool> reference;
  std::vector<std::string> calls;  // set_current_limit and shift_reference, in order
  bool enabled = false;
  int activations = 0;
  explicit FakeRobot(std::vector<Joint> powered)
      : joints(std::move(powered)), state(joints.size()),
        control(joints.size(), {ControlMode::kMAXMotionPosition, 0}), reference(joints.size(), false) {
    read();
  }
  void read() {
    for (std::size_t i = 0; i < joints.size(); ++i) {
      state[i].position = joints[i].position;
      state[i].current = joints[i].current;
    }
  }
  void activate() {
    check(!enabled, "Activated twice");
    enabled = true;
    ++activations;
    read();
  }
  void deactivate() noexcept { enabled = false; }
  const std::vector<JointState> &states() const { return state; }
  bool referenced(std::size_t index) const { return reference.at(index); }
  void set_control(std::size_t index, ControlMode mode, int slot) {
    check(index < 3, "Control mode set on the gripper");
    control.at(index) = {mode, slot};
  }
  void set_current_limit(std::size_t index, int amps) {
    check(index < 3 && !enabled, "Current limit changed on the gripper or while active");
    joints.at(index).limit = amps;
    calls.push_back("limit " + std::to_string(index) + " " + std::to_string(amps));
  }
  void shift_reference(std::size_t index, double offset) {
    check(!enabled, "Reference shifted while active");
    auto &joint = joints.at(index);
    joint.position += offset;
    joint.lower += offset;
    joint.upper += offset;
    reference.at(index) = true;
    read();
    calls.push_back("shift " + std::to_string(index));
  }
  void write(const std::vector<double> &targets) {
    check(enabled, "Write while disabled");
    for (std::size_t i = 0; i < joints.size(); ++i) {
      check(control[i].mode != ControlMode::kPosition || std::abs(targets.at(i) - state[i].position) <= 0.1,
          "Target beyond max_following_error_rad on joint " + std::to_string(i));
      joints[i].follow(targets[i]);
    }
  }
};
// Each joint powers up away from Home and counts 0 there: its stops are where they really are
// minus that power-up pose. The wrist (index 3) has no stops.
FakeRobot powered_up_robot() {
  const auto joint = [](double lower, double upper, double powered_at) {
    Joint j;
    j.lower = lower - powered_at;
    j.upper = upper - powered_at;
    return j;
  };
  return FakeRobot({joint(-1.0, 1.0, 0.2), joint(-2.0, 0.5, -1.6), joint(-0.1, 1.5, 1.2), joint(-10, 10, 0)});
}
bool near(double value, double expected, double tolerance) { return std::abs(value - expected) <= tolerance; }

// Every joint restarted: the wrist takes its power-up pose, the others find their stop in
// homing_order, and the arm goes to the pose with all joints at once.
void homes_an_arm(const char *spec) {
  const auto config = home_config(spec);
  auto robot = powered_up_robot();
  spark_home::Request request;
  request.home = true;
  request.pose = {{0, 0.0}, {1, -1.5708}, {2, 1.0}, {3, 0.3}};
  std::ostringstream out;
  check(spark_home::home(robot, config, request, out) == 0, "Homing failed: " + out.str());
  check(robot.calls == std::vector<std::string>{"shift 3", "limit 1 6", "shift 1", "limit 1 40", "limit 2 6",
      "shift 2", "limit 2 40", "limit 0 6", "shift 0", "limit 0 40"}, "Homing order: " + out.str());
  // In the true scale now: the stops are where joints.json says.
  check(near(robot.joints[1].lower, -2.0, 1e-9) && near(robot.joints[2].upper, 1.5, 1e-9) &&
      near(robot.joints[0].lower, -1.0, 1e-9), "References do not match stop_rad");
  for (const auto &[index, goal] : request.pose)
    check(near(robot.joints[index].position, goal, 0.0125), "Joint " + std::to_string(index) + " not at the pose");
  check(!robot.enabled && robot.activations == 4, "Not disabled at the end, or activated more than once per step");
  for (std::size_t i = 0; i < 3; ++i)
    check(robot.control[i].mode == ControlMode::kMAXMotionPosition && robot.joints[i].limit == 40,
        "Control mode or current limit not restored");
  check(contains(out.str(), "joint_4: sin tope que buscar") && contains(out.str(), "joint_2: buscando el tope -") &&
      contains(out.str(), "Homing terminado"), out.str());
}
// Only the joints that lost their reference are homed; with every reference kept nothing moves.
void homes_only_what_restarted(const char *spec) {
  const auto config = home_config(spec);
  spark_home::Request request;
  request.home = true;
  request.pose = {{0, 0.0}, {2, 1.0}};

  auto kept = powered_up_robot();
  kept.reference = {true, true, true, true};
  std::ostringstream quiet;
  check(spark_home::home(kept, config, request, quiet) == 0 && kept.calls.empty() && kept.activations == 0 &&
      contains(quiet.str(), "conservan su referencia"), "A referenced arm moved: " + quiet.str());

  auto elbow = powered_up_robot();
  elbow.reference = {true, true, false, true};
  std::ostringstream out;
  check(spark_home::home(elbow, config, request, out) == 0, "Homing failed: " + out.str());
  check(elbow.calls == std::vector<std::string>{"limit 2 6", "shift 2", "limit 2 40"}, "Searched more than joint_3");
  check(near(elbow.joints[0].position, 0.0, 0.0125) && near(elbow.joints[2].position, 1.0, 0.0125), "Pose not reached");

  // Named joints are homed even if referenced, and nothing else gets a reference.
  auto named = powered_up_robot();
  named.reference = {true, false, true, false};
  request.named = {0};
  request.pose = {{0, 0.0}, {1, -1.5708}};
  std::ostringstream one;
  rejects_with([&] { spark_home::home(named, config, request, one); },
      "joint_2 no tiene referencia y no se busca su tope");
  check(named.calls.empty() && named.activations == 0, "Moved before refusing the pose");
  request.pose.clear();
  check(spark_home::home(named, config, request, one) == 0 &&
      named.calls == std::vector<std::string>{"limit 0 6", "shift 0", "limit 0 40"} && !named.reference[3],
      "Named homing: " + one.str());
}
// A failed search leaves that joint and the next ones without reference, and the arm disabled.
void homing_failures(const char *spec) {
  const auto config = home_config(spec);
  spark_home::Request request;
  request.home = true;
  request.pose = {{0, 0.0}};

  auto loose = powered_up_robot();
  loose.joints[1].lower = -10;  // no stop within max_travel_rad
  std::ostringstream out;
  check(spark_home::home(loose, config, request, out) == 1 && contains(out.str(), "max_travel_rad") &&
      contains(out.str(), "joint_2 queda sin referencia"), out.str());
  check(loose.calls == std::vector<std::string>{"shift 3", "limit 1 6", "limit 1 40"} && !loose.enabled &&
      !loose.reference[1] && !loose.reference[2] && !loose.reference[0], "Went on after a failed search");

  auto stopped = powered_up_robot();
  std::ostringstream interrupted;
  spark_home::stop_requested = 1;
  const int code = spark_home::home(stopped, config, request, interrupted);
  spark_home::stop_requested = 0;
  check(code == 130 && !stopped.enabled && stopped.joints[1].limit == 40 && !stopped.reference[1],
      "Interruption not handled: " + interrupted.str());
}
// The status names the SPARKs that restarted, against the same driver the node uses, and leaves
// their HAS_RESET set: the controller_manager still takes (and logs) the power-up pose.
void status(const char *spec) {
  const auto config = config_for(spec, 2);
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  spy->has_reset[2] = false;  // only joint_1's SPARK powered up since its last reference
  MoveMasterDriver driver(config, std::move(bus));
  const auto restarted = driver.configure(SparkResetPolicy::kReport);
  check(restarted == std::vector<std::string>{"joint_1"}, "Restarted SPARKs not reported");
  for (int run = 0; run < 2; ++run) {  // the same answer every time
    std::ostringstream out;
    check(spark_home::status(driver, config, out) == 0, "Status failed");
    check(contains(out.str(), "joint_1  CAN 1  reiniciado") && contains(out.str(), "joint_2  CAN 2  sin reinicio") &&
        contains(out.str(), "sigue puesto"), "Status: " + out.str());
  }
  const auto names = spy->names_sent_to(1);
  check(spy->has_reset[1] && std::count(names.begin(), names.end(), "CLEAR_FAULTS") == 0,
      "The status cleared HAS_RESET");
  rejects_with([&] { driver.activate(); }, "kReport");

  auto quiet = std::make_unique<SimulatedSparkBus>(config);
  quiet->has_reset = {{1, false}, {2, false}};
  MoveMasterDriver kept(config, std::move(quiet));
  check(kept.configure(SparkResetPolicy::kReport).empty(), "A SPARK that kept its reference was reported");
  std::ostringstream out;
  spark_home::status(kept, config, out);
  check(contains(out.str(), "Ningun eje se reinicio"), "Status: " + out.str());
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
    stop_below_stall_current_fails();
    brief_holds_are_not_a_failure();
    ramp();
    measures_an_arm(argv[1]);
    measurement_failures(argv[1]);
    arguments(argv[1]);
    status(argv[1]);
    homes_an_arm(argv[1]);
    homes_only_what_restarted(argv[1]);
    homing_failures(argv[1]);
    std::cout << "PASS: stop search (both stops, moving current, false stop, travel limit, held back without "
        "stall), "
        "spark_home measurement and restore, arguments, status, and homing (order, references, pose, only "
        "what restarted, failures).\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
