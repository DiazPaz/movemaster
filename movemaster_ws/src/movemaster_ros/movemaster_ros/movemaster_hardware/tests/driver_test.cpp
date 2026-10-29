#include "simulated_spark_bus.hpp"
#include <cmath>

using namespace movemaster;
using namespace movemaster::testing;
namespace {
void pause_cycle() { std::this_thread::sleep_for(std::chrono::milliseconds(6)); }
void healthy_session(const char *spec, int count) {
  auto config = config_for(spec, count);
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  MoveMasterDriver driver(config, std::move(bus));
  driver.configure();
  for (const auto &p : spy->sent) check(p.arbitration_id != 0x01011840U, "Setup unexpectedly enabled motors");
  spy->sent.clear();
  driver.activate();
  check(driver.active(), "Not active");
  check(spy->sent.size() == static_cast<std::size_t>(count + 1), "Initial setpoint/heartbeat count");
  check(spy->sent.back().arbitration_id == 0x01011840U && spy->sent.back().data == Bytes(8, 0xff), "Heartbeat changed");
  std::vector<double> targets;
  for (int i = 0; i < count; ++i) {
    const auto &s = driver.states()[i];
    const auto &j = config.joints[i];
    const double expected = MoveMasterDriver::rotations_to_radians((i + 1) * 0.25, j);
    check(std::abs(s.position - expected) < 1e-12, "Position conversion incorrect");
    check(std::abs(s.velocity - MoveMasterDriver::rpm_to_rad_s(60, j)) < 1e-12, "Velocity conversion incorrect");
    check(s.primary_heartbeat_lock, "Heartbeat lock state lost");
    const auto d = spy->frames["MAXMOTION_POSITION_SETPOINT"].decode_payload(spy->sent[i].data);
    check(d.at("SETPOINT").get<double>() == (i + 1) * 0.25, "Activation did not hold measured position");
    targets.push_back(expected + 0.05);
    check(std::abs(MoveMasterDriver::rotations_to_radians(MoveMasterDriver::radians_to_rotations(expected, j), j)
        - expected) < 1e-12, "Conversion inverse incorrect");
  }
  pause_cycle(); driver.read(); spy->sent.clear(); driver.write(targets);
  check(spy->sent.size() == static_cast<std::size_t>(count + 1), "Runtime frame count");
  for (int i = 0; i < count; ++i) {
    const double sp = spy->frames["MAXMOTION_POSITION_SETPOINT"].decode_payload(spy->sent[i].data).at("SETPOINT");
    check(std::abs(sp - MoveMasterDriver::radians_to_rotations(targets[i], config.joints[i])) < 1e-6, "Command units incorrect");
  }
  driver.deactivate(); spy->sent.clear(); driver.write(targets);
  check(spy->sent.empty(), "Inactive driver transmitted");
  driver.activate();
  check(driver.active(), "Reactivation failed");
  spy->sent.clear(); targets.back() = 1000;
  rejects([&] { driver.write(targets); }, "Invalid target accepted");
  check(spy->sent.empty() && !driver.active() && !driver.fault().empty(), "Invalid vector transmitted partially or fault not latched");
  rejects([&] { driver.activate(); }, "Fault automatically rearmed");
}
// controller_manager calls write() once per fixed-rate cycle, a little early or late:
// a cycle shorter than period_s must still transmit, a burst far faster must not.
void paced_writes(const char *spec) {
  auto config = config_for(spec, 1);
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  MoveMasterDriver driver(config, std::move(bus));
  driver.configure(); driver.activate();
  const std::vector<double> hold{driver.states()[0].position};
  const auto frames_after = [&](double fraction_of_period) {
    std::this_thread::sleep_for(std::chrono::duration<double>(fraction_of_period * config.period_s));
    driver.read(); spy->sent.clear(); driver.write(hold);
    return spy->sent.size();
  };
  for (int cycle = 0; cycle < 5; ++cycle)
    check(frames_after(0.7) == 2, "Early controller cycle dropped its setpoint and heartbeat");
  check(frames_after(0.2) == 0, "Write burst faster than half a period transmitted");
}
// A Position-only slot: no MAXMotion profile and a reduced PID output.
SlotConfig precise_slot() {
  SlotConfig slot;
  slot.pidf = {{"p", 0.2}, {"i", 0.001}, {"d", 0}, {"f", 0}};
  slot.output_min = -0.3;
  slot.output_max = 0.3;
  return slot;
}
// Decodes the setpoint sent to one SPARK: which frame (the control type) and its slot.
struct Sent { std::string frame; int slot; double setpoint; };
Sent setpoint_sent(const SimulatedSparkBus &bus, const CANPacket &packet) {
  for (const auto *name : {"POSITION_SETPOINT", "MAXMOTION_POSITION_SETPOINT"})
    if ((packet.arbitration_id & ~0x3FU) == (bus.frames[name].base_arb_id() & ~0x3FU)) {
      const auto d = bus.frames[name].decode_payload(packet.data);
      return {name, d.at("PID_SLOT").get<int>(), d.at("SETPOINT").get<double>()};
    }
  throw std::runtime_error("Not a setpoint frame: " + packet.repr());
}
// configure() writes the baseline and every configured slot to RAM, and nothing to flash.
void configures_baseline_and_slots(const char *spec) {
  auto config = config_for(spec, 2);
  config.joints[0].slots[1] = precise_slot();
  config.joints[1].spark = {IdleMode::kCoast, 20};
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  MoveMasterDriver driver(config, std::move(bus));
  driver.configure();
  check(spy->value(1, 2) == 1 && spy->value(2, 2) == 1, "Motor type is not brushless");
  check(spy->value(1, 6) == 1 && spy->value(2, 6) == 0, "Idle mode not written");
  check(spy->value(1, 59) == 40 && spy->value(1, 60) == 40 && spy->value(2, 59) == 20 && spy->value(2, 60) == 20,
      "Current limit not written at stall and free speed");
  check(as_float(spy->value(1, 112)) == 1.0f && as_float(spy->value(1, 113)) == 1.0f,
      "Conversion factors are not 1: the SPARK must stay in motor units");
  check(as_float(spy->value(1, 13)) == 0.05f && as_float(spy->value(1, 13 + 8)) == 0.2f, "Slot gains not written");
  check(as_float(spy->value(1, 19)) == -1.0f && as_float(spy->value(1, 20)) == 1.0f, "Default output range not written");
  check(as_float(spy->value(1, 19 + 8)) == -0.3f && as_float(spy->value(1, 20 + 8)) == 0.3f, "Slot output range not written");
  check(spy->written(1, 166) && !spy->written(1, 166 + 5), "MAXMotion written to a Position-only slot");
  check(!spy->written(2, 13 + 8), "Unconfigured slot written");
  for (const auto &p : spy->sent)
    check((p.arbitration_id & ~0x3FU) != spy->frames["PERSIST_PARAMETERS"].base_arb_id() &&
        (p.arbitration_id & ~0x3FU) != spy->frames["RESET_SAFE_PARAMETERS"].base_arb_id(),
        "configure() touched the flash");
}
// Mode and slot per joint at runtime; Position targets stay near the measured position.
void control_modes(const char *spec) {
  auto config = config_for(spec, 2);
  config.joints[0].slots[1] = precise_slot();
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  MoveMasterDriver driver(config, std::move(bus));
  driver.configure();
  spy->sent.clear();
  driver.activate();
  for (int i : {0, 1})
    check(setpoint_sent(*spy, spy->sent[i]).frame == "MAXMOTION_POSITION_SETPOINT" &&
        setpoint_sent(*spy, spy->sent[i]).slot == 0, "Activation ignored the configured MAXMotion slot");
  rejects_with([&] { driver.set_control(0, ControlMode::kMAXMotionPosition, 1); }, "has no MAXMotion profile");
  rejects_with([&] { driver.set_control(0, ControlMode::kPosition, 2); }, "slot 2 is not configured");
  rejects_with([&] { driver.set_control(2, ControlMode::kPosition, 0); }, "Unknown joint index");
  check(driver.active() && driver.fault().empty(), "A rejected control change tripped the driver");
  driver.set_control(0, ControlMode::kPosition, 1);
  const auto &j1 = config.joints[0], &j2 = config.joints[1];
  pause_cycle(); driver.read(); spy->sent.clear();
  // Joint 2 stays in MAXMotion, where a distant target is the normal case.
  const std::vector<double> near{driver.states()[0].position + 0.4, driver.states()[1].position + 5};
  driver.write(near);
  const auto first = setpoint_sent(*spy, spy->sent[0]), second = setpoint_sent(*spy, spy->sent[1]);
  check(first.frame == "POSITION_SETPOINT" && first.slot == 1, "Joint 1 did not switch to Position slot 1");
  check(std::abs(first.setpoint - MoveMasterDriver::radians_to_rotations(near[0], j1)) < 1e-6, "Position units");
  check(second.frame == "MAXMOTION_POSITION_SETPOINT" && second.slot == 0, "Joint 2 control changed");
  check(std::abs(second.setpoint - MoveMasterDriver::radians_to_rotations(near[1], j2)) < 1e-5, "MAXMotion units");
  pause_cycle(); driver.read(); spy->sent.clear();
  const std::vector<double> far{driver.states()[0].position + 0.6, driver.states()[1].position};
  rejects_with([&] { driver.write(far); }, "max_following_error_rad");
  check(spy->sent.empty() && !driver.active() && !driver.fault().empty(), "Following error transmitted or did not latch");
}
// The configured control applies from activation, and a runtime choice survives reactivation.
void position_at_activation(const char *spec) {
  auto config = config_for(spec, 1);
  config.joints[0].slots[2] = precise_slot();
  config.joints[0].mode = ControlMode::kPosition;
  config.joints[0].slot = 2;
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  MoveMasterDriver driver(config, std::move(bus));
  driver.configure();
  spy->sent.clear();
  driver.activate();
  const auto hold = setpoint_sent(*spy, spy->sent[0]);
  check(hold.frame == "POSITION_SETPOINT" && hold.slot == 2 && hold.setpoint == 0.25,
      "Activation did not hold the measured position in Position slot 2");
  driver.set_control(0, ControlMode::kMAXMotionPosition, 0);
  driver.deactivate();
  spy->sent.clear();
  driver.activate();
  const auto reactivated = setpoint_sent(*spy, spy->sent.at(0));
  check(reactivated.frame == "MAXMOTION_POSITION_SETPOINT" && reactivated.slot == 0,
      "Runtime control lost across reactivation");
}
// A SPARK that restarted counts again from where it powered up. The first configure takes that
// pose as the reference and clears HAS_RESET; afterwards a restart is a fault.
void spark_restarts(const char *spec) {
  auto config = config_for(spec, 2);
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  MoveMasterDriver driver(config, std::move(bus));
  check(driver.configure() == std::vector<std::string>{"joint_1", "joint_2"}, "Powered-up SPARKs not reported");
  check(spy->names_sent_to(1).back() == "CLEAR_FAULTS" && !spy->has_reset[1] && !spy->has_reset[2],
      "HAS_RESET not cleared after taking the power-up pose");
  check(spy->value(1, 159) == 100, "STATUS_1 period not written");
  const auto enable = std::find_if(spy->sent.begin(), spy->sent.end(), [&](const CANPacket &p) {
    return (p.arbitration_id & ~0x3FU) == spy->frames["SET_STATUSES_ENABLED"].base_arb_id(); });
  check(enable != spy->sent.end() && spy->frames["SET_STATUSES_ENABLED"].decode_payload(enable->data).at("MASK") == 7,
      "STATUS_0/1/2 not enabled");
  driver.activate();
  pause_cycle(); driver.read();
  spy->has_reset[2] = true;  // the SPARK of joint_2 reboots mid-session
  std::this_thread::sleep_for(std::chrono::milliseconds(110));  // its next STATUS_1
  spy->sent.clear();
  rejects_with([&] { driver.read(); }, "joint_2: the SPARK restarted");
  rejects([&] { driver.write({0, 0}); }, "A restarted SPARK did not latch a fault");
  check(spy->sent.empty() && !driver.active(), "A restarted SPARK kept the heartbeat");

  // Reconfiguring in the same session: the position was lost, so it is refused and not cleared.
  auto again = config_for(spec, 2);
  auto restarted = std::make_unique<SimulatedSparkBus>(again);
  auto *spy2 = restarted.get();
  spy2->has_reset[1] = false;
  MoveMasterDriver refused(again, std::move(restarted));
  rejects_with([&] { refused.configure(SparkResetPolicy::kFault); }, "joint_2: the SPARK restarted");
  check(spy2->has_reset[2] && spy2->names_sent_to(2).back() != "CLEAR_FAULTS", "A refused restart was cleared");
  auto clean = std::make_unique<SimulatedSparkBus>(again);
  clean->has_reset = {{1, false}, {2, false}};
  MoveMasterDriver kept(again, std::move(clean));
  check(kept.configure(SparkResetPolicy::kFault).empty(), "A SPARK that kept its reference was refused");

  // Firmware that does not clear HAS_RESET, or that sends no STATUS_1, must not pass unnoticed.
  auto stuck = std::make_unique<SimulatedSparkBus>(again);
  stuck->clear_faults = false;
  MoveMasterDriver uncleared(again, std::move(stuck));
  rejects_with([&] { uncleared.configure(); }, "CLEAR_FAULTS did not clear HAS_RESET on CAN 1");
  auto quiet = std::make_unique<SimulatedSparkBus>(again);
  quiet->status1 = false;
  MoveMasterDriver blind(again, std::move(quiet));
  rejects_with([&] { blind.configure(); }, "STATUS_1 required from every SPARK");
}
// The adapter can drop a received frame (an MCP2515 keeps only two): a setup request without
// reply is sent again. A SPARK that never answers still fails, after kSetupAttempts.
void lost_replies(const char *spec) {
  const auto config = config_for(spec, 2);
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  spy->lost_replies = kSetupAttempts - 1;  // STOP_FOLLOWER_MODE of joint_1 and its first retry
  MoveMasterDriver driver(config, std::move(bus));
  driver.configure();
  check(driver.setup_retries() == kSetupAttempts - 1, "Lost replies not retried");
  const auto names = spy->names_sent_to(1);
  check(std::count(names.begin(), names.end(), "STOP_FOLLOWER_MODE") == kSetupAttempts, "Retries not sent");
  driver.activate();
  check(driver.active(), "Not active after recovered replies");

  auto silent = std::make_unique<SimulatedSparkBus>(config);
  silent->lost_replies = kSetupAttempts;
  MoveMasterDriver refused(config, std::move(silent));
  rejects_with([&] { refused.configure(); },
      "Timeout waiting for STOP_FOLLOWER_MODE_RESPONSE on CAN 1 (3 attempts)");
}
// spark_home lowers the Smart Current Limit in RAM while it measures, never while enabled.
void current_limit(const char *spec) {
  auto config = config_for(spec, 2);
  auto bus = std::make_unique<SimulatedSparkBus>(config);
  auto *spy = bus.get();
  MoveMasterDriver driver(config, std::move(bus));
  rejects_with([&] { driver.set_current_limit(0, 6); }, "only while configured and not active");
  driver.configure();
  driver.set_current_limit(1, 6);
  check(spy->value(2, 59) == 6 && spy->value(2, 60) == 6 && spy->value(1, 59) == 40, "Current limit not written");
  rejects_with([&] { driver.set_current_limit(1, 0); }, "1..80 A");
  driver.activate();
  rejects_with([&] { driver.set_current_limit(1, 40); }, "only while configured and not active");
  check(driver.active() && driver.fault().empty(), "A refused current limit tripped the driver");
}
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  try {
    for (int count : {1, 3, 6}) healthy_session(argv[1], count);
    paced_writes(argv[1]);
    configures_baseline_and_slots(argv[1]);
    control_modes(argv[1]);
    position_at_activation(argv[1]);
    spark_restarts(argv[1]);
    current_limit(argv[1]);
    lost_replies(argv[1]);
    auto duplicate = config_for(argv[1]); duplicate.joints[1].device_id = 1;
    rejects([&] { MoveMasterDriver driver(duplicate); }, "Duplicate CAN IDs accepted");
    auto missing = config_for(argv[1]); missing.joints[0].gear_ratio = 0;
    rejects([&] { MoveMasterDriver driver(missing); }, "Zero reduction accepted");
    // ACK success, type, raw value and timeout each independently guard activation.
    for (int fault = 0; fault < 4; ++fault) {
      auto c = config_for(argv[1], 1);
      auto bus = std::make_unique<SimulatedSparkBus>(c); auto *spy = bus.get();
      spy->bad_ack = fault == 0; spy->bad_value = fault == 1;
      spy->drop_ack = fault == 2; spy->reject_ack = fault == 3;
      MoveMasterDriver d(c, std::move(bus));
      rejects([&] { d.configure(); }, "Invalid ACK/setup accepted");
      check(!d.active() && !d.fault().empty(), "Setup failure did not latch");
    }
    // Loss of one axis, malformed payload, NaN/Inf, failed send, paused controller.
    for (int fault = 0; fault < 5; ++fault) {
      auto c = config_for(argv[1]);
      if (fault == 4) c.max_cycle_gap_s = 0.010;
      auto bus = std::make_unique<SimulatedSparkBus>(c); auto *spy = bus.get();
      MoveMasterDriver d(c, std::move(bus)); d.configure(); d.activate();
      spy->sent.clear();
      if (fault == 0) {
        spy->muted_id = 2;
        std::this_thread::sleep_for(std::chrono::milliseconds(45));
        rejects([&] { d.read(); }, "Missing joint feedback accepted");
      } else if (fault == 1 || fault == 2) {
        spy->malformed = fault == 1; spy->nonfinite = fault == 2;
        pause_cycle();
        rejects([&] { d.read(); }, "Invalid telemetry accepted");
      } else if (fault == 3) {
        pause_cycle(); d.read(); spy->fail_tx = true;
        rejects([&] { d.write({0.1, 0.1, 0.1}); }, "TX failure ignored");
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        rejects([&] { d.read(); }, "Paused loop resumed automatically");
      }
      check(!d.active() && !d.fault().empty(), "Runtime failure did not latch");
      for (const auto &p : spy->sent) check(p.arbitration_id != 0x01011840U, "Heartbeat continued after failure");
    }
    std::cout << "PASS: 1/3/6 axes, conversions, ACK checks, activation order, limits, paced writes, "
        "baseline and slots in RAM, Position/MAXMotion per joint, following error, "
        "stale/malformed feedback, TX failure, loop gap, SPARK restarts, current limit and lost replies.\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
