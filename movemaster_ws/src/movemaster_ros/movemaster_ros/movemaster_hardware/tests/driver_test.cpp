#include "movemaster_hardware/movemaster_driver.hpp"
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <thread>

using namespace movemaster;
namespace {
void check(bool ok, const std::string &message) { if (!ok) throw std::runtime_error(message); }
template<class F> void rejects(F fn, const std::string &message) {
  bool failed = false;
  try { fn(); } catch (const std::exception &) { failed = true; }
  check(failed, message);
}
class SimulatedSparkBus : public CANBus {
 public:
  SparkFrameDatabase frames;
  std::vector<CANPacket> sent;
  std::deque<CANPacket> pending;
  std::vector<int> ids;
  bool telemetry = true, bad_ack = false, drop_ack = false, bad_value = false, reject_ack = false;
  bool malformed = false, nonfinite = false, fail_tx = false;
  int muted_id = -1;
  std::map<int, int> types;
  std::chrono::steady_clock::time_point last_status{};
  explicit SimulatedSparkBus(const DriverConfig &config) : frames(config.spec_path) {
    for (const auto &j : config.joints) ids.push_back(j.device_id);
    types = {{9, 2}, {112, 3}, {113, 3}, {149, 4}, {158, 2}, {160, 2}};
    for (int slot = 0; slot < 4; ++slot) {
      for (int i = 13; i <= 16; ++i) types[i + 8 * slot] = 3;
      for (int i : {166, 167, 169}) types[i + 5 * slot] = 3;
    }
  }
  void send(const CANPacket &p, double) override {
    if (fail_tx) throw std::runtime_error("Injected TX failure");
    sent.push_back(p);
    const int id = p.arbitration_id & 63;
    const auto base = p.arbitration_id & ~0x3FU;
    if (drop_ack) return;
    if (base == frames["PARAMETER_WRITE"].base_arb_id()) {
      const auto data = frames["PARAMETER_WRITE"].decode_payload(p.data);
      const int parameter = data.at("PARAMETER_ID").get<int>();
      pending.push_back(frames["PARAMETER_WRITE_RESPONSE"].packet(id,
          {{"PARAMETER_ID", parameter}, {"PARAMETER_TYPE", bad_ack ? 99 : types.at(parameter)},
           {"VALUE", data.at("VALUE").get<std::uint32_t>() ^ (bad_value ? 1U : 0U)},
           {"RESULT_CODE", reject_ack ? 1 : 0}}));
    } else if (base == frames["STOP_FOLLOWER_MODE"].base_arb_id())
      pending.push_back(frames["STOP_FOLLOWER_MODE_RESPONSE"].packet(id));
    else if (base == frames["SET_STATUSES_ENABLED"].base_arb_id())
      pending.push_back(frames["SET_STATUSES_ENABLED_RESPONSE"].packet(id,
          {{"RESULT_CODE", 0}, {"SPECIFIED_MASK", 5}, {"ENABLED_BITFIELD", 5}}));
  }
  void statuses() {
    for (int id : ids) {
      if (id == muted_id) continue;
      pending.push_back(frames["STATUS_0"].packet(id, {{"CURRENT", 2}, {"PRIMARY_HEARTBEAT_LOCK", true}}));
      auto pos = frames["STATUS_2"].packet(id, {{"PRIMARY_ENCODER_POSITION", id * 0.25},
          {"PRIMARY_ENCODER_VELOCITY", 60.0}});
      if (nonfinite) { pos.data[4] = 0; pos.data[5] = 0; pos.data[6] = 0x80; pos.data[7] = 0x7f; }
      if (malformed) pos.data.resize(1);
      pending.push_back(pos);
    }
  }
  std::optional<CANPacket> recv(double timeout) override {
    const auto now = std::chrono::steady_clock::now();
    if (telemetry && now - last_status >= std::chrono::milliseconds(5)) {
      last_status = now;
      statuses();
    }
    if (pending.empty()) {
      if (timeout > 0) std::this_thread::sleep_for(std::chrono::duration<double>(std::min(timeout, 0.001)));
      return std::nullopt;
    }
    auto p = pending.front(); pending.pop_front(); return p;
  }
};
DriverConfig config_for(const char *spec, int count = 3) {
  DriverConfig c;
  c.spec_path = spec;
  c.period_s = 0.005;
  c.status_period_ms = 5;
  c.response_timeout_s = 0.035;
  c.disable_settle_s = 0;
  c.feedback_timeout_s = 0.040;
  c.max_cycle_gap_s = 0.200;
  for (int id = 1; id <= count; ++id) {
    JointConfig j;
    j.name = "joint_" + std::to_string(id); j.device_id = id;
    j.gear_ratio = 10; j.direction = id % 2 ? 1 : -1; j.zero_offset_rad = 0.1;
    j.min_position_rad = -10; j.max_position_rad = 10;
    j.pidf = {{"p", 0.05}, {"i", 0}, {"d", 0}, {"f", 0}};
    j.maxmotion = {{"cruise_velocity", 100}, {"max_acceleration", 200}, {"allowed_profile_error", 0.01}};
    c.joints.push_back(j);
  }
  return c;
}
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
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  try {
    for (int count : {1, 3, 6}) healthy_session(argv[1], count);
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
    std::cout << "PASS: 1/3/6 axes, conversions, ACK checks, activation order, limits, stale/malformed feedback, TX failure and loop gap.\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
