// SPARKs simulated at the CAN frame level, shared by the driver and setup tests.
#pragma once
#include "movemaster_hardware/movemaster_driver.hpp"
#include <deque>
#include <iostream>
#include <map>
#include <thread>

namespace movemaster::testing {
inline void check(bool ok, const std::string &message) { if (!ok) throw std::runtime_error(message); }
template<class F> void rejects(F fn, const std::string &message) {
  bool failed = false;
  try { fn(); } catch (const std::exception &) { failed = true; }
  check(failed, message);
}
// Fails unless fn throws a message containing expected.
template<class F> void rejects_with(F fn, const std::string &expected) {
  try { fn(); } catch (const std::exception &e) {
    check(std::string(e.what()).find(expected) != std::string::npos,
        "Expected an error with \"" + expected + "\", got: " + e.what());
    return;
  }
  throw std::runtime_error("Accepted, expected an error with \"" + expected + "\"");
}

// The gripper's ESP32 as its firmware behaves (tools/esp32_tmc2209): STATUS_0 with the wrist
// position, setpoints only with a heartbeat younger than 100 ms, NOT_REFERENCED from power-up
// until SET_WRIST_POSITION. The wrist reaches each accepted setpoint at once.
struct SimulatedGripper {
  int id = -1;                   // device number; -1: no gripper on the bus
  double degrees = 0;            // wrist position
  bool referenced = false;
  bool moving = false;           // WRIST_MOVING in STATUS_0
  bool foreign_heartbeat = false;  // another program sends its heartbeat
  bool muted = false;            // sends no STATUS_0
  double profile_velocity = 0, profile_acceleration = 0;  // last SET_MOTION_PROFILE, deg/s and deg/s^2
  int setpoints = 0;             // accepted WRIST_POSITION_SETPOINTs
  int servo_angle = 90;          // SERVO_ANGLE: last commanded angle (90 at power-up)
  bool servo_active = false;     // the servo has a signal: since its first accepted command
  bool drop_servo = false;       // SERVO_SETPOINT is lost on the way
  int servo_setpoints = 0;       // SERVO_SETPOINT frames accepted
  std::optional<std::chrono::steady_clock::time_point> last_heartbeat;
  bool heartbeat(std::chrono::steady_clock::time_point now) const {
    return foreign_heartbeat || (last_heartbeat && now - *last_heartbeat < std::chrono::milliseconds(100));
  }
};

class SimulatedSparkBus : public CANBus {
 public:
  SparkFrameDatabase frames, gripper_frames;
  SimulatedGripper gripper;
  std::vector<CANPacket> sent;
  std::deque<CANPacket> pending;
  std::vector<int> ids;
  bool telemetry = true, bad_ack = false, drop_ack = false, bad_value = false, reject_ack = false;
  bool malformed = false, nonfinite = false, fail_tx = false, foreign_heartbeat = false, silent_flash = false;
  bool tx_queue_full = false;  // the adapter still holds earlier frames: every send fails with ENOBUFS
  int muted_id = -1;
  int lost_replies = 0;  // the next requests whose reply is lost on the bus
  bool status1 = true, clear_faults = true;       // STATUS_1 is sent; CLEAR_FAULTS clears HAS_RESET
  std::map<int, bool> has_reset;                  // CAN ID -> HAS_RESET sticky; true at power-up
  std::map<int, int> types;                       // parameter ID -> REV type code
  std::map<std::pair<int, int>, std::uint32_t> ram;  // (CAN ID, parameter ID) -> raw value
  // Scripted RESULT_CODEs for the flash commands; 255 (pending) is followed by the next one,
  // and an empty script answers 0.
  std::deque<int> reset_codes, persist_codes;
  std::chrono::steady_clock::time_point last_status{}, last_status1{};
  SimulatedSparkBus(const std::filesystem::path &spec, std::vector<int> device_ids)
      : frames(spec), gripper_frames(spec.parent_path() / kGripperFramesFile), ids(std::move(device_ids)) {
    for (int id : ids) has_reset[id] = true;
    types = {{2, 2}, {6, 2}, {9, 2}, {59, 2}, {60, 2}, {74, 2}, {75, 3}, {112, 3}, {113, 3}, {149, 4}, {158, 2},
        {159, 2}, {160, 2}};
    for (int slot = 0; slot < 4; ++slot) {
      for (int i = 13; i <= 20; ++i) types[i + 8 * slot] = 3;
      for (int i : {166, 167, 169}) types[i + 5 * slot] = 3;
    }
  }
  explicit SimulatedSparkBus(const DriverConfig &config) : SimulatedSparkBus(config.spec_path, device_ids(config)) {
    for (const auto &j : config.joints)
      if (j.device == JointDevice::kGripper) gripper.id = j.device_id;
  }
  static std::vector<int> device_ids(const DriverConfig &config) {
    std::vector<int> result;
    for (const auto &j : config.joints)
      if (j.device == JointDevice::kSpark) result.push_back(j.device_id);
    return result;
  }
  // Name of a frame sent to the gripper, or "" if it is not one.
  std::string gripper_frame_name(const CANPacket &p) const {
    if (gripper.id >= 0)
      for (const auto &entry : gripper_frames)
        if (p.arbitration_id == entry.second.arbitration_id(gripper.id)) return entry.first;
    return "";
  }
  std::vector<std::string> names_sent_to_gripper() const {
    std::vector<std::string> result;
    for (const auto &p : sent)
      if (const auto name = gripper_frame_name(p); !name.empty()) result.push_back(name);
    return result;
  }
  void gripper_receives(const CANPacket &p) {
    const auto name = gripper_frame_name(p);
    const auto now = std::chrono::steady_clock::now();
    if (name == "HEARTBEAT") gripper.last_heartbeat = now;
    else if (name == "SET_MOTION_PROFILE") {
      const auto d = gripper_frames[name].decode_payload(p.data);
      gripper.profile_velocity = d.at("MAX_VELOCITY").get<double>();
      gripper.profile_acceleration = d.at("MAX_ACCELERATION").get<double>();
    } else if (name == "SET_WRIST_POSITION") {
      gripper.degrees = gripper_frames[name].decode_payload(p.data).at("POSITION").get<double>();
      gripper.referenced = true;
    } else if (name == "WRIST_POSITION_SETPOINT" && gripper.heartbeat(now)) {
      gripper.degrees = gripper_frames[name].decode_payload(p.data).at("SETPOINT").get<double>();
      ++gripper.setpoints;
    } else if (name == "SERVO_SETPOINT" && gripper.heartbeat(now) && !gripper.drop_servo) {
      gripper.servo_angle = gripper_frames[name].decode_payload(p.data).at("ANGLE").get<int>();
      gripper.servo_active = true;
      ++gripper.servo_setpoints;
    }
  }
  std::uint32_t value(int id, int parameter) const { return ram.at({id, parameter}); }
  bool written(int id, int parameter) const { return ram.count({id, parameter}) != 0; }
  // Names of the frames sent to one SPARK, in order.
  std::vector<std::string> names_sent_to(int id) const {
    std::vector<std::string> result;
    for (const auto &p : sent)
      if (p.arbitration_id != kEnableHeartbeatId && static_cast<int>(p.arbitration_id & 63) == id)
        for (const auto &entry : frames)
          if ((entry.second.base_arb_id() & ~0x3FU) == (p.arbitration_id & ~0x3FU)) result.push_back(entry.first);
    return result;
  }
  void send(const CANPacket &p, double) override {
    if (fail_tx) throw std::runtime_error("Injected TX failure");
    if (tx_queue_full) throw TxQueueFull("write(SocketCAN): TX queue full");
    sent.push_back(p);
    if (!gripper_frame_name(p).empty()) return gripper_receives(p);
    const int id = p.arbitration_id & 63;
    const auto base = p.arbitration_id & ~0x3FU;
    if (drop_ack) return;
    if (lost_replies > 0) {
      --lost_replies;
      return;
    }
    const auto flash_reply = [&](std::deque<int> &script, const char *response) {
      if (silent_flash) return;
      for (int code = 255; code == 255;) {
        code = script.empty() ? 0 : script.front();
        if (!script.empty()) script.pop_front();
        pending.push_back(frames[response].packet(id, {{"RESULT_CODE", code}}));
      }
    };
    if (base == frames["PARAMETER_WRITE"].base_arb_id()) {
      const auto data = frames["PARAMETER_WRITE"].decode_payload(p.data);
      const int parameter = data.at("PARAMETER_ID").get<int>();
      const auto raw = data.at("VALUE").get<std::uint32_t>();
      if (!reject_ack) ram[{id, parameter}] = raw;
      pending.push_back(frames["PARAMETER_WRITE_RESPONSE"].packet(id,
          {{"PARAMETER_ID", parameter}, {"PARAMETER_TYPE", bad_ack ? 99 : types.at(parameter)},
           {"VALUE", raw ^ (bad_value ? 1U : 0U)}, {"RESULT_CODE", reject_ack ? 1 : 0}}));
    } else if (base == frames["STOP_FOLLOWER_MODE"].base_arb_id())
      pending.push_back(frames["STOP_FOLLOWER_MODE_RESPONSE"].packet(id));
    else if (base == frames["SET_STATUSES_ENABLED"].base_arb_id()) {
      const auto request = frames["SET_STATUSES_ENABLED"].decode_payload(p.data);
      pending.push_back(frames["SET_STATUSES_ENABLED_RESPONSE"].packet(id, {{"RESULT_CODE", 0},
          {"SPECIFIED_MASK", request.at("MASK")}, {"ENABLED_BITFIELD", request.at("ENABLED_BITFIELD")}}));
    } else if (base == frames["CLEAR_FAULTS"].base_arb_id()) {
      if (clear_faults) has_reset[id] = false;
    }
    else if (base == frames["RESET_SAFE_PARAMETERS"].base_arb_id())
      flash_reply(reset_codes, "RESET_SAFE_PARAMETERS_RESPONSE");
    else if (base == frames["PERSIST_PARAMETERS"].base_arb_id())
      flash_reply(persist_codes, "PERSIST_PARAMETERS_RESPONSE");
  }
  void statuses(bool with_status1) {
    for (int id : ids) {
      if (id == muted_id) continue;
      pending.push_back(frames["STATUS_0"].packet(id, {{"CURRENT", 2}, {"PRIMARY_HEARTBEAT_LOCK", true}}));
      if (status1 && with_status1)
        pending.push_back(frames["STATUS_1"].packet(id, {{"HAS_RESET_STICKY_WARNING", has_reset[id]}}));
      auto pos = frames["STATUS_2"].packet(id, {{"PRIMARY_ENCODER_POSITION", id * 0.25},
          {"PRIMARY_ENCODER_VELOCITY", 60.0}});
      if (nonfinite) { pos.data[4] = 0; pos.data[5] = 0; pos.data[6] = 0x80; pos.data[7] = 0x7f; }
      if (malformed) pos.data.resize(1);
      pending.push_back(pos);
    }
    if (foreign_heartbeat) pending.push_back({kEnableHeartbeatId, Bytes(8, 0xFF), true, false, 8, std::nullopt});
    if (gripper.id >= 0 && !gripper.muted) {
      const bool heartbeat = gripper.heartbeat(std::chrono::steady_clock::now());
      pending.push_back(gripper_frames["STATUS_0"].packet(gripper.id, {{"WRIST_POSITION", gripper.degrees},
          {"SERVO_ANGLE", gripper.servo_angle}, {"SERVO_ACTIVE", gripper.servo_active},
          {"DRIVER_ENERGIZED", gripper.last_heartbeat.has_value()},
          {"WRIST_MOVING", gripper.moving}, {"HEARTBEAT_PRESENT", heartbeat}, {"NOT_REFERENCED", !gripper.referenced}}));
    }
  }
  // Telemetry every 5 ms (status_period_ms of config_for); STATUS_1 every 100 ms, as the driver sets it.
  std::optional<CANPacket> recv(double timeout) override {
    const auto now = std::chrono::steady_clock::now();
    if (telemetry && now - last_status >= std::chrono::milliseconds(5)) {
      last_status = now;
      const bool status1_due = now - last_status1 >= std::chrono::milliseconds(100);
      if (status1_due) last_status1 = now;
      statuses(status1_due);
    }
    if (pending.empty()) {
      if (timeout > 0) std::this_thread::sleep_for(std::chrono::duration<double>(std::min(timeout, 0.001)));
      return std::nullopt;
    }
    auto p = pending.front(); pending.pop_front(); return p;
  }
};

inline SlotConfig full_slot() {
  SlotConfig slot;
  slot.pidf = {{"p", 0.05}, {"i", 0}, {"d", 0}, {"f", 0}};
  slot.maxmotion = {{"cruise_velocity", 100}, {"max_acceleration", 200}, {"allowed_profile_error", 0.01}};
  return slot;
}
inline DriverConfig config_for(const std::filesystem::path &spec, int count = 3) {
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
    j.min_position_rad = -10; j.max_position_rad = 10; j.max_velocity_rad_s = 2;
    j.spark = {IdleMode::kBrake, 40};
    j.max_following_error_rad = 0.5;
    j.slots = {{0, full_slot()}};
    c.joints.push_back(j);
  }
  return c;
}
// config_for(spec, sparks) plus the gripper wrist as the last joint: device 5, reversed, offset.
inline DriverConfig config_with_gripper(const std::filesystem::path &spec, int sparks = 2) {
  auto c = config_for(spec, sparks);
  c.feedback_timeout_s = 0.080;  // the gripper's STATUS_0 period is 20 ms
  JointConfig wrist;
  wrist.name = "joint_" + std::to_string(sparks + 1);
  wrist.device = JointDevice::kGripper;
  wrist.device_id = 5;
  wrist.direction = -1; wrist.zero_offset_rad = 0.1;
  wrist.min_position_rad = -3; wrist.max_position_rad = 3;
  wrist.max_velocity_rad_s = 1.5; wrist.max_acceleration_rad_s2 = 3;
  c.joints.push_back(wrist);
  return c;
}
inline float as_float(std::uint32_t raw) { return static_cast<float>(SignalCodec::_u32_to_float(raw)); }
}  // namespace movemaster::testing
