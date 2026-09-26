#include "movemaster_hardware/movemaster_driver.hpp"
#include "movemaster_hardware/socketcan.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <thread>

namespace movemaster {
namespace {
constexpr double tau = 6.283185307179586476925286766559;
constexpr std::size_t rx_budget = 256;
void require(bool condition, const std::string &message) {
  if (!condition) throw std::invalid_argument(message);
}
bool finite(const Json &value) {
  return value.is_number() && std::isfinite(value.get<double>());
}
}
MoveMasterDriver::MoveMasterDriver(DriverConfig config, std::unique_ptr<CANBus> bus)
    : config_(std::move(config)), bus_(std::move(bus)) {
  validate_config();
  for (const auto &joint : config_.joints)
    protocols_.push_back(std::make_unique<SparkMAXMotionProtocol>(
        config_.spec_path, joint.device_id, config_.parameter_layout));
  states_.resize(config_.joints.size());
  status0_at_.resize(states_.size());
  status2_at_.resize(states_.size());
}
void MoveMasterDriver::validate_config() const {
  require(!config_.joints.empty() && config_.joints.size() <= 6, "Configure 1..6 joints");
  require(std::isfinite(config_.period_s) && config_.period_s >= 0.005 && config_.period_s <= 0.05,
      "period_s must be in 0.005..0.050");
  require(std::isfinite(config_.feedback_timeout_s) && config_.feedback_timeout_s >= 3 * config_.period_s,
      "feedback_timeout_s must be >= 3 * period_s");
  require(config_.status_period_ms >= 1 && config_.status_period_ms <= 1000 &&
      3 * config_.status_period_ms / 1000.0 <= config_.feedback_timeout_s, "Invalid STATUS period/watchdog");
  require(std::isfinite(config_.response_timeout_s) && config_.response_timeout_s > 0 &&
      config_.response_timeout_s <= 10, "response_timeout_s must be in (0,10]");
  require(std::isfinite(config_.disable_settle_s) && config_.disable_settle_s >= 0 &&
      config_.disable_settle_s <= 10, "disable_settle_s must be in 0..10");
  require(std::isfinite(config_.max_cycle_gap_s) && config_.max_cycle_gap_s >= 2 * config_.period_s,
      "max_cycle_gap_s must be >= 2 * period_s");
  std::set<int> ids;
  std::set<std::string> names;
  for (const auto &j : config_.joints) {
    require(!j.name.empty() && names.insert(j.name).second, "Duplicate or empty joint name");
    require(j.device_id >= 0 && j.device_id <= 63 && ids.insert(j.device_id).second, "Duplicate or invalid CAN ID");
    require(j.slot >= 0 && j.slot <= 3, j.name + ": invalid slot");
    require(std::isfinite(j.gear_ratio) && j.gear_ratio > 0, j.name + ": gear_ratio must be positive");
    require(j.direction == 1 || j.direction == -1, j.name + ": direction must be +1 or -1");
    require(std::isfinite(j.zero_offset_rad), j.name + ": missing zero_offset_rad");
    require(std::isfinite(j.min_position_rad) && std::isfinite(j.max_position_rad) &&
        j.min_position_rad < j.max_position_rad, j.name + ": invalid position limits");
    for (const auto *key : {"p", "i", "d", "f"})
      require(j.pidf.contains(key) && finite(j.pidf[key]) && j.pidf[key].get<double>() >= 0,
          j.name + ": provide finite, nonnegative PIDF " + key);
    require(j.pidf.size() == 4, j.name + ": PIDF accepts p, i, d, f");
    for (const auto *key : {"cruise_velocity", "max_acceleration", "allowed_profile_error"}) {
      require(j.maxmotion.contains(key) && finite(j.maxmotion[key]), j.name + ": missing MAXMotion " + key);
      const double value = j.maxmotion[key].get<double>();
      require(std::string(key) == "allowed_profile_error" ? value >= 0 : value > 0,
          j.name + ": invalid MAXMotion " + key);
    }
    require(j.maxmotion.size() == 3, j.name + ": unknown MAXMotion parameter");
    // Fail before opening CAN if any required tuning value overflows binary32.
    for (const auto *group : {&j.pidf, &j.maxmotion})
      for (const auto &value : *group) SignalCodec::_float_to_u32(value.get<double>());
  }
}
double MoveMasterDriver::radians_to_rotations(double radians, const JointConfig &j) {
  return j.direction * (radians - j.zero_offset_rad) * j.gear_ratio / tau;
}
double MoveMasterDriver::rotations_to_radians(double rotations, const JointConfig &j) {
  return j.zero_offset_rad + j.direction * rotations * tau / j.gear_ratio;
}
double MoveMasterDriver::rpm_to_rad_s(double rpm, const JointConfig &j) {
  return j.direction * rpm * tau / (60 * j.gear_ratio);
}
void MoveMasterDriver::ensure_healthy() const {
  if (!fault_.empty()) throw std::runtime_error(fault_);
}
void MoveMasterDriver::trip(const std::string &message) noexcept {
  active_ = false;
  if (fault_.empty()) fault_ = message;
}
void MoveMasterDriver::write_checked(SparkMAXMotionProtocol &p, const ParameterDefinition &parameter, const Json &value) {
  const auto response = p.write_parameter(*bus_, parameter, value, config_.response_timeout_s);
  if (!response.at("success").get<bool>() ||
      response.at("parameter_type_code").get<int>() != PARAMETER_TYPE_CODE.at(parameter.value_type) ||
      SparkMAXMotionProtocol::pack_parameter_value(response.at("current_value"), parameter.value_type) !=
          SparkMAXMotionProtocol::pack_parameter_value(value, parameter.value_type))
    throw std::runtime_error("Parameter ACK mismatch: CAN " + std::to_string(p.device_id) + ", " + parameter.key);
}
Json MoveMasterDriver::exchange(SparkMAXMotionProtocol &p, const std::string &request,
    const std::string &response, const Json &values) {
  bus_->send(p.frames[request].packet(p.device_id, values), config_.response_timeout_s);
  const auto &expected = p.frames[response];
  const auto deadline = Clock::now() + std::chrono::duration<double>(config_.response_timeout_s);
  while (Clock::now() < deadline) {
    const double left = std::chrono::duration<double>(deadline - Clock::now()).count();
    const auto rx = bus_->recv(std::max(0.0, std::min(0.01, left)));
    if (rx && rx->is_extended_id && !rx->is_remote_frame && rx->arbitration_id == expected.arbitration_id(p.device_id))
      return expected.decode_payload(rx->data);
  }
  throw TimeoutError("Timeout waiting for " + response + " on CAN " + std::to_string(p.device_id));
}
void MoveMasterDriver::configure() {
  ensure_healthy();
  if (configured_ || active_) throw std::logic_error("Driver already configured");
  try {
    if (!bus_) {
      auto bus = std::make_unique<SocketCAN>(config_.channel);
      std::vector<std::uint32_t> filters;
      for (const auto &p : protocols_)
        for (const auto *name : {"STATUS_0", "STATUS_2", "PARAMETER_WRITE_RESPONSE",
            "STOP_FOLLOWER_MODE_RESPONSE", "SET_STATUSES_ENABLED_RESPONSE"})
          filters.push_back(p->frames[name].arbitration_id(p->device_id));
      bus->set_filters(filters);
      bus_ = std::move(bus);
    }
    // Quiet interval, as in the original backend. No enable heartbeat in setup.
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.disable_settle_s));
    for (std::size_t i = 0; i < protocols_.size(); ++i) {
      auto &p = *protocols_[i];
      const auto &j = config_.joints[i];
      exchange(p, "STOP_FOLLOWER_MODE", "STOP_FOLLOWER_MODE_RESPONSE");
      const auto set = [&](const char *name, int id, const char *type, const Json &value) {
        write_checked(p, {"setup", name, 0, id, type, "", std::nullopt}, value);
      };
      // Auxiliary IDs copied from the supplied backend; not inferred from CAN frame IDs.
      set("feedback_sensor", 9, "uint", 1);
      set("position_factor", 112, "float", 1.0);
      set("velocity_factor", 113, "float", 1.0);
      set("position_wrapping", 149, "bool", false);
      set("status0_period_ms", 158, "uint", config_.status_period_ms);
      set("status2_period_ms", 160, "uint", config_.status_period_ms);
      const auto enabled = exchange(p, "SET_STATUSES_ENABLED", "SET_STATUSES_ENABLED_RESPONSE",
          {{"MASK", 5}, {"ENABLED_BITFIELD", 5}});
      if (enabled.at("RESULT_CODE").get<int>() != 0 || enabled.at("SPECIFIED_MASK").get<int>() != 5 ||
          (enabled.at("ENABLED_BITFIELD").get<int>() & 5) != 5)
        throw std::runtime_error("Cannot enable STATUS_0/2");
      for (const auto &item : j.pidf.items()) write_checked(p, p["pidf"][j.slot][item.key()], item.value());
      for (const auto &item : j.maxmotion.items()) write_checked(p, p["maxmotion"][j.slot][item.key()], item.value());
    }
    configured_ = true;  // RAM only; no PERSIST_PARAMETERS.
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::receive(const CANPacket &packet) {
  if (!packet.is_extended_id || packet.is_remote_frame) return;
  for (std::size_t i = 0; i < protocols_.size(); ++i) {
    const auto &p = *protocols_[i];
    const auto &j = config_.joints[i];
    if (packet.arbitration_id == p.frames["STATUS_2"].arbitration_id(j.device_id)) {
      const auto d = p.frames["STATUS_2"].decode_payload(packet.data);
      const double pos = d.at("PRIMARY_ENCODER_POSITION").get<double>();
      const double vel = d.at("PRIMARY_ENCODER_VELOCITY").get<double>();
      if (!std::isfinite(pos) || !std::isfinite(vel)) throw std::runtime_error("Nonfinite STATUS_2");
      states_[i].position = rotations_to_radians(pos, j);
      states_[i].velocity = rpm_to_rad_s(vel, j);
      if (!std::isfinite(states_[i].position) || !std::isfinite(states_[i].velocity))
        throw std::runtime_error("Joint conversion overflow");
      status2_at_[i] = Clock::now();
      return;
    }
    if (packet.arbitration_id == p.frames["STATUS_0"].arbitration_id(j.device_id)) {
      const auto d = p.frames["STATUS_0"].decode_payload(packet.data);
      states_[i].current = d.at("CURRENT").get<double>();
      states_[i].primary_heartbeat_lock = d.at("PRIMARY_HEARTBEAT_LOCK").get<bool>();
      status0_at_[i] = Clock::now();
      return;
    }
  }
}
bool MoveMasterDriver::feedback_fresh() const {
  const auto now = Clock::now();
  for (std::size_t i = 0; i < states_.size(); ++i)
    for (const auto *at : {&status0_at_[i], &status2_at_[i]})
      if (!*at || std::chrono::duration<double>(now - **at).count() > config_.feedback_timeout_s) return false;
  return true;
}
void MoveMasterDriver::wait_feedback() {
  // Discard queued pre-activation frames before accepting a new measured hold target.
  std::size_t drained = 0;
  while (bus_->recv(0))
    if (++drained >= 4096) throw std::runtime_error("Excessive CAN backlog during activation");
  std::fill(status0_at_.begin(), status0_at_.end(), std::nullopt);
  std::fill(status2_at_.begin(), status2_at_.end(), std::nullopt);
  const auto deadline = Clock::now() + std::chrono::duration<double>(config_.response_timeout_s);
  while (!feedback_fresh()) {
    if (Clock::now() >= deadline) throw TimeoutError("Fresh STATUS_0 and STATUS_2 required for every joint");
    const auto rx = bus_->recv(0.005);
    if (rx) receive(*rx);
  }
}
void MoveMasterDriver::check_cycle_gap() const {
  if (active_ && last_tx_ && std::chrono::duration<double>(Clock::now() - *last_tx_).count() > config_.max_cycle_gap_s)
    throw std::runtime_error("Control loop gap exceeded; refusing automatic re-enable");
}
double MoveMasterDriver::checked_target(double radians, const JointConfig &j) const {
  if (!std::isfinite(radians) || radians < j.min_position_rad || radians > j.max_position_rad)
    throw std::out_of_range(j.name + ": position command outside calibrated limits");
  const double rotations = radians_to_rotations(radians, j);
  if (!std::isfinite(rotations)) throw std::out_of_range(j.name + ": target conversion overflow");
  const double quantized = SignalCodec::_u32_to_float(SignalCodec::_float_to_u32(rotations));
  const double actual = rotations_to_radians(quantized, j);
  if (actual < j.min_position_rad || actual > j.max_position_rad)
    throw std::out_of_range(j.name + ": float32 target outside calibrated limits");
  return quantized;
}
void MoveMasterDriver::activate() {
  ensure_healthy();
  if (!configured_ || active_) throw std::logic_error("Configure once before activation");
  try {
    // Allow the prior heartbeat to expire before loading hold positions on reactivation.
    if (last_tx_) {
      const auto quiet_until = *last_tx_ + std::chrono::duration<double>(config_.disable_settle_s);
      std::this_thread::sleep_until(quiet_until);
    }
    wait_feedback();
    std::vector<double> hold;
    for (const auto &s : states_) hold.push_back(s.position);
    last_tx_.reset();
    active_ = true;
    write(hold);  // All setpoints are loaded before the first global heartbeat.
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::deactivate() noexcept {
  active_ = false;  // Firmware watchdog handles disable; there is no invented stop frame.
}
void MoveMasterDriver::read() {
  ensure_healthy();
  if (!configured_) throw std::logic_error("Driver is not configured");
  try {
    check_cycle_gap();
    std::size_t count = 0;
    while (count < rx_budget) {
      const auto rx = bus_->recv(0);
      if (!rx) break;
      receive(*rx);
      ++count;
    }
    if (count == rx_budget) throw std::runtime_error("RX budget exceeded; feedback could be queued/stale");
    if (active_ && !feedback_fresh()) throw std::runtime_error("STATUS_0/2 watchdog expired");
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::write(const std::vector<double> &positions) {
  ensure_healthy();
  if (!active_) return;
  try {
    check_cycle_gap();
    if (!feedback_fresh()) throw std::runtime_error("STATUS_0/2 watchdog expired");
    require(positions.size() == protocols_.size(), "Incorrect joint command count");
    // Validate the entire vector before transmitting any axis.
    std::vector<CANPacket> packets;
    packets.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i)
      packets.push_back(protocols_[i]->maxmotion_setpoint_packet(
          checked_target(positions[i], config_.joints[i]), config_.joints[i].slot));
    if (last_tx_ && std::chrono::duration<double>(Clock::now() - *last_tx_).count() < config_.period_s) return;
    for (const auto &packet : packets) bus_->send(packet, 0);
    bus_->send(heartbeat_, 0);
    last_tx_ = Clock::now();
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
}  // namespace movemaster
