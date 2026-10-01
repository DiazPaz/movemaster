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
void validate_driver_config(const DriverConfig &config) {
  require(!config.joints.empty() && config.joints.size() <= 6, "Configure 1..6 joints");
  require(std::isfinite(config.period_s) && config.period_s >= 0.005 && config.period_s <= 0.05,
      "period_s must be in 0.005..0.050");
  require(std::isfinite(config.feedback_timeout_s) && config.feedback_timeout_s >= 3 * config.period_s,
      "feedback_timeout_s must be >= 3 * period_s");
  require(config.status_period_ms >= 1 && config.status_period_ms <= 1000 &&
      3 * config.status_period_ms / 1000.0 <= config.feedback_timeout_s, "Invalid STATUS period/watchdog");
  require(std::isfinite(config.response_timeout_s) && config.response_timeout_s > 0 &&
      config.response_timeout_s <= 10, "response_timeout_s must be in (0,10]");
  require(std::isfinite(config.disable_settle_s) && config.disable_settle_s >= 0 &&
      config.disable_settle_s <= 10, "disable_settle_s must be in 0..10");
  require(std::isfinite(config.max_cycle_gap_s) && config.max_cycle_gap_s >= 2 * config.period_s,
      "max_cycle_gap_s must be >= 2 * period_s");
  std::set<int> ids;
  std::set<std::string> names;
  for (const auto &j : config.joints) {
    require(!j.name.empty() && names.insert(j.name).second, "Duplicate or empty joint name");
    require(j.device_id >= 0 && j.device_id <= 63 && ids.insert(j.device_id).second, "Duplicate or invalid CAN ID");
    require(std::isfinite(j.gear_ratio) && j.gear_ratio > 0, j.name + ": gear_ratio must be positive");
    require(j.direction == 1 || j.direction == -1, j.name + ": direction must be +1 or -1");
    require(std::isfinite(j.zero_offset_rad), j.name + ": missing zero_offset_rad");
    require(std::isfinite(j.min_position_rad) && std::isfinite(j.max_position_rad) &&
        j.min_position_rad < j.max_position_rad, j.name + ": invalid position limits");
    require(std::isfinite(j.max_velocity_rad_s) && j.max_velocity_rad_s > 0,
        j.name + ": max_velocity_rad_s must be positive");
    require(j.spark.idle_mode == IdleMode::kCoast || j.spark.idle_mode == IdleMode::kBrake,
        j.name + ": invalid idle mode");
    require(j.spark.current_limit_a >= 1 && j.spark.current_limit_a <= 80,
        j.name + ": current_limit_a must be in 1..80 A");
    require(std::isfinite(j.max_following_error_rad) && j.max_following_error_rad > 0,
        j.name + ": max_following_error_rad must be positive");
    require(!j.slots.empty(), j.name + ": configure at least one slot");
    for (const auto &[slot, s] : j.slots) {
      const auto where = j.name + " slot " + std::to_string(slot);
      require(slot >= 0 && slot <= 3, where + ": slots are 0..3");
      require(s.pidf.is_object() && s.pidf.size() == 4, where + ": PIDF accepts p, i, d, f");
      for (const auto *key : {"p", "i", "d", "f"})
        require(s.pidf.contains(key) && finite(s.pidf[key]) && s.pidf[key].get<double>() >= 0,
            where + ": provide finite, nonnegative PIDF " + key);
      require(s.output_min >= -1 && s.output_min < 0 && s.output_max > 0 && s.output_max <= 1,
          where + ": output_range must satisfy -1 <= min < 0 < max <= 1");
      if (!s.maxmotion.is_null()) {
        require(s.maxmotion.is_object() && s.maxmotion.size() == 3, where + ": unknown MAXMotion parameter");
        for (const auto *key : {"cruise_velocity", "max_acceleration", "allowed_profile_error"}) {
          require(s.maxmotion.contains(key) && finite(s.maxmotion[key]), where + ": missing MAXMotion " + key);
          const double value = s.maxmotion[key].get<double>();
          require(std::string(key) == "allowed_profile_error" ? value >= 0 : value > 0,
              where + ": invalid MAXMotion " + key);
        }
        // MAXMotion may not move the joint faster than the limit MoveIt plans with.
        const double cruise_rad_s = s.maxmotion["cruise_velocity"].get<double>() * tau / (60 * j.gear_ratio);
        require(cruise_rad_s <= j.max_velocity_rad_s * (1 + 1e-9),
            where + ": MAXMotion cruise_velocity exceeds max_velocity_rad_s");
      }
      // Fail before opening CAN if any tuning value overflows binary32.
      for (const auto *group : {&s.pidf, &s.maxmotion})
        if (!group->is_null())
          for (const auto &value : *group) SignalCodec::_float_to_u32(value.get<double>());
    }
    const auto selected = j.slots.find(j.slot);
    require(selected != j.slots.end(), j.name + ": control slot " + std::to_string(j.slot) + " is not configured");
    require(j.mode == ControlMode::kPosition || !selected->second.maxmotion.is_null(),
        j.name + ": MAXMotion control needs a maxmotion block in slot " + std::to_string(j.slot));
  }
}
MoveMasterDriver::MoveMasterDriver(DriverConfig config, std::unique_ptr<CANBus> bus)
    : config_(std::move(config)), bus_(std::move(bus)) {
  validate_driver_config(config_);
  for (const auto &joint : config_.joints) {
    protocols_.push_back(std::make_unique<SparkMAXMotionProtocol>(
        config_.spec_path, joint.device_id, config_.parameter_layout));
    controls_.push_back({joint.mode, joint.slot});
  }
  states_.resize(config_.joints.size());
  status0_at_.resize(states_.size());
  status2_at_.resize(states_.size());
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
      const auto &j = config_.joints[i];
      const SparkSetup spark(*bus_, *protocols_[i], config_.response_timeout_s);
      spark.exchange("STOP_FOLLOWER_MODE", "STOP_FOLLOWER_MODE_RESPONSE");
      // Also persisted by spark_commission; written again so RAM never depends on it.
      spark.write_baseline(j.spark);
      // Factors of 1 keep the SPARK in motor rotations and RPM: gear_ratio, direction and
      // zero_offset_rad convert to the joint here, once.
      spark.write(rev::kFeedbackSensor, 1);
      spark.write(rev::kPositionConversionFactor, 1.0);
      spark.write(rev::kVelocityConversionFactor, 1.0);
      spark.write(rev::kPositionWrapping, false);
      spark.write(rev::kStatus0Period, config_.status_period_ms);
      spark.write(rev::kStatus2Period, config_.status_period_ms);
      const auto enabled = spark.exchange("SET_STATUSES_ENABLED", "SET_STATUSES_ENABLED_RESPONSE",
          {{"MASK", 5}, {"ENABLED_BITFIELD", 5}});
      if (enabled.at("RESULT_CODE").get<int>() != 0 || enabled.at("SPECIFIED_MASK").get<int>() != 5 ||
          (enabled.at("ENABLED_BITFIELD").get<int>() & 5) != 5)
        throw std::runtime_error("Cannot enable STATUS_0/2");
      for (const auto &[slot, slot_config] : j.slots) spark.write_slot(slot, slot_config);
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
void MoveMasterDriver::set_control(std::size_t joint, ControlMode mode, int slot) {
  require(joint < config_.joints.size(), "Unknown joint index");
  const auto &j = config_.joints[joint];
  const auto found = j.slots.find(slot);
  require(found != j.slots.end(), j.name + ": slot " + std::to_string(slot) + " is not configured");
  require(mode == ControlMode::kPosition || !found->second.maxmotion.is_null(),
      j.name + ": slot " + std::to_string(slot) + " has no MAXMotion profile");
  controls_[joint] = {mode, slot};
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
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const auto &j = config_.joints[i];
      const auto &control = controls_[i];
      const double rotations = checked_target(positions[i], j);
      if (control.mode == ControlMode::kPosition &&
          !(std::abs(positions[i] - states_[i].position) <= j.max_following_error_rad))
        throw std::out_of_range(j.name + ": Position target exceeds max_following_error_rad from the measured position");
      packets.push_back(protocols_[i]->setpoint_packet(control.mode, rotations, control.slot));
    }
    // A caller paced at period_s (the controller_manager loop) wakes up a little early or late
    // every cycle; requiring a full period would drop about a third of the cycles.
    if (last_tx_ && std::chrono::duration<double>(Clock::now() - *last_tx_).count() < 0.5 * config_.period_s) return;
    for (const auto &packet : packets) bus_->send(packet, 0);
    bus_->send(heartbeat_, 0);
    last_tx_ = Clock::now();
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
}  // namespace movemaster
