#include "movemaster_hardware/movemaster_driver.hpp"
#include "movemaster_hardware/socketcan.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <thread>

namespace movemaster {
namespace {
constexpr double tau = 6.283185307179586476925286766559;
constexpr std::size_t rx_budget = 256;
constexpr int kStatus1PeriodMs = 100;  // faults and warnings, HAS_RESET among them
// The ESP32 applies a new servo angle within 50 ms (ESPERA_SERVO_US in its firmware) and reports
// it in its next STATUS_0; an angle still not reported after this long is sent again.
constexpr double kServoRetryS = 0.1;
void require(bool condition, const std::string &message) {
  if (!condition) throw std::invalid_argument(message);
}
std::string join(const std::vector<std::string> &names) {
  std::string text;
  for (const auto &name : names) text += (text.empty() ? "" : ", ") + name;
  return text;
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
  // A SPARK and the gripper may share a number: their IDs differ in device type and manufacturer.
  std::set<std::pair<JointDevice, int>> ids;
  std::set<std::string> names;
  for (const auto &j : config.joints) {
    require(!j.name.empty() && names.insert(j.name).second, "Duplicate or empty joint name");
    require(j.device_id >= 0 && j.device_id <= 63 && ids.insert({j.device, j.device_id}).second,
        "Duplicate or invalid CAN ID");
    require(j.direction == 1 || j.direction == -1, j.name + ": direction must be +1 or -1");
    require(std::isfinite(j.zero_offset_rad), j.name + ": missing zero_offset_rad");
    require(std::isfinite(j.min_position_rad) && std::isfinite(j.max_position_rad) &&
        j.min_position_rad < j.max_position_rad, j.name + ": invalid position limits");
    require(std::isfinite(j.max_velocity_rad_s) && j.max_velocity_rad_s > 0,
        j.name + ": max_velocity_rad_s must be positive");
    if (j.device == JointDevice::kGripper) {
      for (const double limit : {j.min_position_rad, j.max_position_rad})
        require(std::abs(MoveMasterDriver::radians_to_wrist_degrees(limit, j)) <= kGripperMaxDegrees * (1 + 1e-9),
            j.name + ": the gripper firmware takes positions within 360 degrees of its zero; check "
            "min_position_rad and max_position_rad");
      require(std::isfinite(j.max_acceleration_rad_s2) && j.max_acceleration_rad_s2 > 0,
          j.name + ": max_acceleration_rad_s2 must be positive");
      continue;
    }
    require(std::isfinite(j.gear_ratio) && j.gear_ratio > 0, j.name + ": gear_ratio must be positive");
    require(std::isnan(j.max_acceleration_rad_s2) ||
        (std::isfinite(j.max_acceleration_rad_s2) && j.max_acceleration_rad_s2 > 0),
        j.name + ": max_acceleration_rad_s2 must be positive");
    require(j.spark.idle_mode == IdleMode::kCoast || j.spark.idle_mode == IdleMode::kBrake,
        j.name + ": invalid idle mode");
    require(j.spark.current_limit_a >= 1 && j.spark.current_limit_a <= 80,
        j.name + ": current_limit_a must be in 1..80 A");
    // The SPARK MAX runs from 5.5 to 24 V; a nominal voltage outside that range cannot be met.
    require(!j.spark.voltage_compensation_v || (std::isfinite(*j.spark.voltage_compensation_v) &&
        *j.spark.voltage_compensation_v >= 5.5 && *j.spark.voltage_compensation_v <= 24),
        j.name + ": voltage_compensation_v must be in 5.5..24 V");
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
        const double acceleration_rad_s2 =
            s.maxmotion["max_acceleration"].get<double>() * tau / (60 * j.gear_ratio);
        require(std::isnan(j.max_acceleration_rad_s2) ||
            acceleration_rad_s2 <= j.max_acceleration_rad_s2 * (1 + 1e-9),
            where + ": MAXMotion max_acceleration exceeds max_acceleration_rad_s2");
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
    if (j.homing) {
      const auto &h = *j.homing;
      const auto where = j.name + " homing";
      require(h.current_limit_a >= 1 && h.current_limit_a <= j.spark.current_limit_a,
          where + ": current_limit_a must be in 1..spark.current_limit_a");
      require(std::isfinite(h.stall_current_a) && h.stall_current_a > 0 && h.stall_current_a < h.current_limit_a,
          where + ": stall_current_a must be between 0 and current_limit_a");
      require(std::isfinite(h.stall_time_s) && h.stall_time_s >= 3 * config.period_s && h.stall_time_s <= 5,
          where + ": stall_time_s must be in 3 * period_s..5 s");
      require(std::isfinite(h.slow_speed_rad_s) && h.slow_speed_rad_s > 0 &&
          h.slow_speed_rad_s <= h.fast_speed_rad_s && h.fast_speed_rad_s <= j.max_velocity_rad_s,
          where + ": speeds must satisfy 0 < slow_speed_rad_s <= fast_speed_rad_s <= max_velocity_rad_s");
      require(std::isfinite(h.backoff_rad) && h.backoff_rad > 0, where + ": backoff_rad must be positive");
      // Shorter, the backoff could end with the joint still on the stop.
      require(h.backoff_rad >= min_backoff_rad(j.max_following_error_rad) * (1 - 1e-9),
          where + ": backoff_rad must be at least max_following_error_rad / 4");
      require(std::isfinite(h.max_travel_rad) && h.max_travel_rad > 2 * h.backoff_rad,
          where + ": max_travel_rad must exceed twice backoff_rad");
      require(h.stop == 0 || h.stop == 1 || h.stop == -1, where + ": stop must be + or -");
      if (h.stop != 0) {
        // spark_home --home leaves the joint backoff_rad away from its stop, under control.
        const double rest = h.stop_rad - h.stop * h.backoff_rad;
        require(std::isfinite(h.stop_rad) && rest >= j.min_position_rad && rest <= j.max_position_rad,
            where + ": backoff_rad away from stop_rad, the joint must be within min/max_position_rad");
      }
    }
  }
  std::set<std::string> homed;
  for (const auto &name : config.homing_order) {
    const auto joint = std::find_if(config.joints.begin(), config.joints.end(),
        [&](const JointConfig &j) { return j.name == name; });
    require(joint != config.joints.end(), "homing_order: " + name + " is not a configured joint");
    require(homed.insert(name).second, "homing_order: " + name + " appears twice");
    require(joint->homing && joint->homing->stop != 0,
        "homing_order: " + name + " needs homing.stop and homing.stop_rad");
  }
  if (config.gripper) {
    const auto &g = *config.gripper;
    require(std::any_of(config.joints.begin(), config.joints.end(), [&](const JointConfig &j) {
          return j.device == JointDevice::kGripper && j.device_id == g.device_id; }),
        "gripper.can_id must be the can_id of the joint with \"device\": \"gripper\": the servo is on that ESP32");
    for (const double angle : {g.closed_deg, g.open_deg})
      require(std::isfinite(angle) && angle >= 0 && angle <= 180, "gripper: closed_deg and open_deg must be in 0..180");
    require(g.closed_deg != g.open_deg, "gripper: closed_deg and open_deg must differ");
  }
}
MoveMasterDriver::MoveMasterDriver(DriverConfig config, std::unique_ptr<CANBus> bus)
    : config_(std::move(config)), bus_(std::move(bus)) {
  validate_driver_config(config_);
  for (const auto &joint : config_.joints) {
    controls_.push_back({joint.mode, joint.slot});
    if (joint.device == JointDevice::kGripper) {
      if (!gripper_frames_) {
        gripper_frames_ = std::make_unique<SparkFrameDatabase>(config_.spec_path.parent_path() / kGripperFramesFile);
        gripper_status_period_s_ = (*gripper_frames_)["STATUS_0"].get("defaultPeriodMs").get<double>() / 1000;
        require(3 * gripper_status_period_s_ <= config_.feedback_timeout_s,
            "feedback_timeout_s must cover three gripper STATUS_0 periods");
      }
      protocols_.push_back(nullptr);
      status_ids_.push_back({(*gripper_frames_)["STATUS_0"].arbitration_id(joint.device_id), 0, 0});
      gripper_heartbeats_.push_back((*gripper_frames_)["HEARTBEAT"].packet(joint.device_id));
      continue;
    }
    has_spark_ = true;
    protocols_.push_back(std::make_unique<SparkMAXMotionProtocol>(
        config_.spec_path, joint.device_id, config_.parameter_layout));
    const auto &frames = protocols_.back()->frames;
    status_ids_.push_back({frames["STATUS_0"].arbitration_id(joint.device_id),
        frames["STATUS_1"].arbitration_id(joint.device_id), frames["STATUS_2"].arbitration_id(joint.device_id)});
  }
  states_.resize(config_.joints.size());
  referenced_.assign(states_.size(), false);
  status0_at_.resize(states_.size());
  status1_at_.resize(states_.size());
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
double MoveMasterDriver::radians_to_wrist_degrees(double radians, const JointConfig &j) {
  return j.direction * (radians - j.zero_offset_rad) * 360 / tau;
}
double MoveMasterDriver::wrist_degrees_to_radians(double degrees, const JointConfig &j) {
  return j.zero_offset_rad + j.direction * degrees * tau / 360;
}
void MoveMasterDriver::ensure_healthy() const {
  if (!fault_.empty()) throw std::runtime_error(fault_);
}
void MoveMasterDriver::require_spark(std::size_t joint) const {
  require(joint < config_.joints.size(), "Unknown joint index");
  require(config_.joints[joint].device == JointDevice::kSpark,
      config_.joints[joint].name + ": the gripper wrist has no SPARK control modes or current limit");
}
void MoveMasterDriver::trip(const std::string &message) noexcept {
  active_ = false;
  if (fault_.empty()) fault_ = message;
}
std::vector<std::string> MoveMasterDriver::configure(SparkResetPolicy policy) {
  ensure_healthy();
  if (configured_ || active_) throw std::logic_error("Driver already configured");
  try {
    if (!bus_) {
      auto bus = std::make_unique<SocketCAN>(config_.channel);
      std::vector<std::uint32_t> filters;
      for (std::size_t i = 0; i < protocols_.size(); ++i) {
        if (!protocols_[i]) {
          filters.push_back(status_ids_[i].status0);  // the gripper sends only STATUS_0
          continue;
        }
        const auto &p = *protocols_[i];
        for (const auto *name : {"STATUS_0", "STATUS_1", "STATUS_2", "PARAMETER_WRITE_RESPONSE",
            "STOP_FOLLOWER_MODE_RESPONSE", "SET_STATUSES_ENABLED_RESPONSE"})
          filters.push_back(p.frames[name].arbitration_id(p.device_id));
      }
      bus->set_filters(filters);
      bus_ = std::move(bus);
    }
    // Quiet interval, as in the original backend. No enable heartbeat in setup.
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.disable_settle_s));
    // Frames queued during the interval may still show this driver's previous heartbeat.
    for (std::size_t drained = 0; bus_->recv(0);)
      if (++drained >= 4096) throw std::runtime_error("Excessive CAN backlog during setup");
    for (std::size_t i = 0; i < protocols_.size(); ++i) {
      const auto &j = config_.joints[i];
      if (!protocols_[i]) {
        // The gripper does not answer configuration frames; its STATUS_0 is the check.
        const double deg = 360 / tau;
        bus_->send((*gripper_frames_)["SET_MOTION_PROFILE"].packet(j.device_id,
            {{"MAX_VELOCITY", j.max_velocity_rad_s * deg}, {"MAX_ACCELERATION", j.max_acceleration_rad_s2 * deg}}),
            config_.response_timeout_s);
        continue;
      }
      const SparkSetup spark(*bus_, *protocols_[i], config_.response_timeout_s);
      spark.exchange("STOP_FOLLOWER_MODE", "STOP_FOLLOWER_MODE_RESPONSE");
      // Also persisted by spark_commission; written again so RAM never depends on it.
      spark.write_baseline(j.spark);
      // Factors of 1 keep the SPARK in motor rotations and RPM: gear_ratio, direction and
      // zero_offset_rad convert to the joint here, once.
      spark.write(rev::kFeedbackSensor, rev::kMainEncoder);
      spark.write(rev::kPositionConversionFactor, 1.0);
      spark.write(rev::kVelocityConversionFactor, 1.0);
      spark.write(rev::kPositionWrapping, false);
      spark.write(rev::kStatus0Period, config_.status_period_ms);
      spark.write(rev::kStatus1Period, kStatus1PeriodMs);
      spark.write(rev::kStatus2Period, config_.status_period_ms);
      const auto enabled = spark.exchange("SET_STATUSES_ENABLED", "SET_STATUSES_ENABLED_RESPONSE",
          {{"MASK", 7}, {"ENABLED_BITFIELD", 7}});
      if (enabled.at("RESULT_CODE").get<int>() != 0 || enabled.at("SPECIFIED_MASK").get<int>() != 7 ||
          (enabled.at("ENABLED_BITFIELD").get<int>() & 7) != 7)
        throw std::runtime_error("Cannot enable STATUS_0/1/2");
      for (const auto &[slot, slot_config] : j.slots) spark.write_slot(slot, slot_config);
      setup_retries_ += spark.retries();
    }
    std::vector<std::size_t> all(protocols_.size()), restarted;
    std::iota(all.begin(), all.end(), 0);
    wait_status1(all, false);
    // This driver must be the only sender of the gripper heartbeat, as of the global one: a
    // heartbeat from a terminal would keep the wrist enabled after the driver stops.
    for (const auto i : all)
      if (!protocols_[i] && states_[i].heartbeat_present)
        throw std::runtime_error(config_.joints[i].name + ": another program is sending the gripper HEARTBEAT; "
            "stop it (for example a cansend loop) before starting the driver");
    std::vector<std::string> names, sparks, grippers;
    for (const auto i : all)
      if (states_[i].spark_reset) {
        restarted.push_back(i);
        names.push_back(config_.joints[i].name);
        (protocols_[i] ? sparks : grippers).push_back(config_.joints[i].name);
      }
    if (!names.empty() && policy == SparkResetPolicy::kFault) {
      std::string message;
      if (!sparks.empty())
        message = join(sparks) + ": the SPARK restarted (power cut or emergency stop) and lost its position";
      if (!grippers.empty())
        message += (message.empty() ? "" : "; ") + join(grippers) +
            ": the gripper ESP32 restarted (power cut or reset) and lost its position";
      throw std::runtime_error(message + "; reference it again (spark_home --home, which the bringup runs) "
          "and restart the node");
    }
    report_only_ = policy == SparkResetPolicy::kReport;
    if (policy == SparkResetPolicy::kAcceptPowerOnPose) {
      // Cleared now, HAS_RESET (NOT_REFERENCED in the gripper) flags the next restart. The gripper
      // keeps counting from its power-up pose: it is told the position it reports.
      for (const auto i : restarted) {
        const auto &j = config_.joints[i];
        bus_->send(protocols_[i] ? protocols_[i]->frames["CLEAR_FAULTS"].packet(j.device_id)
                                 : (*gripper_frames_)["SET_WRIST_POSITION"].packet(j.device_id,
                                       {{"POSITION", radians_to_wrist_degrees(states_[i].position, j)}}),
            config_.response_timeout_s);
      }
      wait_status1(restarted, true);
    }
    // kHoming leaves the restarted joints unreferenced; kReport references none, so read() never
    // fails on a restart.
    for (const auto i : all) referenced_[i] = !report_only_ && !states_[i].spark_reset;
    configured_ = true;  // RAM only; no PERSIST_PARAMETERS.
    return names;
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::receive_gripper(std::size_t i, const CANPacket &packet) {
  const auto d = (*gripper_frames_)["STATUS_0"].decode_payload(packet.data);
  const double degrees = d.at("WRIST_POSITION").get<double>();
  if (!std::isfinite(degrees)) throw std::runtime_error("Nonfinite gripper STATUS_0");
  const auto &j = config_.joints[i];
  auto &s = states_[i];
  const auto now = Clock::now();
  const double position = wrist_degrees_to_radians(degrees, j);
  // The gripper reports no velocity: it is estimated from consecutive positions. read() can
  // take several queued frames at once, so the interval is never shorter than their period.
  double velocity = 0;
  if (d.at("WRIST_MOVING").get<bool>() && status0_at_[i] && std::isfinite(s.position))
    velocity = (position - s.position) /
        std::max(std::chrono::duration<double>(now - *status0_at_[i]).count(), gripper_status_period_s_);
  s.position = position;
  s.velocity = velocity;
  s.spark_reset = d.at("NOT_REFERENCED").get<bool>();
  s.heartbeat_present = d.at("HEARTBEAT_PRESENT").get<bool>();
  status0_at_[i] = now;
  if (config_.gripper && config_.gripper->device_id == j.device_id) {
    const auto &g = *config_.gripper;
    const bool active = d.at("SERVO_ACTIVE").get<bool>();
    const double angle = d.at("SERVO_ANGLE").get<double>();
    gripper_opening_ = active ? (angle - g.closed_deg) / (g.open_deg - g.closed_deg)
                              : std::numeric_limits<double>::quiet_NaN();
    gripper_servo_angle_ = active ? std::lround(angle) : -1;
  }
}
void MoveMasterDriver::receive(const CANPacket &packet) {
  if (!packet.is_extended_id || packet.is_remote_frame) return;
  for (std::size_t i = 0; i < protocols_.size(); ++i) {
    const auto &ids = status_ids_[i];
    if (!protocols_[i]) {
      if (packet.arbitration_id == ids.status0) return receive_gripper(i, packet);
      continue;
    }
    const auto &p = *protocols_[i];
    const auto &j = config_.joints[i];
    if (packet.arbitration_id == ids.status2) {
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
    if (packet.arbitration_id == ids.status0) {
      const auto d = p.frames["STATUS_0"].decode_payload(packet.data);
      states_[i].current = d.at("CURRENT").get<double>();
      states_[i].primary_heartbeat_lock = d.at("PRIMARY_HEARTBEAT_LOCK").get<bool>();
      status0_at_[i] = Clock::now();
      return;
    }
    if (packet.arbitration_id == ids.status1) {
      const auto d = p.frames["STATUS_1"].decode_payload(packet.data);
      states_[i].spark_reset = d.at("HAS_RESET_STICKY_WARNING").get<bool>();
      status1_at_[i] = Clock::now();
      return;
    }
  }
}
bool MoveMasterDriver::feedback_fresh() const {
  const auto now = Clock::now();
  for (std::size_t i = 0; i < states_.size(); ++i)
    for (const auto *at : {&status0_at_[i], &status2_at_[i]})
      if (protocols_[i] || at == &status0_at_[i])
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
void MoveMasterDriver::wait_status1(const std::vector<std::size_t> &joints, bool cleared) {
  const auto since = Clock::now();
  const auto deadline = since + std::chrono::duration<double>(
      std::max(config_.response_timeout_s, 3 * kStatus1PeriodMs / 1000.0));
  // The first joint without its flag frame received since the call (and, if cleared, still set).
  const auto missing = [&]() -> std::optional<std::size_t> {
    for (const auto i : joints) {
      const auto &at = protocols_[i] ? status1_at_[i] : status0_at_[i];
      if (!at || *at < since || (cleared && states_[i].spark_reset)) return i;
    }
    return std::nullopt;
  };
  for (auto joint = missing(); joint; joint = missing()) {
    if (Clock::now() >= deadline) {
      const auto id = std::to_string(config_.joints[*joint].device_id);
      if (!protocols_[*joint])
        throw TimeoutError(cleared ? "SET_WRIST_POSITION did not clear NOT_REFERENCED on the gripper, device " + id
                                   : "STATUS_0 required from the gripper ESP32; none from device " + id);
      throw TimeoutError((cleared ? "CLEAR_FAULTS did not clear HAS_RESET on CAN "
                                  : "STATUS_1 required from every SPARK; none from CAN ") + id);
    }
    const auto rx = bus_->recv(0.005);
    if (rx) receive(*rx);
  }
}
void MoveMasterDriver::check_cycle_gap() const {
  if (active_ && last_tx_ && std::chrono::duration<double>(Clock::now() - *last_tx_).count() > config_.max_cycle_gap_s)
    throw std::runtime_error(tx_full_since_ ? tx_full_error_ + " for more than max_cycle_gap_s"
                                            : "Control loop gap exceeded; refusing automatic re-enable");
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
double MoveMasterDriver::checked_wrist_target(double radians, const JointConfig &j) const {
  if (!std::isfinite(radians) || radians < j.min_position_rad || radians > j.max_position_rad)
    throw std::out_of_range(j.name + ": position command outside calibrated limits");
  const double quantized =
      SignalCodec::_u32_to_float(SignalCodec::_float_to_u32(radians_to_wrist_degrees(radians, j)));
  const double actual = wrist_degrees_to_radians(quantized, j);
  if (actual < j.min_position_rad || actual > j.max_position_rad)
    throw std::out_of_range(j.name + ": float32 target outside calibrated limits");
  return quantized;
}
void MoveMasterDriver::activate() {
  ensure_healthy();
  if (!configured_ || active_) throw std::logic_error("Configure once before activation");
  if (report_only_) throw std::logic_error("Configured with SparkResetPolicy::kReport: it never enables the motors");
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
    tx_full_since_.reset();
    servo_sent_.reset();  // the first command of the activation always goes out
    active_ = true;
    write(hold);  // All setpoints are loaded before the first global heartbeat.
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::deactivate() noexcept {
  active_ = false;  // Firmware watchdog handles disable; there is no invented stop frame.
}
void MoveMasterDriver::set_control(std::size_t joint, ControlMode mode, int slot) {
  require_spark(joint);
  const auto &j = config_.joints[joint];
  const auto found = j.slots.find(slot);
  require(found != j.slots.end(), j.name + ": slot " + std::to_string(slot) + " is not configured");
  require(mode == ControlMode::kPosition || !found->second.maxmotion.is_null(),
      j.name + ": slot " + std::to_string(slot) + " has no MAXMotion profile");
  controls_[joint] = {mode, slot};
}
void MoveMasterDriver::set_current_limit(std::size_t joint, int amps) {
  ensure_healthy();
  require_spark(joint);
  require(amps >= 1 && amps <= 80, "Current limit must be in 1..80 A");
  if (!configured_ || active_) throw std::logic_error("The current limit changes only while configured and not active");
  try {
    const SparkSetup spark(*bus_, *protocols_[joint], config_.response_timeout_s);
    spark.write(rev::kSmartCurrentStallLimit, amps);
    spark.write(rev::kSmartCurrentFreeLimit, amps);
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::shift_reference(std::size_t joint, double offset_rad) {
  ensure_healthy();
  require(joint < config_.joints.size(), "Unknown joint index");
  require(std::isfinite(offset_rad), "The reference offset must be finite");
  if (!configured_ || active_ || report_only_)
    throw std::logic_error("The reference changes only while configured (not with kReport) and not active");
  try {
    if (last_tx_)
      std::this_thread::sleep_until(*last_tx_ + std::chrono::duration<double>(config_.disable_settle_s));
    wait_feedback();  // the position where the joint came to rest
    const auto &j = config_.joints[joint];
    const double target = states_[joint].position + offset_rad;
    if (protocols_[joint]) {
      const auto &frames = protocols_[joint]->frames;
      bus_->send(frames["SET_PRIMARY_ENCODER_POSITION"].packet(j.device_id,
          {{"POSITION", radians_to_rotations(target, j)}, {"DATA_TYPE", 3}}), config_.response_timeout_s);
      bus_->send(frames["CLEAR_FAULTS"].packet(j.device_id), config_.response_timeout_s);
    } else {
      const double degrees = radians_to_wrist_degrees(target, j);
      if (!(std::abs(degrees) <= kGripperMaxDegrees))
        throw std::out_of_range(j.name + ": the gripper firmware takes a reference within 360 degrees of its zero");
      bus_->send((*gripper_frames_)["SET_WRIST_POSITION"].packet(j.device_id, {{"POSITION", degrees}}),
          config_.response_timeout_s);
    }
    wait_reference(joint, target);
    referenced_[joint] = true;
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::wait_reference(std::size_t i, double target) {
  // The joint is disabled: it may only settle a little while the new reference arrives.
  constexpr double kToleranceRad = 0.01;
  const auto since = Clock::now();
  const auto deadline = since + std::chrono::duration<double>(
      std::max(config_.response_timeout_s, 3 * kStatus1PeriodMs / 1000.0));
  const auto done = [&] {
    const auto &position_at = protocols_[i] ? status2_at_[i] : status0_at_[i];
    const auto &flag_at = protocols_[i] ? status1_at_[i] : status0_at_[i];
    return position_at && *position_at >= since && flag_at && *flag_at >= since && !states_[i].spark_reset &&
        std::abs(states_[i].position - target) <= kToleranceRad;
  };
  while (!done()) {
    if (Clock::now() >= deadline) {
      const auto id = std::to_string(config_.joints[i].device_id);
      throw TimeoutError(protocols_[i]
          ? "SET_PRIMARY_ENCODER_POSITION and CLEAR_FAULTS did not take effect on CAN " + id
          : "SET_WRIST_POSITION did not take effect on the gripper, device " + id + " (the wrist must be stopped)");
    }
    const auto rx = bus_->recv(0.005);
    if (rx) receive(*rx);
  }
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
    // A restart is a fault only on a referenced joint; the others keep spark_reset in the states.
    for (std::size_t i = 0; i < states_.size(); ++i)
      if (states_[i].spark_reset && referenced_[i])
        throw std::runtime_error(config_.joints[i].name + (protocols_[i]
            ? ": the SPARK restarted (HAS_RESET); its position and RAM configuration are lost"
            : ": the gripper ESP32 restarted (NOT_REFERENCED); its position is lost"));
    if (active_ && !feedback_fresh()) throw std::runtime_error("STATUS_0/2 watchdog expired");
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
void MoveMasterDriver::write(const std::vector<double> &positions, double gripper_opening) {
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
      if (!protocols_[i]) {
        // The gripper profiles the move itself, like MAXMotion; it ignores a setpoint until its
        // heartbeat arrives, so the hold sent at activation starts nothing.
        packets.push_back((*gripper_frames_)["WRIST_POSITION_SETPOINT"].packet(j.device_id,
            {{"SETPOINT", checked_wrist_target(positions[i], j)}}));
        continue;
      }
      const auto &control = controls_[i];
      const double rotations = checked_target(positions[i], j);
      if (control.mode == ControlMode::kPosition &&
          !(std::abs(positions[i] - states_[i].position) <= j.max_following_error_rad))
        throw std::out_of_range(j.name + ": Position target exceeds max_following_error_rad from the measured position");
      packets.push_back(protocols_[i]->setpoint_packet(control.mode, rotations, control.slot));
    }
    std::optional<long> servo_angle;  // the servo angle this cycle sends, if any
    if (!std::isnan(gripper_opening)) {
      require(config_.gripper.has_value(), "No gripper configured in joints.json");
      if (!(gripper_opening >= 0 && gripper_opening <= 1))
        throw std::out_of_range("gripper: opening command must be in 0 (closed) .. 1 (open)");
      const auto &g = *config_.gripper;
      const long angle = std::lround(g.closed_deg + gripper_opening * (g.open_deg - g.closed_deg));
      // The ESP32 keeps its angle, so it is sent only when it changes, not every cycle. It goes
      // again if STATUS_0 has not reported it kServoRetryS later: the ESP32 ignores it without
      // its heartbeat, as in the first cycle after activation, like the wrist setpoint.
      if (!servo_sent_ || angle != *servo_sent_ || (angle != gripper_servo_angle_ &&
          std::chrono::duration<double>(Clock::now() - *servo_sent_at_).count() > kServoRetryS)) {
        packets.push_back((*gripper_frames_)["SERVO_SETPOINT"].packet(g.device_id, {{"ANGLE", angle}}));
        servo_angle = angle;
      }
    }
    // A caller paced at period_s (the controller_manager loop) wakes up a little early or late
    // every cycle; requiring a full period would drop about a third of the cycles.
    if (last_tx_ && std::chrono::duration<double>(Clock::now() - *last_tx_).count() < 0.5 * config_.period_s) return;
    try {
      for (const auto &packet : packets) bus_->send(packet, 0);
      if (has_spark_) bus_->send(heartbeat_, 0);
      for (const auto &packet : gripper_heartbeats_) bus_->send(packet, 0);
    } catch (const TxQueueFull &e) {
      // Frames from earlier cycles are still in the adapter: a burst, or an SPI adapter (MCP2515)
      // that cannot keep up. The rest of this cycle is dropped; the SPARKs and the ESP32 keep
      // their last setpoint, and the next cycle sends everything again. A queue that stays full
      // (no node acknowledges frames) leaves no complete cycle, so the heartbeats stop too: it
      // trips here, or in check_cycle_gap(), once max_cycle_gap_s passes without one.
      ++tx_queue_full_cycles_;
      const auto now = Clock::now();
      if (!tx_full_since_) tx_full_since_ = now;
      tx_full_error_ = e.what();
      if (std::chrono::duration<double>(now - (last_tx_ ? *last_tx_ : *tx_full_since_)).count() > config_.max_cycle_gap_s)
        throw std::runtime_error(tx_full_error_ + " for more than max_cycle_gap_s");
      return;
    }
    tx_full_since_.reset();
    last_tx_ = Clock::now();
    if (servo_angle) {
      servo_sent_ = servo_angle;
      servo_sent_at_ = last_tx_;
    }
  } catch (const std::exception &e) { trip(e.what()); throw; }
}
}  // namespace movemaster
