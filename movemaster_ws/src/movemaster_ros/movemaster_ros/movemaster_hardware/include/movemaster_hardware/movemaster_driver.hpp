#pragma once
#include "movemaster_hardware/homing.hpp"
#include "movemaster_hardware/spark_setup.hpp"
#include <chrono>
#include <limits>
#include <map>
#include <memory>

namespace movemaster {
// What moves a joint: a SPARK MAX with its NEO, or the gripper's ESP32, which drives the wrist
// stepper (spec/gripper-frames-1.0.0). The ESP32 works in wrist degrees and applies the
// reduction itself (REDUCCION in its firmware), so a gripper joint has no gear_ratio and none
// of the SPARK blocks.
enum class JointDevice { kSpark, kGripper };
// The gripper catalog is read from the same folder as the SPARK one (DriverConfig::spec_path).
inline constexpr const char *kGripperFramesFile = "gripper-frames-1.0.0";
// The gripper firmware takes positions within this many degrees of its zero (LIMITE_GRADOS).
inline constexpr double kGripperMaxDegrees = 360;

struct JointConfig {
  std::string name;
  JointDevice device = JointDevice::kSpark;
  int device_id = -1;  // SPARK CAN ID, or the gripper's device number (CAN_DISPOSITIVO)
  double gear_ratio = std::numeric_limits<double>::quiet_NaN();  // motor turns / joint turn
  int direction = 1;                                        // +1 or -1
  double zero_offset_rad = std::numeric_limits<double>::quiet_NaN();
  double min_position_rad = std::numeric_limits<double>::quiet_NaN();
  double max_position_rad = std::numeric_limits<double>::quiet_NaN();
  double max_velocity_rad_s = std::numeric_limits<double>::quiet_NaN();  // bounds every cruise velocity
  // Gripper: with max_velocity_rad_s, the wrist's motion profile (SET_MOTION_PROFILE); required.
  // SPARK: optional, the acceleration trajectories are planned with (the HMI); bounds every
  // MAXMotion max_acceleration. NaN: not given.
  double max_acceleration_rad_s2 = std::numeric_limits<double>::quiet_NaN();
  // The rest is for SPARK joints only.
  SparkBaseline spark;
  // Control at activation; set_control() changes it at runtime.
  ControlMode mode = ControlMode::kMAXMotionPosition;
  int slot = 0;
  // Position mode has no profile in the SPARK: a target may not be farther than this from
  // the measured position.
  double max_following_error_rad = std::numeric_limits<double>::quiet_NaN();
  std::map<int, SlotConfig> slots;
  std::optional<HomingConfig> homing;  // only for spark_home
};
// The gripper servo, on the ESP32 that moves the wrist. ROS commands an opening from 0 (closed)
// to 1 (open), which maps linearly onto the servo angles where the gripper closes and opens.
struct GripperConfig {
  int device_id = -1;  // the can_id of the gripper joint: the servo shares its ESP32
  double closed_deg = std::numeric_limits<double>::quiet_NaN();  // servo angle, 0..180
  double open_deg = std::numeric_limits<double>::quiet_NaN();
};
struct DriverConfig {
  std::filesystem::path spec_path;
  std::string channel = "can0";
  double period_s = 0.020;
  double feedback_timeout_s = 0.300;
  double response_timeout_s = 0.500;
  double disable_settle_s = 0.500;
  double max_cycle_gap_s = 0.100;
  int status_period_ms = 20;
  Json parameter_layout = nullptr;
  std::vector<JointConfig> joints;
  std::optional<GripperConfig> gripper;
  // spark_home --home searches the stops of these joints, in this order (homing.stop of each).
  std::vector<std::string> homing_order;
};
struct JointState {
  double position = std::numeric_limits<double>::quiet_NaN();  // rad
  double velocity = std::numeric_limits<double>::quiet_NaN();  // rad/s
  double current = std::numeric_limits<double>::quiet_NaN();   // A, never torque; none for the gripper
  bool primary_heartbeat_lock = false;
  // The device restarted since the driver last took its reference: HAS_RESET in a SPARK's
  // STATUS_1, NOT_REFERENCED in the gripper's STATUS_0.
  bool spark_reset = false;
  bool heartbeat_present = false;  // gripper STATUS_0: its HEARTBEAT arrives
};
struct JointControl {
  ControlMode mode;
  int slot;
};
DriverConfig load_driver_config(const std::filesystem::path &config_path,
    const std::filesystem::path &spec_path, const std::string &channel = "can0",
    const std::vector<std::string> &joint_order = {});
// Throws std::invalid_argument naming the first invalid field.
void validate_driver_config(const DriverConfig &config);

// A SPARK keeps its encoder count while powered. After a restart (power-up, emergency stop) it
// counts from where it powered up, which reads as zero_offset_rad, until spark_home --home writes
// it the position of a measured stop (shift_reference). The gripper's ESP32 does the same with its
// step count. The policies apply to both. A joint is referenced while its count is trusted: a
// restart of a referenced joint is a fault in read().
enum class SparkResetPolicy {
  kAcceptPowerOnPose,  // the pose at power-up is the reference: every joint is referenced
  kFault,              // a restart is a fault: the position was lost, or never homed
  // spark_home --home: a restarted joint stays unreferenced, with its flag set, until
  // shift_reference(); activate() is allowed, to search its stop.
  kHoming,
  // Only report restarts (spark_home's status): HAS_RESET stays set for whoever configures next,
  // read() leaves it in the states instead of failing, and activate() is refused.
  kReport,
};

// No background thread: ros2_control is the only sender/receiver and heartbeat owner.
// Lifecycle setup may wait for ACKs. Runtime read/write never wait for RX/TX.
class MoveMasterDriver {
 public:
  explicit MoveMasterDriver(DriverConfig config, std::unique_ptr<CANBus> bus = nullptr);
  ~MoveMasterDriver() = default;
  // Returns the joints whose SPARK restarted; with kAcceptPowerOnPose their restart is cleared,
  // so a later one shows up in read() as a fault. kFault throws instead; kHoming and kReport keep it.
  std::vector<std::string> configure(SparkResetPolicy policy = SparkResetPolicy::kAcceptPowerOnPose);
  bool referenced(std::size_t joint) const { return referenced_.at(joint); }
  // The device (SPARK encoder, gripper step count) reports from now on the joint's position plus
  // offset_rad, and the joint is referenced: its restart flag is cleared. Only while configured
  // (not kReport) and not active; it first waits disable_settle_s after the last heartbeat, so
  // that no SPARK is still enabled to chase its last setpoint in the shifted count.
  void shift_reference(std::size_t joint, double offset_rad);
  void activate();
  void deactivate() noexcept;
  void read();
  // gripper_opening: 0 (closed) to 1 (open), only with a gripper configured. NaN sends no
  // command: the servo keeps its angle, or stays without signal until its first command.
  void write(const std::vector<double> &positions_rad,
      double gripper_opening = std::numeric_limits<double>::quiet_NaN());
  // Applies from the next write(), also while active, and is kept across activations.
  void set_control(std::size_t joint, ControlMode mode, int slot);
  // Smart Current Limit (stall and free) in RAM, with ACK; only while not active.
  void set_current_limit(std::size_t joint, int amps);
  const std::vector<JointControl> &controls() const { return controls_; }
  const std::vector<JointState> &states() const { return states_; }
  // The gripper's last commanded opening, as its STATUS_0 reports it: the servo has no sensor.
  // NaN while the servo has no signal (since the ESP32 powered up) or without a gripper.
  double gripper_opening() const { return gripper_opening_; }
  bool active() const { return active_; }
  // Setup requests configure() sent again because no reply arrived (see SparkSetup).
  int setup_retries() const { return setup_retries_; }
  // Cycles write() cut short because the CAN adapter's TX queue was full: frames sent earlier
  // had not left it yet. The next cycle sends everything again.
  long tx_queue_full_cycles() const { return tx_queue_full_cycles_; }
  const std::string &fault() const { return fault_; }
  static double radians_to_rotations(double radians, const JointConfig &joint);
  static double rotations_to_radians(double rotations, const JointConfig &joint);
  static double rpm_to_rad_s(double rpm, const JointConfig &joint);
  // Gripper joints: joint radians <-> wrist degrees of the ESP32 (direction and zero_offset_rad).
  static double radians_to_wrist_degrees(double radians, const JointConfig &joint);
  static double wrist_degrees_to_radians(double degrees, const JointConfig &joint);
 private:
  using Clock = std::chrono::steady_clock;
  DriverConfig config_;
  std::unique_ptr<CANBus> bus_;
  // One per joint; null for a gripper joint, which uses gripper_frames_.
  std::vector<std::unique_ptr<SparkMAXMotionProtocol>> protocols_;
  std::unique_ptr<SparkFrameDatabase> gripper_frames_;  // only with a gripper joint
  double gripper_status_period_s_ = 0;                   // its STATUS_0 period
  std::vector<CANPacket> gripper_heartbeats_;            // one per gripper joint
  double gripper_opening_ = std::numeric_limits<double>::quiet_NaN();
  long gripper_servo_angle_ = -1;  // as STATUS_0 reports it; -1 while the servo has no signal
  // The last servo angle sent, and when: SERVO_SETPOINT goes out only when it changes.
  std::optional<long> servo_sent_;
  std::optional<Clock::time_point> servo_sent_at_;
  bool has_spark_ = false;
  std::vector<JointState> states_;
  std::vector<JointControl> controls_;
  std::vector<bool> referenced_;
  // A gripper joint has only STATUS_0: position, restart and heartbeat in one frame.
  std::vector<std::optional<Clock::time_point>> status0_at_, status1_at_, status2_at_;
  // Each joint's STATUS_0/1/2 arbitration IDs, resolved once: receive() checks every frame.
  struct StatusIds { std::uint32_t status0, status1, status2; };
  std::vector<StatusIds> status_ids_;
  std::optional<Clock::time_point> last_tx_;  // the last complete cycle, heartbeats included
  // Since when write() finds the TX queue full, with no complete cycle since, and why.
  std::optional<Clock::time_point> tx_full_since_;
  std::string tx_full_error_;
  long tx_queue_full_cycles_ = 0;
  bool configured_ = false, active_ = false, report_only_ = false;
  std::string fault_;
  int setup_retries_ = 0;
  const CANPacket heartbeat_{kEnableHeartbeatId, Bytes(8, 0xFF), true, false, 8, "REFERENCE_HEARTBEAT"};
  void ensure_healthy() const;
  void trip(const std::string &message) noexcept;
  void receive(const CANPacket &packet);
  void receive_gripper(std::size_t joint, const CANPacket &packet);
  bool feedback_fresh() const;
  void wait_feedback();
  // Waits for the frame with each joint's restart flag (a SPARK's STATUS_1, the gripper's
  // STATUS_0) received after the call; with cleared, also for the flag to be off.
  void wait_status1(const std::vector<std::size_t> &joints, bool cleared);
  // Waits for the joint to report position near target and its restart flag off.
  void wait_reference(std::size_t joint, double target);
  void check_cycle_gap() const;
  double checked_target(double radians, const JointConfig &joint) const;
  double checked_wrist_target(double radians, const JointConfig &joint) const;
  void require_spark(std::size_t joint) const;
};
}  // namespace movemaster
