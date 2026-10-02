#pragma once
#include "movemaster_hardware/spark_setup.hpp"
#include <chrono>
#include <limits>
#include <map>
#include <memory>

namespace movemaster {
struct JointConfig {
  std::string name;
  int device_id = -1;
  double gear_ratio = std::numeric_limits<double>::quiet_NaN();  // motor turns / joint turn, in the SPARK
  int direction = 1;                                        // +1 or -1
  double zero_offset_rad = std::numeric_limits<double>::quiet_NaN();
  double min_position_rad = std::numeric_limits<double>::quiet_NaN();
  double max_position_rad = std::numeric_limits<double>::quiet_NaN();
  double max_velocity_rad_s = std::numeric_limits<double>::quiet_NaN();  // bounds every cruise velocity
  SparkBaseline spark;
  // Control at activation; set_control() changes it at runtime.
  ControlMode mode = ControlMode::kMAXMotionPosition;
  int slot = 0;
  // Position mode has no profile in the SPARK: a target may not be farther than this from
  // the measured position.
  double max_following_error_rad = std::numeric_limits<double>::quiet_NaN();
  std::map<int, SlotConfig> slots;
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
};
struct JointState {
  double position = std::numeric_limits<double>::quiet_NaN();  // rad
  double velocity = std::numeric_limits<double>::quiet_NaN();  // rad/s
  double current = std::numeric_limits<double>::quiet_NaN();   // A, never torque
  bool primary_heartbeat_lock = false;
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

// No background thread: ros2_control is the only sender/receiver and heartbeat owner.
// Lifecycle setup may wait for ACKs. Runtime read/write never wait for RX/TX.
class MoveMasterDriver {
 public:
  explicit MoveMasterDriver(DriverConfig config, std::unique_ptr<CANBus> bus = nullptr);
  ~MoveMasterDriver() = default;
  void configure();
  void activate();
  void deactivate() noexcept;
  void read();
  void write(const std::vector<double> &positions_rad);
  // Applies from the next write(), also while active, and is kept across activations.
  void set_control(std::size_t joint, ControlMode mode, int slot);
  const std::vector<JointControl> &controls() const { return controls_; }
  const std::vector<JointState> &states() const { return states_; }
  bool active() const { return active_; }
  const std::string &fault() const { return fault_; }
  // The SPARK scales the motor to the joint with its conversion factors, set from gear_ratio;
  // the driver only applies direction and zero_offset_rad.
  static double position_factor(const JointConfig &joint);  // rad per motor rotation: 2π / G
  static double velocity_factor(const JointConfig &joint);  // rad/s per motor RPM: 2π / (60 G)
  static double joint_to_spark(double radians, const JointConfig &joint);   // d · (q − q0)
  static double spark_to_joint(double position, const JointConfig &joint);  // q0 + d · p
 private:
  using Clock = std::chrono::steady_clock;
  DriverConfig config_;
  std::unique_ptr<CANBus> bus_;
  std::vector<std::unique_ptr<SparkMAXMotionProtocol>> protocols_;
  std::vector<JointState> states_;
  std::vector<JointControl> controls_;
  std::vector<std::optional<Clock::time_point>> status0_at_, status2_at_;
  std::optional<Clock::time_point> last_tx_;
  bool configured_ = false, active_ = false;
  std::string fault_;
  const CANPacket heartbeat_{kEnableHeartbeatId, Bytes(8, 0xFF), true, false, 8, "REFERENCE_HEARTBEAT"};
  void ensure_healthy() const;
  void trip(const std::string &message) noexcept;
  void receive(const CANPacket &packet);
  bool feedback_fresh() const;
  void wait_feedback();
  void check_cycle_gap() const;
  double checked_target(double radians, const JointConfig &joint) const;
};
}  // namespace movemaster
