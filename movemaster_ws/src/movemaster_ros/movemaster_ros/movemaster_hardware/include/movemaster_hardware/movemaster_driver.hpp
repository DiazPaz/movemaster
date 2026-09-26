#pragma once
#include "movemaster_hardware/sparkmax_json_protocol.hpp"
#include <chrono>
#include <limits>
#include <memory>

namespace movemaster {
struct JointConfig {
  std::string name;
  int device_id = -1;
  int slot = 0;
  double gear_ratio = std::numeric_limits<double>::quiet_NaN();  // motor turns / joint turn
  int direction = 1;                                        // +1 or -1
  double zero_offset_rad = std::numeric_limits<double>::quiet_NaN();
  double min_position_rad = std::numeric_limits<double>::quiet_NaN();
  double max_position_rad = std::numeric_limits<double>::quiet_NaN();
  Json pidf = Json::object();
  Json maxmotion = Json::object();  // RPM, RPM/s, motor rotations
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
DriverConfig load_driver_config(const std::filesystem::path &config_path,
    const std::filesystem::path &spec_path, const std::string &channel = "can0",
    const std::vector<std::string> &joint_order = {});

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
  const std::vector<JointState> &states() const { return states_; }
  bool active() const { return active_; }
  const std::string &fault() const { return fault_; }
  static double radians_to_rotations(double radians, const JointConfig &joint);
  static double rotations_to_radians(double rotations, const JointConfig &joint);
  static double rpm_to_rad_s(double rpm, const JointConfig &joint);
 private:
  using Clock = std::chrono::steady_clock;
  DriverConfig config_;
  std::unique_ptr<CANBus> bus_;
  std::vector<std::unique_ptr<SparkMAXMotionProtocol>> protocols_;
  std::vector<JointState> states_;
  std::vector<std::optional<Clock::time_point>> status0_at_, status2_at_;
  std::optional<Clock::time_point> last_tx_;
  bool configured_ = false, active_ = false;
  std::string fault_;
  // Exact global heartbeat from teach_pendant_backend.py, not present in the JSON.
  const CANPacket heartbeat_{0x01011840U, Bytes(8, 0xFF), true, false, 8, "REFERENCE_HEARTBEAT"};
  void validate_config() const;
  void ensure_healthy() const;
  void trip(const std::string &message) noexcept;
  void write_checked(SparkMAXMotionProtocol &protocol, const ParameterDefinition &parameter, const Json &value);
  Json exchange(SparkMAXMotionProtocol &protocol, const std::string &request,
      const std::string &response, const Json &values = Json::object());
  void receive(const CANPacket &packet);
  bool feedback_fresh() const;
  void wait_feedback();
  void check_cycle_gap() const;
  double checked_target(double radians, const JointConfig &joint) const;
};
}  // namespace movemaster
