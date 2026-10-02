#pragma once
#include "movemaster_hardware/sparkmax_json_protocol.hpp"
#include <optional>
#include <set>

namespace movemaster {
// Exact global heartbeat from teach_pendant_backend.py, not present in the JSON.
inline constexpr std::uint32_t kEnableHeartbeatId = 0x01011840U;

// SPARK parameters that MoveMaster writes besides DEFAULT_PARAMETER_LAYOUT (PIDF and MAXMotion,
// shared with the Python module). IDs, types and enum values follow spec/SparkParameters-v0.1.2.md;
// each description is the parameter's name there, and setup_test checks them against the table.
namespace rev {
inline constexpr int kBrushless = 1;    // MotorType.BRUSHLESS
inline constexpr int kMainEncoder = 1;  // Sensor.MAIN_ENCODER
extern const ParameterDefinition kMotorType;                 // MotorType
extern const ParameterDefinition kIdleMode;                  // IdleMode
extern const ParameterDefinition kFeedbackSensor;            // Sensor
extern const ParameterDefinition kSmartCurrentStallLimit;    // A
extern const ParameterDefinition kSmartCurrentFreeLimit;     // A
extern const ParameterDefinition kPositionConversionFactor;  // joint rad per motor rotation
extern const ParameterDefinition kVelocityConversionFactor;  // joint rad/s per motor RPM
extern const ParameterDefinition kPositionWrapping;
extern const ParameterDefinition kStatus0Period;             // ms
extern const ParameterDefinition kStatus2Period;             // ms
ParameterDefinition output_min(int slot);                    // duty cycle, -1..0
ParameterDefinition output_max(int slot);                    // duty cycle, 0..1
}  // namespace rev

enum class IdleMode { kCoast = 0, kBrake = 1 };  // IdleMode.COAST, IdleMode.BRAKE

// What stays in the SPARK's flash, besides its CAN ID. spark_commission persists it and every
// configure() writes it again to RAM. The motor type is always brushless: MoveMaster uses NEOs.
struct SparkBaseline {
  IdleMode idle_mode = IdleMode::kBrake;
  int current_limit_a = 0;  // smart current limit, same at stall and at free speed
};

// MAXMotion profile of a slot, in joint units: the driver sets the SPARK's conversion factors
// from gear_ratio, so the SPARK measures the joint in radians.
struct MAXMotionProfile {
  double cruise_velocity_rad_s = 0;
  double max_acceleration_rad_s2 = 0;
  double allowed_profile_error_rad = 0;
};

// One closed-loop slot. Position and MAXMotion Position use its PIDF and output range;
// MAXMotion adds the profile, so a slot without one only allows Position.
struct SlotConfig {
  Json pidf = Json::object();                  // p, i, d, f per joint radian
  double output_min = -1.0, output_max = 1.0;  // duty cycle
  std::optional<MAXMotionProfile> maxmotion;
};

// Request/response exchanges with one SPARK. Only for setup: they wait for the reply, so the
// caller must own bus.recv() exclusively, as configure() and spark_commission do.
class SparkSetup {
 public:
  SparkSetup(CANBus &bus, const SparkMAXMotionProtocol &spark, double timeout_s,
      double flash_timeout_s = 2.5);
  Json exchange(const std::string &request, const std::string &response,
      const Json &values = Json::object()) const;
  // Fails unless the ACK reports success with the expected type and exactly the written value.
  void write(const ParameterDefinition &parameter, const Json &value) const;
  void write_baseline(const SparkBaseline &baseline) const;
  void write_slot(int slot, const SlotConfig &config) const;
  // Every writable parameter back to its REV default except CAN ID, motor type, idle mode,
  // PWM input deadband and duty cycle offset (spark-frames 2.1.0).
  void reset_safe_parameters() const;
  // Copies every parameter in RAM to flash.
  void persist_parameters() const;
  // Leaves in flash the REV defaults plus CAN ID and baseline, whatever RAM held before.
  void commission(const SparkBaseline &baseline) const;
 private:
  CANBus &bus_;
  const SparkMAXMotionProtocol &spark_;
  double timeout_s_, flash_timeout_s_;
  void flash_command(const std::string &request, const std::string &response) const;
};

struct BusSurvey {
  std::set<int> sparks;           // CAN IDs that sent STATUS_0
  bool enable_heartbeat = false;  // another program is enabling the motors
};
// Listens without transmitting. The bus filters must pass STATUS_0 and kEnableHeartbeatId.
BusSurvey survey_bus(CANBus &bus, const SparkFrameDatabase &frames, double listen_s);
}  // namespace movemaster
