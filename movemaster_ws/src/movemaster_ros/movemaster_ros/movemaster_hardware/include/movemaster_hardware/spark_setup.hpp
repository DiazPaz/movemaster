#pragma once
#include "movemaster_hardware/sparkmax_json_protocol.hpp"
#include <set>

namespace movemaster {
// Exact global heartbeat from teach_pendant_backend.py, not present in the JSON.
inline constexpr std::uint32_t kEnableHeartbeatId = 0x01011840U;

// REV SparkParameter IDs that MoveMaster writes besides DEFAULT_PARAMETER_LAYOUT (PIDF and
// MAXMotion, shared with the Python module). Same REV table as the IDs the validated Python
// backend already writes: 9, 112, 113, 149, 158 and 160.
namespace rev {
extern const ParameterDefinition kMotorType;                 // 0 brushed, 1 brushless
extern const ParameterDefinition kIdleMode;                  // 0 coast, 1 brake
extern const ParameterDefinition kFeedbackSensor;            // 1 primary encoder
extern const ParameterDefinition kSmartCurrentStallLimit;    // A
extern const ParameterDefinition kSmartCurrentFreeLimit;     // A
extern const ParameterDefinition kPositionConversionFactor;  // SPARK position unit per rotation
extern const ParameterDefinition kVelocityConversionFactor;  // SPARK velocity unit per RPM
extern const ParameterDefinition kPositionWrapping;
extern const ParameterDefinition kStatus0Period;             // ms
extern const ParameterDefinition kStatus2Period;             // ms
ParameterDefinition output_min(int slot);                    // duty cycle, -1..0
ParameterDefinition output_max(int slot);                    // duty cycle, 0..1
}  // namespace rev

enum class IdleMode { kCoast = 0, kBrake = 1 };

// What stays in the SPARK's flash, besides its CAN ID. spark_commission persists it and every
// configure() writes it again to RAM. The motor type is always brushless: MoveMaster uses NEOs.
struct SparkBaseline {
  IdleMode idle_mode = IdleMode::kBrake;
  int current_limit_a = 0;  // smart current limit, same at stall and at free speed
};

// One closed-loop slot. Position and MAXMotion Position use its PIDF and output range;
// MAXMotion adds the profile, so a slot without one only allows Position.
struct SlotConfig {
  Json pidf = Json::object();                  // p, i, d, f
  double output_min = -1.0, output_max = 1.0;  // duty cycle
  Json maxmotion = nullptr;                    // RPM, RPM/s, motor rotations
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
