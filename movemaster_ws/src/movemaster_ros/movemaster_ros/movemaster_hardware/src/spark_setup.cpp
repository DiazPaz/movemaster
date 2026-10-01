#include "movemaster_hardware/spark_setup.hpp"
#include <algorithm>
#include <chrono>

namespace movemaster {
namespace rev {
const ParameterDefinition kMotorType{"baseline", "motor_type", 0, 2, "uint", "Motor Type", std::nullopt};
const ParameterDefinition kIdleMode{"baseline", "idle_mode", 0, 6, "uint", "Idle Mode", std::nullopt};
const ParameterDefinition kFeedbackSensor{"setup", "feedback_sensor", 0, 9, "uint", "Closed Loop Control Sensor",
    std::nullopt};
const ParameterDefinition kSmartCurrentStallLimit{"baseline", "smart_current_stall_limit", 0, 59, "uint",
    "Smart Current Stall Limit", "A"};
const ParameterDefinition kSmartCurrentFreeLimit{"baseline", "smart_current_free_limit", 0, 60, "uint",
    "Smart Current Free Limit", "A"};
const ParameterDefinition kPositionConversionFactor{"setup", "position_factor", 0, 112, "float",
    "Position Conversion Factor", std::nullopt};
const ParameterDefinition kVelocityConversionFactor{"setup", "velocity_factor", 0, 113, "float",
    "Velocity Conversion Factor", std::nullopt};
const ParameterDefinition kPositionWrapping{"setup", "position_wrapping", 0, 149, "bool", "Position PID Wrap Enable",
    std::nullopt};
// The table says microseconds, but its defaults (10, 20) are the JSON's defaultPeriodMs: ms.
const ParameterDefinition kStatus0Period{"setup", "status0_period_ms", 0, 158, "uint", "Status 0 Period", "ms"};
const ParameterDefinition kStatus2Period{"setup", "status2_period_ms", 0, 160, "uint", "Status 2 Period", "ms"};
// Each slot holds 8 parameters from ID 13: P, I, D, F, IZone, D Filter, Output Min, Output Max.
ParameterDefinition output_min(int slot) {
  return {"slot", "output_min", slot, 19 + 8 * slot, "float", "Output Min " + std::to_string(slot), "duty cycle"};
}
ParameterDefinition output_max(int slot) {
  return {"slot", "output_max", slot, 20 + 8 * slot, "float", "Output Max " + std::to_string(slot), "duty cycle"};
}
}  // namespace rev

namespace {
using Clock = std::chrono::steady_clock;
double seconds_left(Clock::time_point deadline) {
  return std::max(0.0, std::chrono::duration<double>(deadline - Clock::now()).count());
}
Clock::time_point after(double seconds) {
  return Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}
// The validated Python backend keeps waiting while PERSIST_PARAMETERS answers 255.
constexpr int kFlashPending = 255;
}  // namespace

SparkSetup::SparkSetup(CANBus &bus, const SparkMAXMotionProtocol &spark, double timeout_s, double flash_timeout_s)
    : bus_(bus), spark_(spark), timeout_s_(timeout_s), flash_timeout_s_(flash_timeout_s) {}

Json SparkSetup::exchange(const std::string &request, const std::string &response, const Json &values) const {
  bus_.send(spark_.frames[request].packet(spark_.device_id, values), timeout_s_);
  const auto &expected = spark_.frames[response];
  const auto deadline = after(timeout_s_);
  while (Clock::now() < deadline) {
    const auto rx = bus_.recv(std::min(0.01, seconds_left(deadline)));
    if (rx && rx->is_extended_id && !rx->is_remote_frame && rx->arbitration_id == expected.arbitration_id(spark_.device_id))
      return expected.decode_payload(rx->data);
  }
  throw TimeoutError("Timeout waiting for " + response + " on CAN " + std::to_string(spark_.device_id));
}
void SparkSetup::write(const ParameterDefinition &parameter, const Json &value) const {
  const auto response = spark_.write_parameter(bus_, parameter, value, timeout_s_);
  if (!response.at("success").get<bool>() ||
      response.at("parameter_type_code").get<int>() != PARAMETER_TYPE_CODE.at(parameter.value_type) ||
      SparkMAXMotionProtocol::pack_parameter_value(response.at("current_value"), parameter.value_type) !=
          SparkMAXMotionProtocol::pack_parameter_value(value, parameter.value_type))
    throw std::runtime_error("Parameter ACK mismatch: CAN " + std::to_string(spark_.device_id) + ", " + parameter.key);
}
void SparkSetup::write_baseline(const SparkBaseline &baseline) const {
  write(rev::kMotorType, rev::kBrushless);
  write(rev::kIdleMode, static_cast<int>(baseline.idle_mode));
  write(rev::kSmartCurrentStallLimit, baseline.current_limit_a);
  write(rev::kSmartCurrentFreeLimit, baseline.current_limit_a);
}
void SparkSetup::write_slot(int slot, const SlotConfig &config) const {
  for (const auto &item : config.pidf.items()) write(spark_["pidf"][slot][item.key()], item.value());
  write(rev::output_min(slot), config.output_min);
  write(rev::output_max(slot), config.output_max);
  if (!config.maxmotion.is_null())
    for (const auto &item : config.maxmotion.items()) write(spark_["maxmotion"][slot][item.key()], item.value());
}
void SparkSetup::flash_command(const std::string &request, const std::string &response) const {
  const auto &frame = spark_.frames[request];
  bus_.send(frame.packet(spark_.device_id, {{"MAGIC_NUMBER", frame.signals().at("MAGIC_NUMBER").at("decodedMin")}}),
      timeout_s_);
  const auto &expected = spark_.frames[response];
  const auto deadline = after(flash_timeout_s_);
  while (Clock::now() < deadline) {
    const auto rx = bus_.recv(std::min(0.01, seconds_left(deadline)));
    if (!rx || !rx->is_extended_id || rx->is_remote_frame || rx->arbitration_id != expected.arbitration_id(spark_.device_id))
      continue;
    const int result = expected.decode_payload(rx->data).at("RESULT_CODE").get<int>();
    if (result == 0) return;
    if (result != kFlashPending)
      throw std::runtime_error(request + " rejected on CAN " + std::to_string(spark_.device_id) +
          ": RESULT_CODE " + std::to_string(result));
  }
  throw TimeoutError("Timeout waiting for " + response + " on CAN " + std::to_string(spark_.device_id));
}
void SparkSetup::reset_safe_parameters() const {
  flash_command("RESET_SAFE_PARAMETERS", "RESET_SAFE_PARAMETERS_RESPONSE");
}
void SparkSetup::persist_parameters() const {
  flash_command("PERSIST_PARAMETERS", "PERSIST_PARAMETERS_RESPONSE");
}
void SparkSetup::commission(const SparkBaseline &baseline) const {
  reset_safe_parameters();
  write_baseline(baseline);
  persist_parameters();
}

BusSurvey survey_bus(CANBus &bus, const SparkFrameDatabase &frames, double listen_s) {
  BusSurvey survey;
  const auto status0 = frames["STATUS_0"].base_arb_id() & ~0x3FU;
  const auto deadline = after(listen_s);
  while (Clock::now() < deadline) {
    const auto rx = bus.recv(std::min(0.01, seconds_left(deadline)));
    if (!rx || !rx->is_extended_id || rx->is_remote_frame) continue;
    if (rx->arbitration_id == kEnableHeartbeatId) survey.enable_heartbeat = true;
    else if ((rx->arbitration_id & ~0x3FU) == status0) survey.sparks.insert(static_cast<int>(rx->arbitration_id & 0x3FU));
  }
  return survey;
}
}  // namespace movemaster
