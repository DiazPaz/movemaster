// sparkmax_protocol.hpp
//
// SparkMaxProtocol: fachada de protocolo para UN SPARK MAX (un ID CAN).
//
// Reúne lo que es común a cualquier tipo de control:
//   * base de tramas cargada del JSON de REV (SparkFrameDatabase)
//   * catálogo de parámetros por slot (ParameterGroup)
//   * PARAMETER_WRITE / lectura RTR de parámetros y sus respuestas
//   * comandos de dispositivo (STOP_FOLLOWER_MODE, SET_STATUSES_ENABLED,
//     PERSIST_PARAMETERS, CLEAR_FAULTS, SET_PRIMARY_ENCODER_POSITION)
//   * decodificación de STATUS_0 / STATUS_1 / STATUS_2
//
// Las clases derivadas exponen UN tipo de control:
//   * MAXMotionProtocol → MAXMOTION_POSITION_SETPOINT (port de SparkMAXMotionProtocol)
//   * PositionProtocol  → POSITION_SETPOINT (ver position_protocol.hpp)
//
// Esta capa sólo construye/interpreta CANPacket: no posee el bus, ni hilos, ni
// el heartbeat. El transporte está en socketcan.hpp y la política en MoveMasterDriver.

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "sparkmax_protocol/can_packet.hpp"
#include "sparkmax_protocol/frame_spec.hpp"
#include "sparkmax_protocol/parameter.hpp"

namespace sparkmax_protocol
{

enum class FeedforwardUnits : uint8_t { Voltage = 0, DutyCycle = 1 };

/// Respuesta a PARAMETER_WRITE.
struct ParameterWriteResponse
{
  uint8_t parameter_id{0};
  uint8_t parameter_type_code{0};
  ParameterType parameter_type{ParameterType::Unused};
  uint32_t raw_value{0};
  double current_value{0.0};
  uint8_t result_code{0};
  bool success{false};
};

/// Texto de RESULT_CODE de PARAMETER_WRITE_RESPONSE (según el JSON 2.1.0).
const char * parameterResultToString(uint8_t result_code);

struct SetStatusesEnabledResponse
{
  uint8_t result_code{0};
  uint16_t specified_mask{0};
  uint16_t enabled_bitfield{0};
  bool success{false};
};

/// STATUS_0: salida aplicada, tensión, corriente, temperatura, límites, heartbeat.
struct Status0
{
  double applied_output{0.0};   // -1..1
  double voltage{0.0};          // V
  double current{0.0};          // A
  double motor_temperature{0.0};  // °C
  bool hard_forward_limit{false};
  bool hard_reverse_limit{false};
  bool soft_forward_limit{false};
  bool soft_reverse_limit{false};
  bool inverted{false};
  bool primary_heartbeat_lock{false};
  uint8_t spark_model{0};
};

/// STATUS_1: fallas y advertencias como campos de bits (bit i = i-ésima señal).
struct Status1
{
  uint8_t active_faults{0};     // OTHER, MOTOR_TYPE, SENSOR, CAN, TEMPERATURE, DRV, ESC_EEPROM, FIRMWARE
  uint8_t active_warnings{0};   // BROWNOUT, OVERCURRENT, ESC_EEPROM, EXT_EEPROM, SENSOR, STALL, HAS_RESET, OTHER
  uint8_t sticky_faults{0};
  uint8_t sticky_warnings{0};
  bool is_follower{false};
};

/// STATUS_2: encoder primario.
struct Status2
{
  double velocity{0.0};   // RPM (con factor de velocidad = 1)
  double position{0.0};   // rotaciones (con factor de posición = 1)
};

class SparkMaxProtocol
{
public:
  static constexpr int kDefaultSlotCount = 4;

  SparkMaxProtocol(SparkFrameDatabase::Ptr frames, uint8_t device_id, ParameterLayout layout,
    int slot_count = kDefaultSlotCount);
  virtual ~SparkMaxProtocol() = default;

  SparkMaxProtocol(const SparkMaxProtocol &) = delete;
  SparkMaxProtocol & operator=(const SparkMaxProtocol &) = delete;

  /// Nombre legible del tipo de control que expone la clase.
  virtual const char * controlType() const {return "generic";}

  uint8_t deviceId() const {return device_id_;}
  int slotCount() const {return slot_count_;}
  const SparkFrameDatabase & frames() const {return *frames_;}
  const SparkFrameDatabase::Ptr & frameDatabase() const {return frames_;}
  const FrameSpec & frame(std::string_view name) const {return frames_->at(name);}
  /// ID de arbitraje de una trama para ESTE dispositivo.
  uint32_t arbitrationId(std::string_view frame_name) const
  {
    return frames_->at(frame_name).arbitrationId(device_id_);
  }

  // ---- catálogo de parámetros -----------------------------------------------
  const ParameterLayout & parameterLayout() const {return layout_;}
  bool hasGroup(std::string_view name) const;
  /// Grupo de parámetros de un slot, p.ej. group("pidf", 0)["p"].
  const ParameterGroup & group(std::string_view name, int slot = 0) const;
  const ParameterGroup & pidf(int slot = 0) const {return group("pidf", slot);}

  // ---- parámetros -------------------------------------------------------------
  CANPacket parameterWritePacket(const ParameterDefinition & parameter, double value) const;
  CANPacket parameterWriteRawPacket(uint8_t parameter_id, uint32_t raw_value) const;
  CANPacket parameterReadPacket(const ParameterDefinition & parameter) const;
  ParameterWriteResponse decodeParameterWriteResponse(const CANPacket & packet) const;
  double decodeParameterReadResponse(
    const ParameterDefinition & parameter, const CANPacket & packet) const;

  // ---- comandos de dispositivo -------------------------------------------------
  CANPacket stopFollowerModePacket() const;
  CANPacket setStatusesEnabledPacket(uint16_t mask, uint16_t enabled_bitfield) const;
  SetStatusesEnabledResponse decodeSetStatusesEnabledResponse(const CANPacket & packet) const;
  CANPacket persistParametersPacket() const;
  /// RESULT_CODE: 0 = éxito (255 = en curso, según el ejemplo validado).
  uint8_t decodePersistParametersResponse(const CANPacket & packet) const;
  CANPacket clearFaultsPacket() const;
  /// Reescribe la posición del encoder primario (rotaciones). Útil para homing.
  CANPacket setPrimaryEncoderPositionPacket(double rotations) const;

  // ---- telemetría ---------------------------------------------------------------
  Status0 decodeStatus0(const CANPacket & packet) const;
  Status1 decodeStatus1(const CANPacket & packet) const;
  Status2 decodeStatus2(const CANPacket & packet) const;

  /// Resumen legible (versión del JSON, IDs, parámetros por slot).
  std::string describe() const;

protected:
  /// Señales de una trama de setpoint (POSITION_SETPOINT, MAXMOTION_..., etc.).
  struct SetpointFrame
  {
    const FrameSpec * frame{nullptr};
    const SignalSpec * setpoint{nullptr};
    const SignalSpec * arbitrary_feedforward{nullptr};
    const SignalSpec * pid_slot{nullptr};
    const SignalSpec * feedforward_units{nullptr};
    const SignalSpec * reserved{nullptr};
  };

  /// Resuelve y valida las señales de una trama de setpoint del JSON.
  SetpointFrame bindSetpointFrame(std::string_view frame_name) const;

  /// Construye un setpoint sin asignar memoria dinámica (apto para el lazo RT).
  CANPacket buildSetpoint(const SetpointFrame & binding, double setpoint, int slot,
    double arbitrary_feedforward, FeedforwardUnits units) const;

  void requireFrame(std::string_view name) const;
  void checkSlot(int slot) const;
  const FrameSpec & checkIncoming(std::string_view frame_name, const CANPacket & packet) const;

private:
  SparkFrameDatabase::Ptr frames_;
  uint8_t device_id_{0};
  int slot_count_{kDefaultSlotCount};
  ParameterLayout layout_;
  std::map<std::string, std::vector<ParameterGroup>, std::less<>> groups_;
};

/// Port directo de SparkMAXMotionProtocol (sparkmax_json_protocol.py):
/// sólo MAXMotion Position Control; catálogo pidf + maxmotion.
class MAXMotionProtocol : public SparkMaxProtocol
{
public:
  MAXMotionProtocol(SparkFrameDatabase::Ptr frames, uint8_t device_id,
    ParameterLayout layout = parameter_layouts::maxMotionDefault(),
    int slot_count = kDefaultSlotCount);

  const char * controlType() const override {return "MAXMotion Position Control";}

  const ParameterGroup & maxmotion(int slot = 0) const {return group("maxmotion", slot);}

  /// MAXMOTION_POSITION_SETPOINT (rotaciones del motor por defecto).
  CANPacket maxmotionSetpointPacket(double setpoint, int slot = 0,
    double arbitrary_feedforward = 0.0,
    FeedforwardUnits units = FeedforwardUnits::Voltage) const;

private:
  SetpointFrame setpoint_;
};

}  // namespace sparkmax_protocol
