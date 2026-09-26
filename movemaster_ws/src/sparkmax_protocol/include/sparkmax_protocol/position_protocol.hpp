// position_protocol.hpp
//
// PositionProtocol: variante de SparkMaxProtocol que expone ÚNICAMENTE el tipo
// de control "Position Control" del REV SPARK MAX (trama POSITION_SETPOINT del
// JSON spark-frames-2.1.0).
//
// Estructura de POSITION_SETPOINT (arbId 0x02050100 + ID de dispositivo, 8 bytes):
//
//   bits  0..31  SETPOINT                     float  rotaciones (factor de posición = 1)
//   bits 32..47  ARBITRARY_FEEDFORWARD        int16  escala 0.0009765923 (±32 V)
//   bits 48..49  PID_SLOT                     uint2  0..3
//   bit  50      ARBITRARY_FEEDFORWARD_UNITS  uint1  0 = Voltage, 1 = Duty Cycle
//   bits 51..63  RESERVED                     uint13 0
//
// Enviar POSITION_SETPOINT selecciona a la vez el ControlType = Position en el
// SPARK: no hace falta otra trama. El PID corre en el SPARK con las ganancias del
// slot indicado (grupo "pidf": P, I, D, F, i_zone, d_filter, output_min/max).
// Las posiciones y señales se obtienen del JSON; no se duplican aquí.

#pragma once

#include "sparkmax_protocol/sparkmax_protocol.hpp"

namespace sparkmax_protocol
{

class PositionProtocol : public SparkMaxProtocol
{
public:
  static constexpr const char * kSetpointFrame = "POSITION_SETPOINT";

  PositionProtocol(SparkFrameDatabase::Ptr frames, uint8_t device_id,
    ParameterLayout layout = parameter_layouts::positionDefault(),
    int slot_count = kDefaultSlotCount);

  const char * controlType() const override {return "Position Control";}

  /// Trama del JSON usada para el setpoint.
  const FrameSpec & setpointFrame() const {return *setpoint_.frame;}

  /// POSITION_SETPOINT con la posición objetivo en rotaciones del MOTOR.
  /// No asigna memoria dinámica: se puede llamar desde el lazo de control.
  /// Lanza std::invalid_argument / std::out_of_range si algún valor es inválido.
  CANPacket positionSetpointPacket(double motor_rotations, int slot = 0,
    double arbitrary_feedforward = 0.0,
    FeedforwardUnits units = FeedforwardUnits::Voltage) const;

  /// Decodifica un POSITION_SETPOINT (útil para pruebas y diagnóstico).
  struct Setpoint
  {
    double motor_rotations{0.0};
    double arbitrary_feedforward{0.0};
    int slot{0};
    FeedforwardUnits units{FeedforwardUnits::Voltage};
  };
  Setpoint decodePositionSetpoint(const CANPacket & packet) const;

private:
  SetpointFrame setpoint_;
};

}  // namespace sparkmax_protocol
