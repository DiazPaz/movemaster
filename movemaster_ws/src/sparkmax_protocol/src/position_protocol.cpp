#include "sparkmax_protocol/position_protocol.hpp"

namespace sparkmax_protocol
{

PositionProtocol::PositionProtocol(
  SparkFrameDatabase::Ptr frames, uint8_t device_id, ParameterLayout layout, int slot_count)
: SparkMaxProtocol(std::move(frames), device_id, std::move(layout), slot_count),
  setpoint_(bindSetpointFrame(kSetpointFrame))
{
  if (!hasGroup("pidf")) {
    throw std::invalid_argument("PositionProtocol requiere el grupo de parámetros 'pidf'");
  }
}

CANPacket PositionProtocol::positionSetpointPacket(
  double motor_rotations, int slot, double arbitrary_feedforward, FeedforwardUnits units) const
{
  return buildSetpoint(setpoint_, motor_rotations, slot, arbitrary_feedforward, units);
}

PositionProtocol::Setpoint PositionProtocol::decodePositionSetpoint(const CANPacket & packet) const
{
  const DecodedFrame decoded = checkIncoming(kSetpointFrame, packet).decodePayload(packet);
  Setpoint setpoint;
  setpoint.motor_rotations = decoded.value("SETPOINT");
  setpoint.arbitrary_feedforward = decoded.value("ARBITRARY_FEEDFORWARD");
  setpoint.slot = static_cast<int>(decoded.raw("PID_SLOT"));
  setpoint.units = decoded.raw("ARBITRARY_FEEDFORWARD_UNITS") != 0 ?
    FeedforwardUnits::DutyCycle : FeedforwardUnits::Voltage;
  return setpoint;
}

}  // namespace sparkmax_protocol
