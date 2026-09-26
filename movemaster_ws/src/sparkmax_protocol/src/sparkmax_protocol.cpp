#include "sparkmax_protocol/sparkmax_protocol.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace sparkmax_protocol
{

const char * parameterResultToString(uint8_t result_code)
{
  switch (result_code) {
    case 0: return "Success";
    case 1: return "Invalid ID";
    case 2: return "Mismatched Type";
    case 3: return "Access Mode";
    case 4: return "Invalid";
    case 5: return "Not Implemented";
    default: return "Unknown";
  }
}

// -----------------------------------------------------------------------------
// SparkMaxProtocol
// -----------------------------------------------------------------------------

SparkMaxProtocol::SparkMaxProtocol(
  SparkFrameDatabase::Ptr frames, uint8_t device_id, ParameterLayout layout, int slot_count)
: frames_(std::move(frames)), device_id_(device_id), slot_count_(slot_count),
  layout_(std::move(layout))
{
  if (!frames_) {throw std::invalid_argument("SparkMaxProtocol: base de tramas nula");}
  if (device_id_ > FrameSpec::kMaxDeviceId) {
    throw std::out_of_range("device_id debe estar entre 0 y 63");
  }
  if (slot_count_ <= 0 || slot_count_ > 4) {
    // PID_SLOT es de 2 bits en el JSON.
    throw std::out_of_range("slot_count debe estar entre 1 y 4");
  }
  // Tramas requeridas por cualquier variante. Falla pronto si el JSON no es compatible.
  requireFrame("PARAMETER_WRITE");
  requireFrame("PARAMETER_WRITE_RESPONSE");

  for (const auto & group_layout : layout_) {
    if (groups_.count(group_layout.name) != 0) {
      throw std::invalid_argument("Grupo de parámetros duplicado: " + group_layout.name);
    }
    groups_.emplace(group_layout.name, buildGroupsBySlot(group_layout, slot_count_));
  }
}

void SparkMaxProtocol::requireFrame(std::string_view name) const
{
  if (!frames_->contains(name)) {
    throw SpecError("El JSON cargado (" + frames_->framesVersion() +
            ") no contiene la trama requerida '" + std::string(name) + "'");
  }
}

void SparkMaxProtocol::checkSlot(int slot) const
{
  if (slot < 0 || slot >= slot_count_) {
    throw std::out_of_range("slot debe estar en 0.." + std::to_string(slot_count_ - 1));
  }
}

bool SparkMaxProtocol::hasGroup(std::string_view name) const
{
  return groups_.find(name) != groups_.end();
}

const ParameterGroup & SparkMaxProtocol::group(std::string_view name, int slot) const
{
  const auto it = groups_.find(name);
  if (it == groups_.end()) {
    throw std::out_of_range("Grupo de parámetros desconocido: '" + std::string(name) + "'");
  }
  checkSlot(slot);
  return it->second[static_cast<std::size_t>(slot)];
}

const FrameSpec & SparkMaxProtocol::checkIncoming(
  std::string_view frame_name, const CANPacket & packet) const
{
  const FrameSpec & spec = frames_->at(frame_name);
  if (!spec.matches(packet.arbitration_id)) {
    throw std::invalid_argument("La trama recibida no es " + std::string(frame_name));
  }
  if (packet.deviceId() != device_id_) {
    throw std::invalid_argument(std::string(frame_name) + " pertenece a otro dispositivo");
  }
  return spec;
}

// ---- parámetros -------------------------------------------------------------

CANPacket SparkMaxProtocol::parameterWritePacket(
  const ParameterDefinition & parameter, double value) const
{
  return parameterWriteRawPacket(
    parameter.parameter_id, packParameterValue(value, parameter.value_type));
}

CANPacket SparkMaxProtocol::parameterWriteRawPacket(uint8_t parameter_id, uint32_t raw_value) const
{
  const FrameSpec & frame = frames_->at("PARAMETER_WRITE");
  // VALUE es uint en el JSON porque el tipo real depende del parámetro:
  // se envían los 32 bits crudos.
  return frame.packet(device_id_, {
      {"PARAMETER_ID", SignalValue(parameter_id)},
      {"VALUE", SignalValue::raw(raw_value)},
    });
}

CANPacket SparkMaxProtocol::parameterReadPacket(const ParameterDefinition & parameter) const
{
  const std::string name = parameter.readFrameName();
  if (!frames_->contains(name)) {
    throw SpecError("El JSON no tiene trama de lectura para el parámetro " +
            std::to_string(parameter.parameter_id) + ": " + name);
  }
  return frames_->at(name).packet(device_id_);
}

ParameterWriteResponse SparkMaxProtocol::decodeParameterWriteResponse(
  const CANPacket & packet) const
{
  const DecodedFrame decoded =
    checkIncoming("PARAMETER_WRITE_RESPONSE", packet).decodePayload(packet);
  ParameterWriteResponse response;
  response.parameter_id = static_cast<uint8_t>(decoded.raw("PARAMETER_ID"));
  response.parameter_type_code = static_cast<uint8_t>(decoded.raw("PARAMETER_TYPE"));
  response.parameter_type = parameterTypeFromCode(response.parameter_type_code);
  response.raw_value = static_cast<uint32_t>(decoded.raw("VALUE"));
  response.current_value = response.parameter_type == ParameterType::Unused ?
    static_cast<double>(response.raw_value) :
    unpackParameterValue(response.raw_value, response.parameter_type);
  response.result_code = static_cast<uint8_t>(decoded.raw("RESULT_CODE"));
  response.success = response.result_code == 0;
  return response;
}

double SparkMaxProtocol::decodeParameterReadResponse(
  const ParameterDefinition & parameter, const CANPacket & packet) const
{
  const DecodedFrame decoded =
    checkIncoming(parameter.readFrameName(), packet).decodePayload(packet);
  const char * field = parameter.pairIndex() == 0 ?
    "FIRST_PARAMETER_VALUE" : "SECOND_PARAMETER_VALUE";
  return unpackParameterValue(static_cast<uint32_t>(decoded.raw(field)), parameter.value_type);
}

// ---- comandos de dispositivo ---------------------------------------------------

CANPacket SparkMaxProtocol::stopFollowerModePacket() const
{
  return frames_->at("STOP_FOLLOWER_MODE").packet(device_id_);
}

CANPacket SparkMaxProtocol::setStatusesEnabledPacket(uint16_t mask, uint16_t enabled_bitfield) const
{
  return frames_->at("SET_STATUSES_ENABLED").packet(device_id_, {
      {"MASK", SignalValue(mask)},
      {"ENABLED_BITFIELD", SignalValue(enabled_bitfield)},
    });
}

SetStatusesEnabledResponse SparkMaxProtocol::decodeSetStatusesEnabledResponse(
  const CANPacket & packet) const
{
  const DecodedFrame decoded =
    checkIncoming("SET_STATUSES_ENABLED_RESPONSE", packet).decodePayload(packet);
  SetStatusesEnabledResponse response;
  response.result_code = static_cast<uint8_t>(decoded.raw("RESULT_CODE"));
  response.specified_mask = static_cast<uint16_t>(decoded.raw("SPECIFIED_MASK"));
  response.enabled_bitfield = static_cast<uint16_t>(decoded.raw("ENABLED_BITFIELD"));
  response.success = response.result_code == 0;
  return response;
}

CANPacket SparkMaxProtocol::persistParametersPacket() const
{
  // MAGIC_NUMBER es constante en el JSON (15011); FrameSpec lo rellena solo.
  return frames_->at("PERSIST_PARAMETERS").packet(device_id_);
}

uint8_t SparkMaxProtocol::decodePersistParametersResponse(const CANPacket & packet) const
{
  const DecodedFrame decoded =
    checkIncoming("PERSIST_PARAMETERS_RESPONSE", packet).decodePayload(packet);
  return static_cast<uint8_t>(decoded.raw("RESULT_CODE"));
}

CANPacket SparkMaxProtocol::clearFaultsPacket() const
{
  return frames_->at("CLEAR_FAULTS").packet(device_id_);
}

CANPacket SparkMaxProtocol::setPrimaryEncoderPositionPacket(double rotations) const
{
  if (!std::isfinite(rotations)) {
    throw std::invalid_argument("La posición del encoder debe ser finita");
  }
  // DATA_TYPE (= 3, float) es constante en el JSON y se rellena solo.
  return frames_->at("SET_PRIMARY_ENCODER_POSITION").packet(device_id_, {
      {"POSITION", SignalValue(rotations)},
    });
}

// ---- telemetría -------------------------------------------------------------------

namespace
{
bool flag(uint64_t raw, const FrameSpec & frame, std::string_view name)
{
  const SignalSpec * signal = frame.findSignal(name);
  return signal != nullptr && SignalCodec::extract(raw, *signal).raw != 0;
}

double number(uint64_t raw, const FrameSpec & frame, std::string_view name)
{
  return SignalCodec::extract(raw, frame.signal(name)).value;
}

uint8_t bitfield(uint64_t raw, const FrameSpec & frame, const char * const (&names)[8])
{
  uint8_t bits = 0;
  for (int i = 0; i < 8; ++i) {
    if (flag(raw, frame, names[i])) {bits = static_cast<uint8_t>(bits | (1u << i));}
  }
  return bits;
}

uint64_t checkedRaw(const FrameSpec & frame, const CANPacket & packet)
{
  if (packet.dlc != frame.lengthBytes()) {
    throw std::invalid_argument("La trama " + frame.key() + " espera " +
            std::to_string(frame.lengthBytes()) + " bytes");
  }
  return FrameSpec::payloadToRaw(packet.data.data(), packet.dlc);
}
}  // namespace

Status0 SparkMaxProtocol::decodeStatus0(const CANPacket & packet) const
{
  const FrameSpec & frame = checkIncoming("STATUS_0", packet);
  const uint64_t raw = checkedRaw(frame, packet);
  Status0 status;
  status.applied_output = number(raw, frame, "APPLIED_OUTPUT");
  status.voltage = number(raw, frame, "VOLTAGE");
  status.current = number(raw, frame, "CURRENT");
  status.motor_temperature = number(raw, frame, "MOTOR_TEMPERATURE");
  status.hard_forward_limit = flag(raw, frame, "HARD_FORWARD_LIMIT_REACHED");
  status.hard_reverse_limit = flag(raw, frame, "HARD_REVERSE_LIMIT_REACHED");
  status.soft_forward_limit = flag(raw, frame, "SOFT_FORWARD_LIMIT_REACHED");
  status.soft_reverse_limit = flag(raw, frame, "SOFT_REVERSE_LIMIT_REACHED");
  status.inverted = flag(raw, frame, "INVERTED");
  status.primary_heartbeat_lock = flag(raw, frame, "PRIMARY_HEARTBEAT_LOCK");
  if (const SignalSpec * model = frame.findSignal("SPARK_MODEL")) {
    status.spark_model = static_cast<uint8_t>(SignalCodec::extract(raw, *model).raw);
  }
  return status;
}

Status1 SparkMaxProtocol::decodeStatus1(const CANPacket & packet) const
{
  static const char * const kFaults[8] = {
    "OTHER_FAULT", "MOTOR_TYPE_FAULT", "SENSOR_FAULT", "CAN_FAULT", "TEMPERATURE_FAULT",
    "DRV_FAULT", "ESC_EEPROM_FAULT", "FIRMWARE_FAULT"};
  static const char * const kWarnings[8] = {
    "BROWNOUT_WARNING", "OVERCURRENT_WARNING", "ESC_EEPROM_WARNING", "EXT_EEPROM_WARNING",
    "SENSOR_WARNING", "STALL_WARNING", "HAS_RESET_WARNING", "OTHER_WARNING"};
  static const char * const kStickyFaults[8] = {
    "OTHER_STICKY_FAULT", "MOTOR_TYPE_STICKY_FAULT", "SENSOR_STICKY_FAULT", "CAN_STICKY_FAULT",
    "TEMPERATURE_STICKY_FAULT", "DRV_STICKY_FAULT", "ESC_EEPROM_STICKY_FAULT",
    "FIRMWARE_STICKY_FAULT"};
  static const char * const kStickyWarnings[8] = {
    "BROWNOUT_STICKY_WARNING", "OVERCURRENT_STICKY_WARNING", "ESC_EEPROM_STICKY_WARNING",
    "EXT_EEPROM_STICKY_WARNING", "SENSOR_STICKY_WARNING", "STALL_STICKY_WARNING",
    "HAS_RESET_STICKY_WARNING", "OTHER_STICKY_WARNING"};

  const FrameSpec & frame = checkIncoming("STATUS_1", packet);
  const uint64_t raw = checkedRaw(frame, packet);
  Status1 status;
  status.active_faults = bitfield(raw, frame, kFaults);
  status.active_warnings = bitfield(raw, frame, kWarnings);
  status.sticky_faults = bitfield(raw, frame, kStickyFaults);
  status.sticky_warnings = bitfield(raw, frame, kStickyWarnings);
  status.is_follower = flag(raw, frame, "IS_FOLLOWER");
  return status;
}

Status2 SparkMaxProtocol::decodeStatus2(const CANPacket & packet) const
{
  const FrameSpec & frame = checkIncoming("STATUS_2", packet);
  const uint64_t raw = checkedRaw(frame, packet);
  Status2 status;
  // El JSON coloca la velocidad en los primeros 4 bytes y la posición en los siguientes.
  status.velocity = number(raw, frame, "PRIMARY_ENCODER_VELOCITY");
  status.position = number(raw, frame, "PRIMARY_ENCODER_POSITION");
  return status;
}

// ---- setpoints (usado por las clases derivadas) ------------------------------------

SparkMaxProtocol::SetpointFrame SparkMaxProtocol::bindSetpointFrame(
  std::string_view frame_name) const
{
  requireFrame(frame_name);
  SetpointFrame binding;
  binding.frame = &frames_->at(frame_name);
  binding.setpoint = &binding.frame->signal("SETPOINT");
  binding.arbitrary_feedforward = &binding.frame->signal("ARBITRARY_FEEDFORWARD");
  binding.pid_slot = &binding.frame->signal("PID_SLOT");
  binding.feedforward_units = &binding.frame->signal("ARBITRARY_FEEDFORWARD_UNITS");
  binding.reserved = binding.frame->findSignal("RESERVED");
  if (binding.setpoint->type != SignalType::Float || binding.setpoint->length_bits != 32) {
    throw SpecError(std::string(frame_name) + ".SETPOINT debe ser float de 32 bits");
  }
  return binding;
}

CANPacket SparkMaxProtocol::buildSetpoint(
  const SetpointFrame & binding, double setpoint, int slot, double arbitrary_feedforward,
  FeedforwardUnits units) const
{
  if (!std::isfinite(setpoint) ||
    std::fabs(setpoint) > static_cast<double>(std::numeric_limits<float>::max()))
  {
    throw std::invalid_argument("El setpoint debe ser finito y representable como float32");
  }
  if (!std::isfinite(arbitrary_feedforward)) {
    throw std::invalid_argument("El feedforward arbitrario debe ser finito");
  }
  checkSlot(slot);
  if (units != FeedforwardUnits::Voltage && units != FeedforwardUnits::DutyCycle) {
    throw std::invalid_argument("Unidades de feedforward: 0 (V) o 1 (duty cycle)");
  }
  uint64_t raw = 0;
  raw = SignalCodec::insert(raw, *binding.setpoint, SignalValue(setpoint));
  raw = SignalCodec::insert(raw, *binding.arbitrary_feedforward, SignalValue(arbitrary_feedforward));
  raw = SignalCodec::insert(raw, *binding.pid_slot, SignalValue(slot));
  raw = SignalCodec::insert(
    raw, *binding.feedforward_units, SignalValue(static_cast<int>(units)));
  if (binding.reserved != nullptr) {
    raw = SignalCodec::insert(raw, *binding.reserved, SignalValue(0));
  }
  return binding.frame->packetFromRaw(device_id_, raw);
}

// ---- introspección ---------------------------------------------------------------

std::string SparkMaxProtocol::describe() const
{
  std::ostringstream out;
  out << "SparkMaxProtocol [" << controlType() << "]\n";
  out << "  JSON: " << frames_->source() << " (framesVersion " << frames_->framesVersion() <<
    ", " << frames_->size() << " tramas)\n";
  out << "  device_id: " << static_cast<int>(device_id_) << ", slots: " << slot_count_ << "\n";
  for (const auto & entry : groups_) {
    for (int slot = 0; slot < slot_count_; ++slot) {
      out << "  " << entry.first << "[" << slot << "]:";
      for (const auto & p : entry.second[static_cast<std::size_t>(slot)]) {
        out << " " << p.key << "=" << static_cast<unsigned>(p.parameter_id) << "(" <<
          toString(p.value_type) << ")";
      }
      out << "\n";
    }
  }
  return out.str();
}

}  // namespace sparkmax_protocol
