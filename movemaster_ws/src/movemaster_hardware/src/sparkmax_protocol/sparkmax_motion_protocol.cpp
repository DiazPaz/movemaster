#include "sparkmax_protocol/sparkmax_motion_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sparkmax
{

namespace
{

using Clock = std::chrono::steady_clock;

double seconds_until(Clock::time_point deadline)
{
  return std::chrono::duration<double>(deadline - Clock::now()).count();
}

bool is_close(double a, double b, double rel_tol, double abs_tol)
{
  // math.isclose()
  return std::fabs(a - b) <= std::max(rel_tol * std::max(std::fabs(a), std::fabs(b)), abs_tol);
}

std::int64_t python_int(double value)
{
  // int(x): trunca hacia cero; ValueError/OverflowError si no es finito.
  if (!std::isfinite(value) || std::fabs(value) > 9.0e18) {
    throw std::invalid_argument("Parameter value is not a finite integer");
  }
  return static_cast<std::int64_t>(std::trunc(value));
}

}  // namespace

SparkMAXMotionProtocol::SparkMAXMotionProtocol(
  const std::string & json_path,
  int device_id,
  const ParameterLayout & parameter_layout,
  int slot_count)
: device_id(device_id),
  slot_count(slot_count),
  frames(json_path),
  parameter_layout(parameter_layout)
{
  if (device_id < 0 || device_id > 63) {
    throw std::invalid_argument("device_id must be between 0 and 63");
  }
  if (slot_count <= 0) {
    throw std::invalid_argument("slot_count must be positive");
  }

  // Required frame names. Fail early if the loaded spec is incompatible.
  for (const char * required :
    {"MAXMOTION_POSITION_SETPOINT", "PARAMETER_WRITE", "PARAMETER_WRITE_RESPONSE"})
  {
    if (!frames.contains(required)) {
      throw SpecError(std::string("Loaded JSON does not contain required frame '") +
                required + "'");
    }
  }

  pidf_ = build_group_by_slot("pidf");
  maxmotion_ = build_group_by_slot("maxmotion");
}

const ParameterGroup & SparkMAXMotionProtocol::group(const std::string & name, int slot) const
{
  if (name == "pidf") {return pidf(slot);}
  if (name == "maxmotion") {return maxmotion(slot);}
  throw std::out_of_range("Unknown parameter group '" + name + "'");
}

std::map<int, ParameterGroup> SparkMAXMotionProtocol::build_group_by_slot(
  const std::string & group) const
{
  const auto & layout = parameter_layout.at(group);
  std::map<int, ParameterGroup> result;

  for (int slot = 0; slot < slot_count; ++slot) {
    std::map<std::string, ParameterDefinition> canonical;
    std::map<std::string, std::string> aliases;

    for (const auto & [key, meta] : layout) {
      std::string value_type = meta.type;
      std::transform(value_type.begin(), value_type.end(), value_type.begin(), ::tolower);
      canonical.emplace(key, ParameterDefinition{
          group, key, slot, meta.base_id + slot * meta.slot_stride,
          value_type, meta.description, meta.unit});
      aliases[ParameterGroup::normalize(key)] = key;
      for (const auto & alias : meta.aliases) {
        aliases[ParameterGroup::normalize(alias)] = key;
      }
    }
    result.emplace(slot, ParameterGroup(std::move(canonical), std::move(aliases)));
  }
  return result;
}

// ---- parameter value packing --------------------------------------------------

std::uint32_t SparkMAXMotionProtocol::pack_parameter_value(
  double value, const std::string & value_type)
{
  if (value_type == "float") {
    if (std::isfinite(value) && std::fabs(value) > std::numeric_limits<float>::max()) {
      throw std::invalid_argument("float too large to pack with f format");
    }
    return SignalCodec::float_to_u32(static_cast<float>(value));
  }
  if (value_type == "int") {
    return static_cast<std::uint32_t>(python_int(value) & 0xFFFFFFFF);
  }
  if (value_type == "uint") {
    const auto integer = python_int(value);
    if (integer < 0 || integer > 0xFFFFFFFF) {
      throw std::invalid_argument("uint parameter must fit in 32 bits");
    }
    return static_cast<std::uint32_t>(integer);
  }
  if (value_type == "bool" || value_type == "boolean") {
    return value != 0.0 ? 1U : 0U;
  }
  throw std::invalid_argument("Unsupported parameter type: '" + value_type + "'");
}

double SparkMAXMotionProtocol::unpack_parameter_value(
  std::uint32_t raw_u32, const std::string & value_type)
{
  if (value_type == "float") {
    return SignalCodec::u32_to_float(raw_u32);
  }
  if (value_type == "int") {
    return static_cast<double>(static_cast<std::int32_t>(raw_u32));
  }
  if (value_type == "uint") {
    return static_cast<double>(raw_u32);
  }
  if (value_type == "bool" || value_type == "boolean") {
    return raw_u32 ? 1.0 : 0.0;
  }
  throw std::invalid_argument("Unsupported parameter type: '" + value_type + "'");
}

// ---- frame builders -----------------------------------------------------------

CANPacket SparkMAXMotionProtocol::parameter_write_packet(
  const ParameterDefinition & parameter, double value) const
{
  const auto raw_value = pack_parameter_value(value, parameter.value_type);
  return frames["PARAMETER_WRITE"].packet(
    device_id,
    {
      {"PARAMETER_ID", parameter.parameter_id},
      // VALUE is uint in the frame JSON because the true type is
      // parameter-dependent. We intentionally supply the raw 32-bit bits.
      {"VALUE", static_cast<double>(raw_value)},
    });
}

CANPacket SparkMAXMotionProtocol::parameter_read_packet(const ParameterDefinition & parameter) const
{
  const auto frame_name = parameter.read_frame_name();
  if (!frames.contains(frame_name)) {
    throw SpecError(
            "Loaded JSON has no read frame for parameter " +
            std::to_string(parameter.parameter_id) + ": " + frame_name);
  }
  return frames[frame_name].packet(device_id);
}

CANPacket SparkMAXMotionProtocol::maxmotion_setpoint_packet(
  double setpoint,
  int slot,
  double arbitrary_feedforward,
  int arbitrary_feedforward_units) const
{
  if (slot < 0 || slot >= slot_count) {
    throw std::invalid_argument("slot must be in 0.." + std::to_string(slot_count - 1));
  }
  if (arbitrary_feedforward_units != 0 && arbitrary_feedforward_units != 1) {
    throw std::invalid_argument("arbitrary_feedforward_units must be 0 (V) or 1 (duty cycle)");
  }
  return this->setpoint().packet(
    device_id,
    {
      {"SETPOINT", setpoint},
      {"ARBITRARY_FEEDFORWARD", arbitrary_feedforward},
      {"PID_SLOT", slot},
      {"ARBITRARY_FEEDFORWARD_UNITS", arbitrary_feedforward_units},
      {"RESERVED", 0},
    });
}

// ---- decoders -----------------------------------------------------------------

ParameterWriteResult SparkMAXMotionProtocol::decode_parameter_write_response(
  const Bytes & data) const
{
  const auto decoded = frames["PARAMETER_WRITE_RESPONSE"].decode_payload(data);

  ParameterWriteResult r;
  r.parameter_id = static_cast<int>(decoded.at("PARAMETER_ID"));
  r.parameter_type_code = static_cast<int>(decoded.at("PARAMETER_TYPE"));
  r.raw_value = static_cast<std::uint32_t>(decoded.at("VALUE"));
  r.result_code = static_cast<int>(decoded.at("RESULT_CODE"));

  const auto type = PARAMETER_TYPE_NAME.find(r.parameter_type_code);
  r.parameter_type = type != PARAMETER_TYPE_NAME.end() ? type->second : "unknown";
  r.current_value = r.parameter_type != "unknown" ?
    unpack_parameter_value(r.raw_value, r.parameter_type) :
    static_cast<double>(r.raw_value);
  r.success = r.result_code == 0;
  return r;
}

double SparkMAXMotionProtocol::decode_parameter_read_response(
  const ParameterDefinition & parameter, const Bytes & data) const
{
  const auto decoded = frames[parameter.read_frame_name()].decode_payload(data);
  const char * field =
    parameter.pair_index() == 0 ? "FIRST_PARAMETER_VALUE" : "SECOND_PARAMETER_VALUE";
  return unpack_parameter_value(
    static_cast<std::uint32_t>(decoded.at(field)), parameter.value_type);
}

// ---- optional synchronous helpers ---------------------------------------------

ParameterWriteResult SparkMAXMotionProtocol::write_parameter(
  CanBus & bus,
  const ParameterDefinition & parameter,
  double value,
  double timeout,
  bool verify) const
{
  bus.send(parameter_write_packet(parameter, value));

  const auto expected_id = frames["PARAMETER_WRITE_RESPONSE"].arbitration_id(device_id);
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(timeout));

  while (Clock::now() < deadline) {
    const auto rx = bus.recv(std::min(0.05, std::max(0.0, seconds_until(deadline))));
    if (!rx || !rx->is_extended_id) {continue;}
    if (rx->arbitration_id != expected_id) {continue;}

    auto decoded = decode_parameter_write_response(rx->data);
    if (decoded.parameter_id != parameter.parameter_id) {continue;}

    decoded.requested_value = value;
    decoded.parameter = parameter;

    if (verify && decoded.success) {
      decoded.value_matches = parameter.value_type == "float" ?
        is_close(decoded.current_value, value, 1e-5, 1e-7) :
        decoded.current_value == value;
    }
    return decoded;
  }

  throw TimeoutError(
          "Timeout waiting for PARAMETER_WRITE_RESPONSE for parameter " +
          std::to_string(parameter.parameter_id));
}

CANPacket SparkMAXMotionProtocol::send_setpoint(
  CanBus & bus,
  double setpoint,
  int slot,
  double arbitrary_feedforward,
  int arbitrary_feedforward_units) const
{
  auto packet = maxmotion_setpoint_packet(
    setpoint, slot, arbitrary_feedforward, arbitrary_feedforward_units);
  bus.send(packet);
  return packet;
}

std::map<std::string, ParameterWriteResult> SparkMAXMotionProtocol::configure_slot(
  CanBus & bus,
  int slot,
  const std::map<std::string, double> & pidf,
  const std::map<std::string, double> & maxmotion,
  double timeout) const
{
  if (slot < 0 || slot >= slot_count) {
    throw std::invalid_argument("slot must be in 0.." + std::to_string(slot_count - 1));
  }

  std::map<std::string, ParameterWriteResult> results;
  for (const auto & [group_name, supplied] :
    {std::pair{"pidf", &pidf}, std::pair{"maxmotion", &maxmotion}})
  {
    const auto & params = group(group_name, slot);
    for (const auto & [key, value] : *supplied) {
      const auto & parameter = params[key];
      results[std::string(group_name) + "." + parameter.key] =
        write_parameter(bus, parameter, value, timeout, true);
    }
  }
  return results;
}

nlohmann::ordered_json SparkMAXMotionProtocol::describe() const
{
  const auto group_to_dict = [](const std::map<int, ParameterGroup> & group_by_slot) {
      auto out = nlohmann::ordered_json::object();
      for (const auto & [slot, params] : group_by_slot) {
        auto & slot_json = out[std::to_string(slot)];
        for (const auto & [key, p] : params) {
          slot_json[key] = {
            {"parameter_id", p.parameter_id},
            {"type", p.value_type},
            {"unit", p.unit ? nlohmann::ordered_json(*p.unit) : nlohmann::ordered_json()},
            {"description", p.description},
            {"read_frame", p.read_frame_name()},
          };
        }
      }
      return out;
    };

  const auto & setpoint_frame = setpoint();
  auto signals = setpoint_frame.raw().value("signals", nlohmann::ordered_json::object());
  return {
    {"frames_version", frames.frames_version()},
    {"device_id", device_id},
    {"device_info", frames.device_info()},
    {"maxmotion_position_setpoint", {
        {"base_arb_id", setpoint_frame.base_arb_id()},
        {"device_arb_id", setpoint_frame.arbitration_id(device_id)},
        {"length_bytes", setpoint_frame.length_bytes()},
        {"signals", signals},
      }},
    {"pidf", group_to_dict(pidf_)},
    {"maxmotion", group_to_dict(maxmotion_)},
  };
}

}  // namespace sparkmax
