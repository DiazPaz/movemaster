#include "movemaster_hardware/sparkmax_json_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace movemaster {
const Json DEFAULT_PARAMETER_LAYOUT = Json::parse(R"json({
  "pidf": {
    "p": {
      "base_id": 13,
      "slot_stride": 8,
      "type": "float",
      "description": "Proportional gain"
    },
    "i": {
      "base_id": 14,
      "slot_stride": 8,
      "type": "float",
      "description": "Integral gain"
    },
    "d": {
      "base_id": 15,
      "slot_stride": 8,
      "type": "float",
      "description": "Derivative gain"
    },
    "f": {
      "base_id": 16,
      "slot_stride": 8,
      "type": "float",
      "description": "Feedforward gain (F parameter)"
    }
  },
  "maxmotion": {
    "cruise_velocity": {
      "base_id": 166,
      "slot_stride": 5,
      "type": "float",
      "description": "MAXMotion cruise/max velocity",
      "unit": "RPM by default",
      "aliases": [
        "cruisevelocity",
        "max_velocity",
        "maxvelocity"
      ]
    },
    "max_acceleration": {
      "base_id": 167,
      "slot_stride": 5,
      "type": "float",
      "description": "MAXMotion maximum acceleration",
      "unit": "RPM/s by default",
      "aliases": [
        "maxaccel",
        "max_accel",
        "maxacceleration"
      ]
    },
    "allowed_profile_error": {
      "base_id": 169,
      "slot_stride": 5,
      "type": "float",
      "description": "MAXMotion allowed profile/closed-loop error",
      "unit": "rotations by default",
      "aliases": [
        "allowedprofileerror",
        "allowed_error",
        "allowed_closed_loop_error"
      ]
    }
  }
})json");
const std::map<std::string, int> PARAMETER_TYPE_CODE = {
    {"int", 1}, {"uint", 2}, {"float", 3}, {"bool", 4}, {"boolean", 4}};
const std::map<int, std::string> PARAMETER_TYPE_NAME = {
    {1, "int"}, {2, "uint"}, {3, "float"}, {4, "bool"}};

namespace {
std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}
double number(const Json &value) {
  if (value.is_boolean()) return value.get<bool>() ? 1.0 : 0.0;
  if (value.is_number()) return value.get<double>();
  if (value.is_string()) {
    const auto s = value.get<std::string>();
    std::size_t used = 0;
    const double result = std::stod(s, &used);
    if (s.find_first_not_of(" \t\r\n", used) != std::string::npos)
      throw std::invalid_argument("Invalid numeric string");
    return result;
  }
  throw std::invalid_argument("Expected a numeric value");
}
bool truth(const Json &value) {
  if (value.is_null()) return false;
  if (value.is_boolean()) return value.get<bool>();
  if (value.is_number()) return number(value) != 0;
  if (value.is_string()) return !value.get_ref<const std::string &>().empty();
  return !value.empty();
}
std::uint64_t mask_for(int bits) {
  if (bits < 1 || bits > 64) throw SpecError("Signal width must be in 1..64");
  return bits == 64 ? UINT64_MAX : (std::uint64_t{1} << bits) - 1;
}
// Python round(): ties to even, independent of the process floating-point mode.
double python_round(double x) {
  if (!std::isfinite(x)) throw std::invalid_argument("Cannot round a nonfinite value");
  const double lo = std::floor(x), fraction = x - lo;
  if (fraction < 0.5) return lo;
  if (fraction > 0.5) return lo + 1;
  return std::fmod(lo, 2.0) == 0 ? lo : lo + 1;
}
using Clock = std::chrono::steady_clock;
double remaining(Clock::time_point deadline) {
  return std::max(0.0, std::chrono::duration<double>(deadline - Clock::now()).count());
}
}  // namespace

can_frame CANPacket::to_socketcan() const {
  const int length = dlc.value_or(static_cast<int>(data.size()));
  if (length < 0 || length > CAN_MAX_DLEN || data.size() > CAN_MAX_DLEN)
    throw std::invalid_argument("Classic CAN DLC must be in 0..8");
  if ((!is_remote_frame && static_cast<int>(data.size()) != length) ||
      (is_remote_frame && !data.empty()))
    throw std::invalid_argument("CAN payload and DLC/RTR disagree");
  if (arbitration_id > (is_extended_id ? CAN_EFF_MASK : CAN_SFF_MASK))
    throw std::invalid_argument("CAN arbitration ID out of range");
  can_frame frame{};
  frame.can_id = arbitration_id | (is_extended_id ? CAN_EFF_FLAG : 0U) |
      (is_remote_frame ? CAN_RTR_FLAG : 0U);
  frame.can_dlc = static_cast<std::uint8_t>(length);
  std::copy(data.begin(), data.end(), frame.data);
  return frame;
}
CANPacket CANPacket::from_socketcan(const can_frame &frame) {
  if (frame.can_id & CAN_ERR_FLAG) throw std::runtime_error("CAN error frame received");
  if (frame.can_dlc > CAN_MAX_DLEN) throw std::invalid_argument("Invalid classic CAN DLC");
  CANPacket packet;
  packet.is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
  packet.is_remote_frame = (frame.can_id & CAN_RTR_FLAG) != 0;
  packet.arbitration_id = frame.can_id & (packet.is_extended_id ? CAN_EFF_MASK : CAN_SFF_MASK);
  packet.dlc = frame.can_dlc;
  if (!packet.is_remote_frame) packet.data.assign(frame.data, frame.data + frame.can_dlc);
  return packet;
}
std::string CANPacket::repr() const {
  std::ostringstream out;
  out << "CANPacket(frame=" << frame_name.value_or("None") << ", id=0x"
      << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << arbitration_id
      << ", dlc=" << std::dec << (dlc ? std::to_string(*dlc) : "None") << ", data=";
  if (data.empty()) out << "<RTR/no data>";
  for (std::size_t i = 0; i < data.size(); ++i)
    out << (i ? " " : "") << std::hex << std::setw(2) << static_cast<int>(data[i]);
  return out.str() + ")";
}

void SignalCodec::_require_little_endian(const Json &spec) {
  if (spec.value("isBigEndian", false)) throw SpecError("Big-endian signals are not supported");
}
std::uint32_t SignalCodec::_float_to_u32(double value) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
      "IEEE-754 binary32 is required");
  if (std::isfinite(value) && std::abs(value) > std::numeric_limits<float>::max())
    throw std::overflow_error("Value does not fit float32");
  const float f = static_cast<float>(value);
  std::uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}
double SignalCodec::_u32_to_float(std::uint32_t value) {
  float f;
  std::memcpy(&f, &value, sizeof(f));
  return f;
}
std::uint64_t SignalCodec::encode_bits(const Json &spec, const Json &value) {
  _require_little_endian(spec);
  const auto type = lower(spec.at("type").get<std::string>());
  const int bits = spec.at("lengthBits").get<int>();
  const auto mask = mask_for(bits);
  const double scale = spec.value("decodeScaleFactor", 1.0), offset = spec.value("offset", 0.0);
  if (type == "float") {
    if (bits != 32) throw SpecError("Unsupported float width");
    if (scale == 0) throw SpecError("decodeScaleFactor cannot be zero");
    return _float_to_u32((number(value) - offset) / scale);
  }
  double encoded;
  if (type == "bool" || type == "boolean") encoded = truth(value) ? 1 : 0;
  else {
    if (scale == 0) throw SpecError("decodeScaleFactor cannot be zero");
    encoded = python_round((number(value) - offset) / scale);
  }
  if (spec.contains("encodedMin") && !spec["encodedMin"].is_null() &&
      static_cast<long double>(encoded) < spec["encodedMin"].get<long double>())
    throw std::out_of_range("Encoded value below encodedMin");
  if (spec.contains("encodedMax") && !spec["encodedMax"].is_null() &&
      static_cast<long double>(encoded) > spec["encodedMax"].get<long double>())
    throw std::out_of_range("Encoded value above encodedMax");
  if (type == "int") {
    const double limit = std::ldexp(1.0, bits - 1);
    if (!(encoded >= -limit && encoded < limit)) throw std::out_of_range("Signed value out of range");
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(encoded)) & mask;
  }
  if (type == "uint" || type == "bool" || type == "boolean") {
    if (!(encoded >= 0 && encoded < std::ldexp(1.0, bits)))
      throw std::out_of_range("Unsigned value out of range");
    return static_cast<std::uint64_t>(encoded);
  }
  throw SpecError("Unsupported signal type: " + type);
}
Json SignalCodec::decode_bits(const Json &spec, std::uint64_t raw_bits) {
  _require_little_endian(spec);
  const auto type = lower(spec.at("type").get<std::string>());
  const int bits = spec.at("lengthBits").get<int>();
  const auto mask = mask_for(bits);
  raw_bits &= mask;
  const double scale = spec.value("decodeScaleFactor", 1.0), offset = spec.value("offset", 0.0);
  if (type == "float") {
    if (bits != 32) throw SpecError("Unsupported float width");
    return _u32_to_float(static_cast<std::uint32_t>(raw_bits)) * scale + offset;
  }
  if (type == "int") {
    // Compute the negative magnitude before converting to double (including INT64_MIN).
    const double encoded = (raw_bits & (std::uint64_t{1} << (bits - 1)))
        ? -static_cast<double>(((~raw_bits) & mask) + 1) : static_cast<double>(raw_bits);
    return encoded * scale + offset;
  }
  if (type == "uint") return static_cast<double>(raw_bits) * scale + offset;
  if (type == "bool" || type == "boolean") return raw_bits != 0;
  throw SpecError("Unsupported signal type: " + type);
}

FrameSpec::FrameSpec(std::string name, std::string group, Json spec)
    : key(std::move(name)), section(std::move(group)), _spec(std::move(spec)) {}
Json FrameSpec::get(const std::string &name, Json fallback) const {
  return _spec.contains(name) ? _spec.at(name) : fallback;
}
std::uint32_t FrameSpec::base_arb_id() const { return _spec.at("arbId").get<std::uint32_t>(); }
int FrameSpec::length_bytes() const { return _spec.value("lengthBytes", 0); }
const Json &FrameSpec::signals() const {
  static const Json empty = Json::object();
  return _spec.contains("signals") ? _spec.at("signals") : empty;
}
bool FrameSpec::rtr() const { return _spec.value("rtr", false); }
std::uint32_t FrameSpec::arbitration_id(int id) const {
  if (id < 0 || id > 63) throw std::out_of_range("SPARK CAN device ID must be in 0..63");
  return (base_arb_id() & ~0x3FU) | static_cast<std::uint32_t>(id);
}
Bytes FrameSpec::encode_payload(const Json &values, bool require_all) const {
  if (rtr()) return {};
  const int length = length_bytes();
  if (length < 0 || length > 8) throw SpecError("Only classic CAN frames are supported");
  std::uint64_t raw = 0;
  for (const auto &item : signals().items()) {
    if (require_all && !values.contains(item.key()))
      throw std::out_of_range("Missing signal " + item.key() + " for frame " + key);
    const auto &spec = item.value();
    const int pos = spec.at("bitPosition").get<int>(), bits = spec.at("lengthBits").get<int>();
    if (pos < 0 || pos + bits > length * 8) throw SpecError("Signal outside frame: " + key);
    raw |= (SignalCodec::encode_bits(spec, values.contains(item.key()) ? values.at(item.key()) : Json(0))
        & mask_for(bits)) << pos;
  }
  Bytes data(static_cast<std::size_t>(length));
  for (int i = 0; i < length; ++i) data[i] = static_cast<std::uint8_t>(raw >> (8 * i));
  return data;
}
Json FrameSpec::decode_payload(const Bytes &data) const {
  if (static_cast<int>(data.size()) != length_bytes())
    throw std::invalid_argument("Incorrect payload length for frame " + key);
  if (data.size() > 8) throw SpecError("Only classic CAN frames are supported");
  std::uint64_t raw = 0;
  for (std::size_t i = 0; i < data.size(); ++i) raw |= std::uint64_t{data[i]} << (8 * i);
  Json decoded = Json::object();
  for (const auto &item : signals().items()) {
    const int pos = item.value().at("bitPosition").get<int>();
    const int bits = item.value().at("lengthBits").get<int>();
    if (pos < 0 || pos + bits > static_cast<int>(data.size() * 8))
      throw SpecError("Signal outside frame: " + key);
    decoded[item.key()] = SignalCodec::decode_bits(item.value(), (raw >> pos) & mask_for(bits));
  }
  return decoded;
}
CANPacket FrameSpec::packet(int id, const Json &values) const {
  return {arbitration_id(id), encode_payload(values), true, rtr(), length_bytes(), key};
}
SparkFrameDatabase::SparkFrameDatabase(const std::filesystem::path &json_path) : path(json_path) {
  std::ifstream file(path);
  if (!file) throw SpecError("Cannot open JSON: " + path.string());
  file >> raw;
  for (const auto *section : {"periodicFrames", "nonPeriodicFrames"}) {
    if (!raw.contains(section)) continue;
    for (const auto &item : raw.at(section).items()) {
      if (!frames.emplace(item.key(), FrameSpec(item.key(), section, item.value())).second)
        throw SpecError("Duplicate frame key: " + item.key());
    }
  }
}
std::string SparkFrameDatabase::frames_version() const {
  if (!raw.contains("framesVersion")) return "unknown";
  const auto &v = raw.at("framesVersion");
  return v.is_string() ? v.get<std::string>() : v.dump();
}
Json SparkFrameDatabase::device_info() const { return raw.value("deviceInfo", Json::object()); }
std::map<std::string, FrameSpec> SparkFrameDatabase::find(const std::string &text) const {
  const auto needle = lower(text);
  std::map<std::string, FrameSpec> result;
  for (const auto &entry : frames) {
    if (lower(entry.first).find(needle) != std::string::npos ||
        lower(entry.second.get("name", "").get<std::string>()).find(needle) != std::string::npos ||
        lower(entry.second.get("description", "").get<std::string>()).find(needle) != std::string::npos)
      result.emplace(entry.first, entry.second);
  }
  return result;
}

std::string ParameterDefinition::read_frame_name() const {
  return "READ_PARAMETER_" + std::to_string(pair_start_id()) + "_AND_" + std::to_string(pair_start_id() + 1);
}
void to_json(Json &out, const ParameterDefinition &p) {
  out = {{"group", p.group}, {"key", p.key}, {"slot", p.slot}, {"parameter_id", p.parameter_id},
      {"value_type", p.value_type}, {"description", p.description}, {"unit", p.unit ? Json(*p.unit) : Json(nullptr)}};
}
ParameterGroup::ParameterGroup(Definitions canonical, std::map<std::string, std::string> aliases)
    : _canonical(std::move(canonical)), _aliases(std::move(aliases)) {}
std::string ParameterGroup::_normalize(std::string key) {
  const auto first = key.find_first_not_of(" \t\r\n"), last = key.find_last_not_of(" \t\r\n");
  key = first == std::string::npos ? "" : lower(key.substr(first, last - first + 1));
  std::replace(key.begin(), key.end(), '-', '_');
  std::replace(key.begin(), key.end(), ' ', '_');
  return key;
}
const ParameterDefinition &ParameterGroup::operator[](const std::string &key) const {
  const auto normalized = _normalize(key);
  const auto alias = _aliases.find(normalized);
  return _canonical.at(alias == _aliases.end() ? normalized : alias->second);
}
const ParameterGroup &SparkMAXMotionProtocol::Access::operator[](int slot) const {
  if (!slots_) throw std::invalid_argument("This entry is not a parameter group");
  return slots_->at(slot);
}
const FrameSpec &SparkMAXMotionProtocol::Access::operator[](const std::string &name) const {
  if (!frames_) throw std::invalid_argument("This entry is not the frame database");
  return (*frames_)[name];
}
const FrameSpec &SparkMAXMotionProtocol::Access::as_frame() const {
  if (!frame_) throw std::invalid_argument("This entry is not a frame");
  return *frame_;
}
SparkMAXMotionProtocol::SparkMAXMotionProtocol(const std::filesystem::path &json_path,
    int id, Json layout, int count)
    : device_id(id), slot_count(count), frames(json_path),
      parameter_layout(layout.is_null() ? DEFAULT_PARAMETER_LAYOUT : std::move(layout)) {
  if (id < 0 || id > 63) throw std::out_of_range("device_id must be in 0..63");
  if (count <= 0) throw std::invalid_argument("slot_count must be positive");
  for (const auto *required : {"MAXMOTION_POSITION_SETPOINT", "PARAMETER_WRITE", "PARAMETER_WRITE_RESPONSE"})
    if (!frames.contains(required)) throw SpecError("Missing required frame: " + std::string(required));
  _pidf = _build_group_by_slot("pidf");
  _maxmotion = _build_group_by_slot("maxmotion");
  _access.emplace("pidf", Access(&_pidf));
  _access.emplace("maxmotion", Access(&_maxmotion));
  _access.emplace("frames", Access(&frames));
  _access.emplace("setpoint", Access(&frames["MAXMOTION_POSITION_SETPOINT"]));
}
SparkMAXMotionProtocol::Slots SparkMAXMotionProtocol::_build_group_by_slot(const std::string &name) const {
  Slots result;
  for (int slot = 0; slot < slot_count; ++slot) {
    ParameterGroup::Definitions canonical;
    std::map<std::string, std::string> aliases;
    for (const auto &entry : parameter_layout.at(name).items()) {
      const auto &meta = entry.value();
      ParameterDefinition p;
      p.group = name; p.key = entry.key(); p.slot = slot;
      p.parameter_id = meta.at("base_id").get<int>() + slot * meta.at("slot_stride").get<int>();
      p.value_type = lower(meta.value("type", std::string("float")));
      p.description = meta.value("description", std::string());
      if (meta.contains("unit") && !meta["unit"].is_null()) p.unit = meta["unit"].get<std::string>();
      canonical.emplace(p.key, p);
      aliases[ParameterGroup::_normalize(p.key)] = p.key;
      for (const auto &alias : meta.value("aliases", Json::array()))
        aliases[ParameterGroup::_normalize(alias.get<std::string>())] = p.key;
    }
    result.emplace(slot, ParameterGroup(std::move(canonical), std::move(aliases)));
  }
  return result;
}
std::uint32_t SparkMAXMotionProtocol::pack_parameter_value(const Json &value, std::string type) {
  type = lower(type);
  if (type == "float") return SignalCodec::_float_to_u32(number(value));
  if (type == "bool" || type == "boolean") return truth(value) ? 1U : 0U;
  if (type == "int") {
    if (value.is_number_unsigned()) return static_cast<std::uint32_t>(value.get<std::uint64_t>());
    if (value.is_number_integer()) return static_cast<std::uint32_t>(value.get<std::int64_t>());
    const double x = std::trunc(number(value));
    if (!std::isfinite(x)) throw std::invalid_argument("Nonfinite integer parameter");
    double u = std::fmod(x, 4294967296.0);
    if (u < 0) u += 4294967296.0;
    return static_cast<std::uint32_t>(u);
  }
  if (type == "uint") {
    const double x = std::trunc(number(value));
    if (!(x >= 0 && x <= 4294967295.0)) throw std::out_of_range("uint parameter must fit 32 bits");
    return static_cast<std::uint32_t>(x);
  }
  throw std::invalid_argument("Unsupported parameter type: " + type);
}
Json SparkMAXMotionProtocol::unpack_parameter_value(std::uint32_t raw, std::string type) {
  type = lower(type);
  if (type == "float") return SignalCodec::_u32_to_float(raw);
  if (type == "int") return raw & 0x80000000U ? std::int64_t(raw) - 0x100000000LL : std::int64_t(raw);
  if (type == "uint") return raw;
  if (type == "bool" || type == "boolean") return raw != 0;
  throw std::invalid_argument("Unsupported parameter type: " + type);
}
CANPacket SparkMAXMotionProtocol::parameter_write_packet(const ParameterDefinition &p, const Json &value) const {
  return frames["PARAMETER_WRITE"].packet(device_id,
      {{"PARAMETER_ID", p.parameter_id}, {"VALUE", pack_parameter_value(value, p.value_type)}});
}
CANPacket SparkMAXMotionProtocol::parameter_read_packet(const ParameterDefinition &p) const {
  if (!frames.contains(p.read_frame_name())) throw SpecError("No read frame for parameter " + std::to_string(p.parameter_id));
  return frames[p.read_frame_name()].packet(device_id);
}
CANPacket SparkMAXMotionProtocol::maxmotion_setpoint_packet(double setpoint, int slot, double ff, int units) const {
  if (slot < 0 || slot >= slot_count) throw std::out_of_range("Invalid PID slot");
  if (units != 0 && units != 1) throw std::invalid_argument("Feedforward units must be 0 or 1");
  return frames["MAXMOTION_POSITION_SETPOINT"].packet(device_id, {{"SETPOINT", setpoint},
      {"ARBITRARY_FEEDFORWARD", ff}, {"PID_SLOT", slot}, {"ARBITRARY_FEEDFORWARD_UNITS", units}, {"RESERVED", 0}});
}
Json SparkMAXMotionProtocol::decode_parameter_write_response(const Bytes &data) const {
  const auto d = frames["PARAMETER_WRITE_RESPONSE"].decode_payload(data);
  const int code = d.at("PARAMETER_TYPE").get<int>();
  const std::string type = PARAMETER_TYPE_NAME.count(code) ? PARAMETER_TYPE_NAME.at(code) : "unknown";
  const auto raw = d.at("VALUE").get<std::uint32_t>();
  const int result = d.at("RESULT_CODE").get<int>();
  return {{"parameter_id", d.at("PARAMETER_ID").get<int>()}, {"parameter_type_code", code},
      {"parameter_type", type}, {"current_value", type == "unknown" ? Json(raw) : unpack_parameter_value(raw, type)},
      {"result_code", result}, {"success", result == 0}};
}
Json SparkMAXMotionProtocol::decode_parameter_read_response(const ParameterDefinition &p, const Bytes &data) const {
  const auto d = frames[p.read_frame_name()].decode_payload(data);
  return unpack_parameter_value(d.at(p.pair_index() == 0 ? "FIRST_PARAMETER_VALUE" : "SECOND_PARAMETER_VALUE")
      .get<std::uint32_t>(), p.value_type);
}
void SparkMAXMotionProtocol::_send_packet(CANBus &bus, const CANPacket &packet) { bus.send(packet); }
Json SparkMAXMotionProtocol::write_parameter(CANBus &bus, const ParameterDefinition &p,
    const Json &value, double timeout, bool verify) const {
  _send_packet(bus, parameter_write_packet(p, value));
  const auto expected = frames["PARAMETER_WRITE_RESPONSE"].arbitration_id(device_id);
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
  while (remaining(deadline) > 0) {
    const auto rx = bus.recv(std::min(0.05, remaining(deadline)));
    if (!rx || !rx->is_extended_id || rx->arbitration_id != expected) continue;
    auto d = decode_parameter_write_response(rx->data);
    if (d.at("parameter_id").get<int>() != p.parameter_id) continue;
    d["requested_value"] = value;
    d["parameter"] = p;
    d["value_matches"] = nullptr;
    if (verify && d.at("success").get<bool>()) {
      if (p.value_type == "float") {
        const double a = number(d.at("current_value")), b = number(value);
        d["value_matches"] = a == b || (std::isfinite(a) && std::isfinite(b) &&
            std::abs(a - b) <= std::max(1e-7, 1e-5 * std::max(std::abs(a), std::abs(b))));
      } else {
        const auto &current = d.at("current_value");
        // Python considers bool a numeric type: False == 0 and True == 1.
        d["value_matches"] = (current.is_boolean() || current.is_number()) &&
            (value.is_boolean() || value.is_number())
            ? number(current) == number(value) : current == value;
      }
    }
    return d;
  }
  throw TimeoutError("Timeout waiting for PARAMETER_WRITE_RESPONSE for parameter " + std::to_string(p.parameter_id));
}
CANPacket SparkMAXMotionProtocol::send_setpoint(CANBus &bus, double setpoint, int slot, double ff, int units) const {
  const auto packet = maxmotion_setpoint_packet(setpoint, slot, ff, units);
  _send_packet(bus, packet);
  return packet;
}
Json SparkMAXMotionProtocol::configure_slot(CANBus &bus, int slot, const Json &pidf, const Json &maxmotion, double timeout) const {
  if (slot < 0 || slot >= slot_count) throw std::out_of_range("Invalid PID slot");
  Json results = Json::object();
  for (const auto &entry : std::array<std::pair<std::string, const Json *>, 2>{{{"pidf", &pidf}, {"maxmotion", &maxmotion}}}) {
    if (entry.second->is_null() || entry.second->empty()) continue;
    const auto &group = (*this)[entry.first][slot];
    for (const auto &item : entry.second->items()) {
      const auto &p = group[item.key()];
      results[entry.first + "." + p.key] = write_parameter(bus, p, item.value(), timeout, true);
    }
  }
  return results;
}
Json SparkMAXMotionProtocol::describe() const {
  const auto groups = [](const Slots &slots) {
    Json out = Json::object();
    for (const auto &slot : slots) {
      Json params = Json::object();
      for (const auto &item : slot.second) {
        const auto &p = item.second;
        params[item.first] = {{"parameter_id", p.parameter_id}, {"type", p.value_type},
            {"unit", p.unit ? Json(*p.unit) : Json(nullptr)}, {"description", p.description}, {"read_frame", p.read_frame_name()}};
      }
      out[std::to_string(slot.first)] = params;
    }
    return out;
  };
  const auto &f = frames["MAXMOTION_POSITION_SETPOINT"];
  return {{"frames_version", frames.frames_version()}, {"device_id", device_id}, {"device_info", frames.device_info()},
      {"maxmotion_position_setpoint", {{"base_arb_id", f.base_arb_id()}, {"device_arb_id", f.arbitration_id(device_id)},
          {"length_bytes", f.length_bytes()}, {"signals", f.signals()}}}, {"pidf", groups(_pidf)}, {"maxmotion", groups(_maxmotion)}};
}
}  // namespace movemaster
