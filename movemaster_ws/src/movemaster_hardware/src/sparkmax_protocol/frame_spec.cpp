#include "sparkmax_protocol/frame_spec.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace sparkmax
{

namespace
{

std::string lower(std::string text)
{
  for (auto & c : text) {c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));}
  return text;
}

}  // namespace

// -----------------------------------------------------------------------------
// FrameSpec
// -----------------------------------------------------------------------------

FrameSpec::FrameSpec(std::string key, std::string section, nlohmann::ordered_json spec)
: key(std::move(key)),
  section(std::move(section)),
  spec_(std::move(spec)),
  base_arb_id_(spec_.at("arbId").get<std::uint32_t>()),
  length_bytes_(spec_.value("lengthBytes", std::size_t{0})),
  rtr_(spec_.value("rtr", false))
{
  if (length_bytes_ > 8) {
    throw SpecError("Frame " + this->key + ": lengthBytes > 8 no es CAN clásico");
  }
  if (spec_.contains("signals")) {
    for (const auto & [name, signal] : spec_["signals"].items()) {
      signals_.push_back(SignalSpec::from_json(name, signal));
    }
  }
}

const nlohmann::ordered_json & FrameSpec::operator[](const std::string & field) const
{
  return spec_.at(field);
}

const SignalSpec & FrameSpec::signal(const std::string & name) const
{
  for (const auto & s : signals_) {
    if (s.key == name) {return s;}
  }
  throw std::out_of_range("Frame " + key + " has no signal '" + name + "'");
}

std::uint32_t FrameSpec::arbitration_id(int device_id) const
{
  if (device_id < 0 || device_id > 63) {
    throw std::invalid_argument("SPARK CAN device ID must be between 0 and 63");
  }
  // The JSON's arbId is defined with Device Number = 0.
  return (base_arb_id_ & ~std::uint32_t{0x3F}) | static_cast<std::uint32_t>(device_id);
}

Bytes FrameSpec::encode_payload(const SignalValues & values, bool require_all) const
{
  if (rtr_) {
    return {};
  }

  const unsigned frame_bits = static_cast<unsigned>(length_bytes_ * 8);
  std::uint64_t raw_frame = 0;

  for (const auto & signal_spec : signals_) {
    double decoded_value = 0.0;
    if (auto it = values.find(signal_spec.key); it != values.end()) {
      decoded_value = it->second;
    } else if (require_all) {
      throw std::out_of_range("Missing signal '" + signal_spec.key + "' for frame " + key);
    }
    // else: Reserved/config bits in REV commands are generally zero unless
    // explicitly supplied. This makes sparse dictionaries convenient.

    const std::uint64_t value =
      SignalCodec::encode_bits(signal_spec, decoded_value) & bit_mask(signal_spec.length_bits);
    if (value == 0) {continue;}

    const unsigned pos = signal_spec.bit_position;
    const bool overflow = pos >= frame_bits ||
      (frame_bits - pos < 64 && (value >> (frame_bits - pos)) != 0);
    if (overflow) {
      throw SpecError(
              "Encoded frame " + key + " does not fit in " + std::to_string(frame_bits) + " bits");
    }
    raw_frame |= value << pos;
  }

  Bytes data(length_bytes_);
  for (std::size_t i = 0; i < length_bytes_; ++i) {
    data[i] = static_cast<std::uint8_t>(raw_frame >> (8 * i));  // little-endian
  }
  return data;
}

SignalValues FrameSpec::decode_payload(const Bytes & data) const
{
  if (data.size() != length_bytes_) {
    throw std::invalid_argument(
            "Frame " + key + " expects " + std::to_string(length_bytes_) + " bytes, got " +
            std::to_string(data.size()));
  }

  std::uint64_t raw_frame = 0;
  for (std::size_t i = 0; i < data.size(); ++i) {
    raw_frame |= static_cast<std::uint64_t>(data[i]) << (8 * i);
  }

  SignalValues decoded;
  for (const auto & signal_spec : signals_) {
    const unsigned pos = signal_spec.bit_position;
    const std::uint64_t raw_bits =
      pos >= 64 ? 0 : (raw_frame >> pos) & bit_mask(signal_spec.length_bits);
    decoded[signal_spec.key] = SignalCodec::decode_bits(signal_spec, raw_bits);
  }
  return decoded;
}

CANPacket FrameSpec::packet(int device_id, const SignalValues & values) const
{
  CANPacket packet;
  packet.arbitration_id = arbitration_id(device_id);
  packet.data = encode_payload(values);
  packet.is_extended_id = true;
  packet.is_remote_frame = rtr_;
  packet.dlc = static_cast<std::uint8_t>(length_bytes_);
  packet.frame_name = key;
  return packet;
}

// -----------------------------------------------------------------------------
// SparkFrameDatabase
// -----------------------------------------------------------------------------

SparkFrameDatabase::SparkFrameDatabase(const std::string & json_path)
: path(json_path)
{
  std::ifstream fh(path);
  if (!fh) {
    throw std::runtime_error("No se pudo abrir el JSON de tramas: " + path);
  }
  raw = nlohmann::ordered_json::parse(fh);

  for (const char * section : {"periodicFrames", "nonPeriodicFrames"}) {
    if (!raw.contains(section)) {continue;}
    for (const auto & [key, spec] : raw[section].items()) {
      if (frames.count(key)) {
        throw SpecError("Duplicate frame key: " + key);
      }
      frames.emplace(key, FrameSpec(key, section, spec));
    }
  }
}

const FrameSpec & SparkFrameDatabase::operator[](const std::string & key) const
{
  auto it = frames.find(key);
  if (it == frames.end()) {
    throw std::out_of_range("Unknown frame: " + key);
  }
  return it->second;
}

std::string SparkFrameDatabase::frames_version() const
{
  return raw.contains("framesVersion") ? raw["framesVersion"].get<std::string>() : "unknown";
}

nlohmann::ordered_json SparkFrameDatabase::device_info() const
{
  return raw.value("deviceInfo", nlohmann::ordered_json::object());
}

std::map<std::string, const FrameSpec *> SparkFrameDatabase::find(const std::string & text) const
{
  const auto needle = lower(text);
  const auto has = [&](const std::string & haystack) {
      return lower(haystack).find(needle) != std::string::npos;
    };
  std::map<std::string, const FrameSpec *> result;
  for (const auto & [key, frame] : frames) {
    const auto & spec = frame.raw();
    if (has(key) || has(spec.value("name", "")) || has(spec.value("description", ""))) {
      result.emplace(key, &frame);
    }
  }
  return result;
}

}  // namespace sparkmax
