#include "sparkmax_protocol/frame_spec.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <sstream>

namespace sparkmax_protocol
{

// -----------------------------------------------------------------------------
// CANPacket
// -----------------------------------------------------------------------------

std::string CANPacket::dataHex() const
{
  if (is_remote_frame || dlc == 0) {return "<RTR/sin datos>";}
  std::string out;
  char byte[4];
  for (std::size_t i = 0; i < dlc && i < kMaxDataLength; ++i) {
    std::snprintf(byte, sizeof(byte), i == 0 ? "%02X" : " %02X", data[i]);
    out += byte;
  }
  return out;
}

std::string CANPacket::toString() const
{
  char id[16];
  std::snprintf(id, sizeof(id), "0x%08X", arbitration_id);
  std::ostringstream out;
  out << "CANPacket(frame=" << (frame_name.empty() ? "?" : std::string(frame_name)) <<
    ", id=" << id << ", dlc=" << static_cast<int>(dlc) << ", data=" << dataHex() << ")";
  return out.str();
}

// -----------------------------------------------------------------------------
// DecodedFrame
// -----------------------------------------------------------------------------

const DecodedSignal & DecodedFrame::at(std::string_view name) const
{
  const auto it = signals_.find(name);
  if (it == signals_.end()) {
    throw SpecError("La trama decodificada no contiene la señal '" + std::string(name) + "'");
  }
  return it->second;
}

// -----------------------------------------------------------------------------
// FrameSpec
// -----------------------------------------------------------------------------

FrameSpec FrameSpec::fromJson(
  const std::string & key, const std::string & section, const json::Value & spec)
{
  FrameSpec frame;
  frame.key_ = key;
  frame.section_ = section;
  frame.name_ = spec.stringOr("name", key);
  frame.description_ = spec.stringOr("description", "");
  frame.base_arb_id_ = static_cast<uint32_t>(spec.at("arbId").asUint64());
  frame.length_bytes_ = static_cast<std::size_t>(spec.doubleOr("lengthBytes", 0));
  frame.rtr_ = spec.boolOr("rtr", false);
  frame.api_class_ = static_cast<int>(spec.doubleOr("apiClass", -1));
  frame.api_index_ = static_cast<int>(spec.doubleOr("apiIndex", -1));
  frame.default_period_ms_ = static_cast<int>(spec.doubleOr("defaultPeriodMs", -1));

  if (frame.base_arb_id_ > CANPacket::kExtendedIdMask) {
    throw SpecError("Trama " + key + ": arbId excede 29 bits");
  }
  if (frame.length_bytes_ > CANPacket::kMaxDataLength) {
    throw SpecError("Trama " + key + ": lengthBytes > 8 (CAN FD no soportado)");
  }

  if (const json::Value * signals = spec.find("signals")) {
    for (const auto & member : signals->asObject()) {
      SignalSpec signal = SignalSpec::fromJson(member.key, member.value);
      if (signal.bit_position + signal.length_bits > frame.length_bytes_ * 8) {
        throw SpecError("Trama " + key + ": la señal " + member.key + " excede lengthBytes");
      }
      frame.signals_.push_back(std::move(signal));
    }
  }
  return frame;
}

const SignalSpec * FrameSpec::findSignal(std::string_view signal_name) const
{
  for (const auto & signal : signals_) {
    if (signal.key == signal_name) {return &signal;}
  }
  return nullptr;
}

const SignalSpec & FrameSpec::signal(std::string_view signal_name) const
{
  const SignalSpec * found = findSignal(signal_name);
  if (found == nullptr) {
    throw SpecError("La trama " + key_ + " no tiene la señal '" + std::string(signal_name) + "'");
  }
  return *found;
}

uint32_t FrameSpec::arbitrationId(uint8_t device_id) const
{
  if (device_id > kMaxDeviceId) {
    throw std::out_of_range("El ID CAN del SPARK debe estar entre 0 y 63");
  }
  return (base_arb_id_ & ~0x3Fu) | device_id;
}

uint64_t FrameSpec::encodeRaw(const SignalValues & values, bool require_all) const
{
  if (rtr_) {return 0;}
  for (const auto & entry : values) {
    if (findSignal(entry.first) == nullptr) {
      throw SpecError("La trama " + key_ + " no tiene la señal '" + entry.first + "'");
    }
  }
  uint64_t raw = 0;
  for (const auto & signal : signals_) {
    const auto it = values.find(signal.key);
    if (it != values.end()) {
      raw = SignalCodec::insert(raw, signal, it->second);
    } else if (signal.isConstant()) {
      // Mejora respecto al script Python: MAGIC_NUMBER, DATA_TYPE, etc.
      raw = SignalCodec::insert(raw, signal, SignalValue(*signal.decoded_min));
    } else if (require_all) {
      throw SpecError("Falta la señal '" + signal.key + "' de la trama " + key_);
    } else {
      // Los bits reservados/configuración se envían en cero salvo que se indiquen.
      raw = SignalCodec::insert(raw, signal, SignalValue(0));
    }
  }
  return raw;
}

std::array<uint8_t, CANPacket::kMaxDataLength> FrameSpec::encodePayload(
  const SignalValues & values, bool require_all) const
{
  std::array<uint8_t, CANPacket::kMaxDataLength> data{};
  rawToPayload(encodeRaw(values, require_all), data.data(), length_bytes_);
  return data;
}

DecodedFrame FrameSpec::decodePayload(const uint8_t * data, std::size_t length) const
{
  if (length != length_bytes_) {
    std::ostringstream out;
    out << "La trama " << key_ << " espera " << length_bytes_ << " bytes, llegaron " << length;
    throw std::invalid_argument(out.str());
  }
  const uint64_t raw = payloadToRaw(data, length);
  DecodedFrame decoded;
  for (const auto & signal : signals_) {
    decoded.set(signal.key, SignalCodec::extract(raw, signal));
  }
  return decoded;
}

CANPacket FrameSpec::packet(uint8_t device_id, const SignalValues & values) const
{
  return packetFromRaw(device_id, encodeRaw(values));
}

CANPacket FrameSpec::packetFromRaw(uint8_t device_id, uint64_t raw_payload) const
{
  CANPacket packet;
  packet.arbitration_id = arbitrationId(device_id);
  packet.dlc = static_cast<uint8_t>(length_bytes_);
  packet.is_extended_id = true;
  packet.is_remote_frame = rtr_;
  packet.frame_name = key_;
  if (!rtr_) {rawToPayload(raw_payload, packet.data.data(), length_bytes_);}
  return packet;
}

uint64_t FrameSpec::payloadToRaw(const uint8_t * data, std::size_t length)
{
  uint64_t raw = 0;
  for (std::size_t i = 0; i < length && i < 8; ++i) {
    raw |= static_cast<uint64_t>(data[i]) << (8 * i);
  }
  return raw;
}

void FrameSpec::rawToPayload(uint64_t raw, uint8_t * data, std::size_t length)
{
  for (std::size_t i = 0; i < length && i < 8; ++i) {
    data[i] = static_cast<uint8_t>((raw >> (8 * i)) & 0xFFu);
  }
}

// -----------------------------------------------------------------------------
// SparkFrameDatabase
// -----------------------------------------------------------------------------

SparkFrameDatabase::Ptr SparkFrameDatabase::loadFile(const std::string & json_path)
{
  return fromJson(json::parseFile(json_path), json_path);
}

SparkFrameDatabase::Ptr SparkFrameDatabase::loadString(
  std::string_view json_text, std::string source)
{
  return fromJson(json::parse(json_text), std::move(source));
}

SparkFrameDatabase::Ptr SparkFrameDatabase::fromJson(const json::Value & root, std::string source)
{
  std::shared_ptr<SparkFrameDatabase> db(new SparkFrameDatabase());
  db->source_ = std::move(source);
  db->frames_version_ = root.stringOr("framesVersion", "unknown");
  db->spec_version_ = root.stringOr("frcJsonSpecVersion", "unknown");
  if (const json::Value * info = root.find("deviceInfo")) {
    db->device_info_.device_type = info->stringOr("deviceType", "");
    db->device_info_.device_type_number = static_cast<int>(info->doubleOr("deviceTypeNumber", -1));
    db->device_info_.manufacturer = info->stringOr("manufacturer", "");
    db->device_info_.manufacturer_number =
      static_cast<int>(info->doubleOr("manufacturerNumber", -1));
  }

  for (const char * section : {"periodicFrames", "nonPeriodicFrames"}) {
    const json::Value * frames = root.find(section);
    if (frames == nullptr) {continue;}
    for (const auto & member : frames->asObject()) {
      if (db->by_key_.count(member.key) != 0) {
        throw SpecError("Clave de trama duplicada: " + member.key);
      }
      try {
        db->frames_.push_back(FrameSpec::fromJson(member.key, section, member.value));
      } catch (const json::TypeError & error) {
        throw SpecError("Trama " + member.key + ": " + error.what());
      }
      const std::size_t index = db->frames_.size() - 1;
      db->by_key_.emplace(member.key, index);
      db->by_base_id_.emplace(db->frames_.back().baseArbId() & ~0x3Fu, index);
    }
  }
  if (db->frames_.empty()) {
    throw SpecError(db->source_ + ": el JSON no contiene tramas");
  }
  return db;
}

const FrameSpec * SparkFrameDatabase::find(std::string_view key) const
{
  const auto it = by_key_.find(std::string(key));
  return it == by_key_.end() ? nullptr : &frames_[it->second];
}

const FrameSpec & SparkFrameDatabase::at(std::string_view key) const
{
  const FrameSpec * frame = find(key);
  if (frame == nullptr) {
    throw SpecError("El JSON (" + frames_version_ + ") no contiene la trama '" +
            std::string(key) + "'");
  }
  return *frame;
}

const FrameSpec * SparkFrameDatabase::matchArbitrationId(uint32_t arbitration_id) const
{
  const auto it = by_base_id_.find(arbitration_id & ~0x3Fu);
  return it == by_base_id_.end() ? nullptr : &frames_[it->second];
}

std::vector<const FrameSpec *> SparkFrameDatabase::search(std::string_view text) const
{
  auto lower = [](std::string value) {
      std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) {return static_cast<char>(std::tolower(c));});
      return value;
    };
  const std::string needle = lower(std::string(text));
  std::vector<const FrameSpec *> result;
  for (const auto & frame : frames_) {
    if (lower(frame.key()).find(needle) != std::string::npos ||
      lower(frame.name()).find(needle) != std::string::npos ||
      lower(frame.description()).find(needle) != std::string::npos)
    {
      result.push_back(&frame);
    }
  }
  return result;
}

}  // namespace sparkmax_protocol
