#include "sparkmax_protocol/signal_codec.hpp"

#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace sparkmax
{

namespace
{

std::string lower(std::string text)
{
  for (auto & c : text) {c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));}
  return text;
}

bool is_bool_type(const std::string & type) {return type == "boolean" || type == "bool";}

// Python: int(round(x)) redondea "half to even"; std::nearbyint también
// (modo por defecto FE_TONEAREST). std::round NO sería equivalente.
std::int64_t python_round(double value)
{
  if (!std::isfinite(value) || std::fabs(value) > 9.0e18) {
    throw std::invalid_argument("Valor no representable como entero: " + std::to_string(value));
  }
  return static_cast<std::int64_t>(std::nearbyint(value));
}

// Python: struct.pack("<f", x) lanza OverflowError si x no cabe en float32.
float to_float32(double value)
{
  if (std::isfinite(value) && std::fabs(value) > std::numeric_limits<float>::max()) {
    throw std::invalid_argument("float too large to pack with f format");
  }
  return static_cast<float>(value);
}

}  // namespace

SignalSpec SignalSpec::from_json(const std::string & key, const nlohmann::ordered_json & spec)
{
  SignalSpec s;
  s.key = key;
  s.type = lower(spec.at("type").get<std::string>());
  s.bit_position = spec.at("bitPosition").get<unsigned>();
  s.length_bits = spec.at("lengthBits").get<unsigned>();
  s.is_big_endian = spec.value("isBigEndian", false);
  s.decode_scale_factor = spec.value("decodeScaleFactor", 1.0);
  s.offset = spec.value("offset", 0.0);
  if (spec.contains("encodedMin")) {s.encoded_min = spec["encodedMin"].get<std::int64_t>();}
  if (spec.contains("encodedMax")) {s.encoded_max = spec["encodedMax"].get<std::int64_t>();}
  if (spec.contains("decodedMin")) {s.decoded_min = spec["decodedMin"].get<double>();}
  if (spec.contains("decodedMax")) {s.decoded_max = spec["decodedMax"].get<double>();}
  s.name = spec.value("name", "");
  s.description = spec.value("description", "");
  if (spec.contains("unit")) {s.unit = spec["unit"].get<std::string>();}

  if (s.length_bits == 0 || s.length_bits > 64) {
    throw SpecError("Signal " + key + ": lengthBits fuera de rango");
  }
  return s;
}

void SignalCodec::require_little_endian(const SignalSpec & signal_spec)
{
  if (signal_spec.is_big_endian) {
    // Python: NotImplementedError
    throw std::logic_error(
            "This codec currently supports the little-endian REV signals used "
            "by MAXMotion/parameter access.");
  }
}

std::uint32_t SignalCodec::float_to_u32(float value)
{
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float SignalCodec::u32_to_float(std::uint32_t value)
{
  float result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

std::uint64_t SignalCodec::encode_bits(const SignalSpec & signal_spec, double decoded_value)
{
  require_little_endian(signal_spec);

  const auto & signal_type = signal_spec.type;
  const unsigned bits = signal_spec.length_bits;
  const double scale = signal_spec.decode_scale_factor;
  const double offset = signal_spec.offset;

  if (signal_type == "float") {
    const double raw_float = (decoded_value - offset) / scale;
    if (bits != 32) {
      throw SpecError("Unsupported float width: " + std::to_string(bits));
    }
    return float_to_u32(to_float32(raw_float));
  }

  std::int64_t encoded;
  if (is_bool_type(signal_type)) {
    encoded = decoded_value != 0.0 ? 1 : 0;
  } else {
    if (scale == 0.0) {
      throw SpecError("decodeScaleFactor cannot be zero");
    }
    encoded = python_round((decoded_value - offset) / scale);
  }

  if (signal_spec.encoded_min && encoded < *signal_spec.encoded_min) {
    throw std::invalid_argument(
            "Encoded value " + std::to_string(encoded) + " is below encodedMin=" +
            std::to_string(*signal_spec.encoded_min));
  }
  if (signal_spec.encoded_max && encoded > *signal_spec.encoded_max) {
    throw std::invalid_argument(
            "Encoded value " + std::to_string(encoded) + " is above encodedMax=" +
            std::to_string(*signal_spec.encoded_max));
  }

  if (signal_type == "int") {
    const std::int64_t min_signed = -(std::int64_t{1} << (bits - 1));
    const std::int64_t max_signed = (std::int64_t{1} << (bits - 1)) - 1;
    if (encoded < min_signed || encoded > max_signed) {
      throw std::invalid_argument(
              "Signed " + std::to_string(bits) + "-bit value out of range: " +
              std::to_string(encoded));
    }
    // Complemento a dos en `bits` bits (equivale a (1 << bits) + encoded).
    return static_cast<std::uint64_t>(encoded) & bit_mask(bits);
  }
  if (signal_type == "uint" || is_bool_type(signal_type)) {
    if (encoded < 0 || static_cast<std::uint64_t>(encoded) > bit_mask(bits)) {
      throw std::invalid_argument(
              "Unsigned " + std::to_string(bits) + "-bit value out of range: " +
              std::to_string(encoded));
    }
    return static_cast<std::uint64_t>(encoded);
  }
  throw SpecError("Unsupported signal type: '" + signal_type + "'");
}

double SignalCodec::decode_bits(const SignalSpec & signal_spec, std::uint64_t raw_bits)
{
  require_little_endian(signal_spec);

  const auto & signal_type = signal_spec.type;
  const unsigned bits = signal_spec.length_bits;
  const double scale = signal_spec.decode_scale_factor;
  const double offset = signal_spec.offset;

  raw_bits &= bit_mask(bits);

  if (signal_type == "float") {
    if (bits != 32) {
      throw SpecError("Unsupported float width: " + std::to_string(bits));
    }
    const double raw_float = u32_to_float(static_cast<std::uint32_t>(raw_bits));
    return raw_float * scale + offset;
  }

  if (signal_type == "int") {
    const std::uint64_t sign_bit = std::uint64_t{1} << (bits - 1);
    const std::int64_t encoded = (raw_bits & sign_bit) ?
      static_cast<std::int64_t>(raw_bits | ~bit_mask(bits)) :
      static_cast<std::int64_t>(raw_bits);
    return static_cast<double>(encoded) * scale + offset;
  }

  if (signal_type == "uint") {
    return static_cast<double>(raw_bits) * scale + offset;
  }

  if (is_bool_type(signal_type)) {
    return raw_bits ? 1.0 : 0.0;
  }

  throw SpecError("Unsupported signal type: '" + signal_type + "'");
}

}  // namespace sparkmax
