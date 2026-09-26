#include "sparkmax_protocol/signal_codec.hpp"

#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

namespace sparkmax_protocol
{

SignalType signalTypeFromString(std::string_view text)
{
  if (text == "int") {return SignalType::Int;}
  if (text == "uint") {return SignalType::Uint;}
  if (text == "float") {return SignalType::Float;}
  if (text == "boolean" || text == "bool") {return SignalType::Boolean;}
  throw SpecError("Tipo de señal no soportado: '" + std::string(text) + "'");
}

const char * toString(SignalType type)
{
  switch (type) {
    case SignalType::Int: return "int";
    case SignalType::Uint: return "uint";
    case SignalType::Float: return "float";
    case SignalType::Boolean: return "boolean";
  }
  return "?";
}

namespace
{
std::optional<double> optionalNumber(const json::Value & spec, std::string_view key)
{
  const json::Value * value = spec.find(key);
  if (value == nullptr || !value->isNumber()) {return std::nullopt;}
  return value->asDouble();
}

std::string describe(const SignalSpec & spec)
{
  return "señal '" + spec.key + "'";
}
}  // namespace

SignalSpec SignalSpec::fromJson(const std::string & key, const json::Value & spec)
{
  SignalSpec s;
  s.key = key;
  s.name = spec.stringOr("name", key);
  s.description = spec.stringOr("description", "");
  std::string type = spec.at("type").asString();
  for (auto & c : type) {c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));}
  s.type = signalTypeFromString(type);
  s.bit_position = static_cast<unsigned>(spec.at("bitPosition").asUint64());
  s.length_bits = static_cast<unsigned>(spec.at("lengthBits").asUint64());
  s.is_big_endian = spec.boolOr("isBigEndian", false);
  s.scale = spec.doubleOr("decodeScaleFactor", 1.0);
  s.offset = spec.doubleOr("offset", 0.0);
  s.encoded_min = optionalNumber(spec, "encodedMin");
  s.encoded_max = optionalNumber(spec, "encodedMax");
  s.decoded_min = optionalNumber(spec, "decodedMin");
  s.decoded_max = optionalNumber(spec, "decodedMax");

  if (s.length_bits == 0 || s.length_bits > 64) {
    throw SpecError(describe(s) + ": lengthBits inválido");
  }
  if (s.bit_position + s.length_bits > 64) {
    throw SpecError(describe(s) + ": excede 64 bits");
  }
  if (s.type == SignalType::Float && s.length_bits != 32) {
    // Se acepta al cargar; codificar/decodificar fallará como en Python.
  }
  return s;
}

uint32_t SignalCodec::floatToU32(float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float SignalCodec::u32ToFloat(uint32_t bits)
{
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

namespace
{
void requireLittleEndian(const SignalSpec & spec)
{
  if (spec.is_big_endian) {
    throw SpecError(describe(spec) + ": el codec sólo soporta señales little-endian");
  }
}

void checkEncodedRange(const SignalSpec & spec, double encoded)
{
  if (spec.encoded_min && encoded < *spec.encoded_min) {
    std::ostringstream out;
    out << describe(spec) << ": valor codificado " << encoded << " menor que encodedMin=" <<
      *spec.encoded_min;
    throw std::out_of_range(out.str());
  }
  if (spec.encoded_max && encoded > *spec.encoded_max) {
    std::ostringstream out;
    out << describe(spec) << ": valor codificado " << encoded << " mayor que encodedMax=" <<
      *spec.encoded_max;
    throw std::out_of_range(out.str());
  }
}
}  // namespace

uint64_t SignalCodec::encodeBits(const SignalSpec & spec, const SignalValue & value)
{
  requireLittleEndian(spec);
  const unsigned bits = spec.length_bits;
  const uint64_t mask = spec.mask();

  if (value.isRaw()) {
    const uint64_t raw = value.rawBits();
    if ((raw & ~mask) != 0) {
      throw std::out_of_range(describe(spec) + ": bits crudos no caben en la señal");
    }
    if (spec.type == SignalType::Uint) {
      checkEncodedRange(spec, static_cast<double>(raw));
    }
    return raw;
  }

  const double decoded = value.number();

  if (spec.type == SignalType::Float) {
    if (bits != 32) {throw SpecError(describe(spec) + ": ancho float no soportado");}
    const double raw_float = (decoded - spec.offset) / spec.scale;
    if (std::isfinite(raw_float) &&
      std::fabs(raw_float) > static_cast<double>(std::numeric_limits<float>::max()))
    {
      throw std::out_of_range(describe(spec) + ": valor no representable como float32");
    }
    return floatToU32(static_cast<float>(raw_float));
  }

  if (spec.type == SignalType::Boolean) {
    const uint64_t encoded = decoded != 0.0 ? 1u : 0u;
    checkEncodedRange(spec, static_cast<double>(encoded));
    if (encoded > mask) {throw std::out_of_range(describe(spec) + ": fuera de rango");}
    return encoded;
  }

  if (spec.scale == 0.0) {throw SpecError(describe(spec) + ": decodeScaleFactor es cero");}
  // std::nearbyint redondea al par más cercano, igual que round() de Python.
  const double encoded = std::nearbyint((decoded - spec.offset) / spec.scale);
  if (!std::isfinite(encoded)) {
    throw std::out_of_range(describe(spec) + ": valor no finito");
  }
  checkEncodedRange(spec, encoded);

  if (spec.type == SignalType::Int) {
    const double min_signed = -std::ldexp(1.0, static_cast<int>(bits) - 1);
    const double max_signed = std::ldexp(1.0, static_cast<int>(bits) - 1) - 1.0;
    if (encoded < min_signed || encoded > max_signed) {
      throw std::out_of_range(describe(spec) + ": fuera del rango con signo");
    }
    const auto as_signed = static_cast<int64_t>(encoded);
    return static_cast<uint64_t>(as_signed) & mask;  // complemento a dos
  }

  // Uint
  if (encoded < 0.0 || encoded >= std::ldexp(1.0, static_cast<int>(bits))) {
    throw std::out_of_range(describe(spec) + ": fuera del rango sin signo");
  }
  return static_cast<uint64_t>(encoded);
}

DecodedSignal SignalCodec::decodeBits(const SignalSpec & spec, uint64_t raw_bits)
{
  requireLittleEndian(spec);
  const unsigned bits = spec.length_bits;
  raw_bits &= spec.mask();

  DecodedSignal out;
  out.raw = raw_bits;

  switch (spec.type) {
    case SignalType::Float:
      if (bits != 32) {throw SpecError(describe(spec) + ": ancho float no soportado");}
      out.value = static_cast<double>(u32ToFloat(static_cast<uint32_t>(raw_bits))) *
        spec.scale + spec.offset;
      break;
    case SignalType::Int: {
        int64_t encoded = 0;
        if (bits >= 64) {
          encoded = static_cast<int64_t>(raw_bits);
        } else {
          const uint64_t sign_bit = uint64_t{1} << (bits - 1);
          encoded = (raw_bits & sign_bit) ?
            static_cast<int64_t>(raw_bits | ~spec.mask()) : static_cast<int64_t>(raw_bits);
        }
        out.value = static_cast<double>(encoded) * spec.scale + spec.offset;
        break;
      }
    case SignalType::Uint:
      out.value = static_cast<double>(raw_bits) * spec.scale + spec.offset;
      break;
    case SignalType::Boolean:
      out.value = raw_bits != 0 ? 1.0 : 0.0;
      break;
  }
  return out;
}

uint64_t SignalCodec::insert(uint64_t raw_frame, const SignalSpec & spec, const SignalValue & value)
{
  const uint64_t encoded = encodeBits(spec, value) & spec.mask();
  const uint64_t shifted_mask = spec.mask() << spec.bit_position;
  return (raw_frame & ~shifted_mask) | (encoded << spec.bit_position);
}

DecodedSignal SignalCodec::extract(uint64_t raw_frame, const SignalSpec & spec)
{
  const uint64_t shifted = spec.bit_position >= 64 ? 0 : (raw_frame >> spec.bit_position);
  return decodeBits(spec, shifted & spec.mask());
}

}  // namespace sparkmax_protocol
