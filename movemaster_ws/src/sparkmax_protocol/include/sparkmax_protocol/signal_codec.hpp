// signal_codec.hpp
//
// Definición de una señal del JSON de REV y su codificación/decodificación
// (equivalente a SignalCodec de sparkmax_json_protocol.py).
//
// Todas las señales usadas por el control de posición y el acceso a parámetros
// son little-endian (orden Intel). Las señales big-endian del JSON (sólo
// GET_FIRMWARE_VERSION.BUILD en la versión 2.1.0) se cargan, pero codificarlas
// o decodificarlas lanza una excepción, igual que en la versión Python.

#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include "sparkmax_protocol/json.hpp"

namespace sparkmax_protocol
{

/// Error en la especificación JSON o en su uso (equivale a SpecError en Python).
class SpecError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

enum class SignalType { Int, Uint, Float, Boolean };

SignalType signalTypeFromString(std::string_view text);
const char * toString(SignalType type);

struct SignalSpec
{
  std::string key;          // p.ej. "SETPOINT"
  std::string name;         // p.ej. "Setpoint"
  std::string description;
  SignalType type{SignalType::Uint};
  unsigned bit_position{0};
  unsigned length_bits{0};
  bool is_big_endian{false};
  double scale{1.0};        // decodeScaleFactor
  double offset{0.0};
  std::optional<double> encoded_min;
  std::optional<double> encoded_max;
  std::optional<double> decoded_min;
  std::optional<double> decoded_max;

  /// Máscara de `length_bits` bits (sin desplazar).
  uint64_t mask() const
  {
    return length_bits >= 64 ? ~uint64_t{0} : ((uint64_t{1} << length_bits) - 1u);
  }

  /// true si el JSON define un único valor posible (p.ej. MAGIC_NUMBER).
  bool isConstant() const
  {
    return decoded_min.has_value() && decoded_max.has_value() && *decoded_min == *decoded_max;
  }

  static SignalSpec fromJson(const std::string & key, const json::Value & spec);
};

/// Valor a codificar en una señal: número "decodificado" (con escala/offset del
/// JSON) o bits crudos ya empaquetados (p.ej. VALUE de PARAMETER_WRITE).
class SignalValue
{
public:
  template<typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  SignalValue(T value)  // NOLINT(google-explicit-constructor): conversión intencional
  : number_(static_cast<double>(value)) {}

  static SignalValue raw(uint64_t bits)
  {
    SignalValue v(0.0);
    v.is_raw_ = true;
    v.raw_ = bits;
    return v;
  }

  bool isRaw() const {return is_raw_;}
  double number() const {return number_;}
  uint64_t rawBits() const {return raw_;}

private:
  double number_{0.0};
  uint64_t raw_{0};
  bool is_raw_{false};
};

/// Resultado de decodificar una señal.
struct DecodedSignal
{
  double value{0.0};   // valor físico (escala y offset aplicados; bool → 0/1)
  uint64_t raw{0};     // bits crudos de la señal
};

class SignalCodec
{
public:
  /// Convierte un valor a los bits de la señal (sin desplazar).
  /// Lanza std::out_of_range si el valor no cabe o viola encodedMin/Max.
  static uint64_t encodeBits(const SignalSpec & spec, const SignalValue & value);

  /// Convierte los bits de la señal (sin desplazar) al valor físico.
  static DecodedSignal decodeBits(const SignalSpec & spec, uint64_t raw_bits);

  /// Inserta la señal en una trama little-endian de hasta 64 bits.
  static uint64_t insert(uint64_t raw_frame, const SignalSpec & spec, const SignalValue & value);

  /// Extrae la señal de una trama little-endian de hasta 64 bits.
  static DecodedSignal extract(uint64_t raw_frame, const SignalSpec & spec);

  static uint32_t floatToU32(float value);
  static float u32ToFloat(uint32_t bits);
};

}  // namespace sparkmax_protocol
