// signal_codec.hpp
//
// JSON signal codec (traducción de SpecError y SignalCodec).
//
// En Python cada signal_spec es un dict del JSON. En C++ se convierte una sola
// vez a SignalSpec para no recorrer el JSON en cada ciclo de control.

#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace sparkmax
{

class SpecError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Equivalente al TimeoutError de Python.
class TimeoutError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Una entrada de "signals" del JSON REV.
struct SignalSpec
{
  std::string key;          // p. ej. "SETPOINT"
  std::string type;         // "float" | "int" | "uint" | "boolean" | "bool"
  unsigned bit_position{0};
  unsigned length_bits{0};
  bool is_big_endian{false};
  double decode_scale_factor{1.0};
  double offset{0.0};
  std::optional<std::int64_t> encoded_min{};
  std::optional<std::int64_t> encoded_max{};
  std::optional<double> decoded_min{};
  std::optional<double> decoded_max{};
  std::string name{};
  std::string description{};
  std::optional<std::string> unit{};

  static SignalSpec from_json(const std::string & key, const nlohmann::ordered_json & spec);
};

/// Encode/decode one signal using its JSON definition.
///
/// Los valores decodificados se representan como double: cubre float, int,
/// uint de hasta 32 bits sin pérdida y boolean (0.0 / 1.0).
class SignalCodec
{
public:
  static std::uint64_t encode_bits(const SignalSpec & signal_spec, double decoded_value);
  static double decode_bits(const SignalSpec & signal_spec, std::uint64_t raw_bits);

  static std::uint32_t float_to_u32(float value);
  static float u32_to_float(std::uint32_t value);

private:
  static void require_little_endian(const SignalSpec & signal_spec);
};

/// Máscara de n bits (0 < n <= 64) sin desbordar el desplazamiento.
constexpr std::uint64_t bit_mask(unsigned bits)
{
  return bits >= 64 ? ~std::uint64_t{0} : (std::uint64_t{1} << bits) - 1;
}

}  // namespace sparkmax
