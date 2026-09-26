#include "sparkmax_protocol/parameter.hpp"

#include <cctype>
#include <cmath>
#include <stdexcept>

#include "sparkmax_protocol/signal_codec.hpp"

namespace sparkmax_protocol
{

ParameterType parameterTypeFromString(std::string_view text)
{
  if (text == "int") {return ParameterType::Int;}
  if (text == "uint") {return ParameterType::Uint;}
  if (text == "float") {return ParameterType::Float;}
  if (text == "bool" || text == "boolean") {return ParameterType::Bool;}
  throw std::invalid_argument("Tipo de parámetro no soportado: '" + std::string(text) + "'");
}

ParameterType parameterTypeFromCode(uint8_t code)
{
  switch (code) {
    case 1: return ParameterType::Int;
    case 2: return ParameterType::Uint;
    case 3: return ParameterType::Float;
    case 4: return ParameterType::Bool;
    default: return ParameterType::Unused;
  }
}

const char * toString(ParameterType type)
{
  switch (type) {
    case ParameterType::Unused: return "unknown";
    case ParameterType::Int: return "int";
    case ParameterType::Uint: return "uint";
    case ParameterType::Float: return "float";
    case ParameterType::Bool: return "bool";
  }
  return "unknown";
}

uint32_t packParameterValue(double value, ParameterType type)
{
  switch (type) {
    case ParameterType::Float:
      if (std::isfinite(value) && std::fabs(value) > 3.4028234663852886e38) {
        throw std::out_of_range("Parámetro float fuera del rango de float32");
      }
      return SignalCodec::floatToU32(static_cast<float>(value));
    case ParameterType::Int: {
        if (!std::isfinite(value)) {throw std::invalid_argument("Parámetro int no finito");}
        const double truncated = std::trunc(value);
        if (truncated < -2147483648.0 || truncated > 2147483647.0) {
          throw std::out_of_range("Parámetro int debe caber en 32 bits con signo");
        }
        return static_cast<uint32_t>(static_cast<int32_t>(truncated));
      }
    case ParameterType::Uint: {
        if (!std::isfinite(value)) {throw std::invalid_argument("Parámetro uint no finito");}
        const double truncated = std::trunc(value);
        if (truncated < 0.0 || truncated > 4294967295.0) {
          throw std::out_of_range("Parámetro uint debe caber en 32 bits");
        }
        return static_cast<uint32_t>(truncated);
      }
    case ParameterType::Bool:
      return value != 0.0 ? 1u : 0u;
    case ParameterType::Unused:
      break;
  }
  throw std::invalid_argument("Tipo de parámetro no soportado");
}

double unpackParameterValue(uint32_t raw, ParameterType type)
{
  switch (type) {
    case ParameterType::Float: return static_cast<double>(SignalCodec::u32ToFloat(raw));
    case ParameterType::Int: return static_cast<double>(static_cast<int32_t>(raw));
    case ParameterType::Uint: return static_cast<double>(raw);
    case ParameterType::Bool: return raw != 0 ? 1.0 : 0.0;
    case ParameterType::Unused: break;
  }
  throw std::invalid_argument("Tipo de parámetro no soportado");
}

std::string ParameterDefinition::readFrameName() const
{
  const int start = pairStartId();
  return "READ_PARAMETER_" + std::to_string(start) + "_AND_" + std::to_string(start + 1);
}

// -----------------------------------------------------------------------------
// ParameterGroup
// -----------------------------------------------------------------------------

ParameterGroup::ParameterGroup(
  std::vector<ParameterDefinition> definitions, std::map<std::string, std::string> aliases)
: definitions_(std::move(definitions)), aliases_(std::move(aliases)) {}

std::string ParameterGroup::normalize(std::string_view key)
{
  std::size_t first = 0;
  std::size_t last = key.size();
  while (first < last && std::isspace(static_cast<unsigned char>(key[first]))) {++first;}
  while (last > first && std::isspace(static_cast<unsigned char>(key[last - 1]))) {--last;}
  std::string out;
  out.reserve(last - first);
  for (std::size_t i = first; i < last; ++i) {
    const char c = key[i];
    if (c == '-' || c == ' ') {
      out.push_back('_');
    } else {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  return out;
}

const ParameterDefinition * ParameterGroup::find(std::string_view key) const
{
  const std::string normalized = normalize(key);
  const auto alias = aliases_.find(normalized);
  const std::string & canonical = alias == aliases_.end() ? normalized : alias->second;
  for (const auto & definition : definitions_) {
    if (definition.key == canonical) {return &definition;}
  }
  return nullptr;
}

const ParameterDefinition & ParameterGroup::at(std::string_view key) const
{
  const ParameterDefinition * definition = find(key);
  if (definition == nullptr) {
    throw std::out_of_range("Parámetro desconocido: '" + std::string(key) + "'");
  }
  return *definition;
}

std::vector<ParameterGroup> buildGroupsBySlot(const ParameterGroupLayout & layout, int slot_count)
{
  if (slot_count <= 0) {throw std::invalid_argument("slot_count debe ser positivo");}
  std::vector<ParameterGroup> result;
  result.reserve(static_cast<std::size_t>(slot_count));
  for (int slot = 0; slot < slot_count; ++slot) {
    std::vector<ParameterDefinition> definitions;
    std::map<std::string, std::string> aliases;
    for (const auto & entry : layout.entries) {
      const int id = entry.base_id + slot * entry.slot_stride;
      if (id < 0 || id > 255) {
        throw std::out_of_range("ID de parámetro fuera de 0..255 para " + layout.name + "." +
                entry.key);
      }
      ParameterDefinition definition;
      definition.group = layout.name;
      definition.key = entry.key;
      definition.slot = slot;
      definition.parameter_id = static_cast<uint8_t>(id);
      definition.value_type = entry.type;
      definition.description = entry.description;
      definition.unit = entry.unit;
      definitions.push_back(definition);
      aliases[ParameterGroup::normalize(entry.key)] = entry.key;
      for (const auto & alias : entry.aliases) {
        aliases[ParameterGroup::normalize(alias)] = entry.key;
      }
    }
    result.emplace_back(std::move(definitions), std::move(aliases));
  }
  return result;
}

// -----------------------------------------------------------------------------
// Catálogos por defecto
// -----------------------------------------------------------------------------

namespace parameter_layouts
{

ParameterGroupLayout pidf()
{
  return {"pidf", {
      {"p", 13, 8, ParameterType::Float, "Proportional gain", "", {"kp"}},
      {"i", 14, 8, ParameterType::Float, "Integral gain", "", {"ki"}},
      {"d", 15, 8, ParameterType::Float, "Derivative gain", "", {"kd"}},
      {"f", 16, 8, ParameterType::Float,
        "Feedforward gain (F; REV lo llama kV en firmware reciente)", "", {"kf", "kv"}},
      {"i_zone", 17, 8, ParameterType::Float, "Integral zone (verificar con SparkParameters)",
        "rotaciones", {"izone"}},
      {"d_filter", 18, 8, ParameterType::Float,
        "Derivative filter (verificar con SparkParameters)", "", {"dfilter"}},
      {"output_min", 19, 8, ParameterType::Float,
        "Minimum closed-loop output (verificar con SparkParameters)", "duty cycle -1..1",
        {"min_output"}},
      {"output_max", 20, 8, ParameterType::Float,
        "Maximum closed-loop output (verificar con SparkParameters)", "duty cycle -1..1",
        {"max_output"}},
    }};
}

ParameterGroupLayout maxmotion()
{
  return {"maxmotion", {
      {"cruise_velocity", 166, 5, ParameterType::Float, "MAXMotion cruise/max velocity",
        "RPM by default", {"cruisevelocity", "max_velocity", "maxvelocity"}},
      {"max_acceleration", 167, 5, ParameterType::Float, "MAXMotion maximum acceleration",
        "RPM/s by default", {"maxaccel", "max_accel", "maxacceleration"}},
      {"allowed_profile_error", 169, 5, ParameterType::Float,
        "MAXMotion allowed profile/closed-loop error", "rotations by default",
        {"allowedprofileerror", "allowed_error", "allowed_closed_loop_error"}},
    }};
}

ParameterGroupLayout setup()
{
  return {"setup", {
      {"feedback_sensor", 9, 0, ParameterType::Uint, "Sensor de realimentación (1 = primario)",
        "", {}},
      {"position_factor", 112, 0, ParameterType::Float, "Factor de conversión de posición",
        "", {"position_conversion_factor"}},
      {"velocity_factor", 113, 0, ParameterType::Float, "Factor de conversión de velocidad",
        "", {"velocity_conversion_factor"}},
      {"position_wrapping", 149, 0, ParameterType::Bool, "Position wrapping habilitado", "", {}},
      {"status0_period_ms", 158, 0, ParameterType::Uint, "Periodo de STATUS_0", "ms", {}},
      {"status2_period_ms", 160, 0, ParameterType::Uint, "Periodo de STATUS_2", "ms", {}},
    }};
}

ParameterLayout maxMotionDefault()
{
  return {pidf(), maxmotion()};
}

ParameterLayout positionDefault()
{
  return {pidf(), setup()};
}

}  // namespace parameter_layouts

}  // namespace sparkmax_protocol
