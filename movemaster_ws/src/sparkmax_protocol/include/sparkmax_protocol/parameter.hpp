// parameter.hpp
//
// Catálogo de parámetros del SPARK (ParameterDefinition / ParameterGroup).
//
// IMPORTANTE (igual que en sparkmax_json_protocol.py): el JSON de tramas describe
// CÓMO se accede a los parámetros (PARAMETER_WRITE, PARAMETER_WRITE_RESPONSE,
// READ_PARAMETER_x_AND_y...), pero no asocia nombres como "P 0" a IDs. Esos IDs
// pertenecen a la especificación de parámetros de REV (SparkParameters), por
// eso el catálogo es un dato reemplazable (ParameterLayout).
// "base_id" es el ID del slot 0 y "slot_stride" avanza a los slots 1..3.

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace sparkmax_protocol
{

/// Código de tipo usado por PARAMETER_WRITE_RESPONSE.PARAMETER_TYPE.
enum class ParameterType : uint8_t { Unused = 0, Int = 1, Uint = 2, Float = 3, Bool = 4 };

ParameterType parameterTypeFromString(std::string_view text);
ParameterType parameterTypeFromCode(uint8_t code);
const char * toString(ParameterType type);

/// Empaqueta un valor en los 32 bits de VALUE según el tipo del parámetro.
uint32_t packParameterValue(double value, ParameterType type);
/// Interpreta los 32 bits de VALUE según el tipo del parámetro.
double unpackParameterValue(uint32_t raw, ParameterType type);

struct ParameterDefinition
{
  std::string group;
  std::string key;
  int slot{0};
  uint8_t parameter_id{0};
  ParameterType value_type{ParameterType::Float};
  std::string description;
  std::string unit;

  uint8_t pairStartId() const {return static_cast<uint8_t>(parameter_id & ~1u);}
  int pairIndex() const {return parameter_id - pairStartId();}
  /// Nombre de la trama RTR que lee este parámetro (READ_PARAMETER_a_AND_b).
  std::string readFrameName() const;
};

/// Una entrada del catálogo: el ID del slot s es base_id + s * slot_stride.
struct ParameterLayoutEntry
{
  std::string key;
  int base_id{0};
  int slot_stride{0};
  ParameterType type{ParameterType::Float};
  std::string description;
  std::string unit;
  std::vector<std::string> aliases;
};

struct ParameterGroupLayout
{
  std::string name;
  std::vector<ParameterLayoutEntry> entries;
};

using ParameterLayout = std::vector<ParameterGroupLayout>;

/// Grupo tipo diccionario con nombres canónicos y alias (p.ej. "cruisevelocity").
class ParameterGroup
{
public:
  ParameterGroup() = default;
  ParameterGroup(std::vector<ParameterDefinition> definitions,
    std::map<std::string, std::string> aliases);

  /// minúsculas, '-' y ' ' → '_'
  static std::string normalize(std::string_view key);

  const ParameterDefinition * find(std::string_view key) const;
  /// Lanza std::out_of_range si la clave (o alias) no existe.
  const ParameterDefinition & at(std::string_view key) const;
  const ParameterDefinition & operator[](std::string_view key) const {return at(key);}
  bool contains(std::string_view key) const {return find(key) != nullptr;}

  const std::vector<ParameterDefinition> & definitions() const {return definitions_;}
  std::vector<ParameterDefinition>::const_iterator begin() const {return definitions_.begin();}
  std::vector<ParameterDefinition>::const_iterator end() const {return definitions_.end();}
  std::size_t size() const {return definitions_.size();}

private:
  std::vector<ParameterDefinition> definitions_;
  std::map<std::string, std::string> aliases_;  // normalizado → canónico
};

/// Construye el grupo de cada slot (0..slot_count-1) a partir del catálogo.
std::vector<ParameterGroup> buildGroupsBySlot(const ParameterGroupLayout & layout, int slot_count);

namespace parameter_layouts
{
/// Ganancias de lazo cerrado por slot (IDs de sparkmax_json_protocol.py:
/// P=13, I=14, D=15, F=16, stride 8). i_zone/d_filter/output_min/output_max
/// siguen la tabla REV SparkParameters (17..20); verifíquelos con su firmware.
ParameterGroupLayout pidf();
/// Parámetros MAXMotion por slot (IDs de sparkmax_json_protocol.py).
ParameterGroupLayout maxmotion();
/// Parámetros de inicialización sin slot usados por teach_pendant_backend.py
/// (sensor de realimentación, factores de conversión, wrapping, periodos STATUS).
ParameterGroupLayout setup();

/// Catálogo por defecto de SparkMAXMotionProtocol en Python (pidf + maxmotion).
ParameterLayout maxMotionDefault();
/// Catálogo por defecto de PositionProtocol (pidf + setup).
ParameterLayout positionDefault();
}  // namespace parameter_layouts

}  // namespace sparkmax_protocol
