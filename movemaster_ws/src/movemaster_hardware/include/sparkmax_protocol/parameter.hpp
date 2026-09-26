// parameter.hpp
//
// Parameter catalog: DEFAULT_PARAMETER_LAYOUT, PARAMETER_TYPE_CODE,
// PARAMETER_TYPE_NAME, ParameterDefinition y ParameterGroup.
//
// IMPORTANT (igual que en Python):
// The CAN-frame JSON describes HOW parameter access works (PARAMETER_WRITE,
// PARAMETER_WRITE_RESPONSE, READ_PARAMETER_x_AND_y, etc.), but it does not map
// semantic names such as "P 0" or "MAXMotion Max Accel 0" to parameter IDs.
// Those IDs belong to REV's separate SPARK parameter specification.
// "base_id" is the slot-0 parameter ID and "slot_stride" advances to slot 1..3.

#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sparkmax
{

struct ParameterLayoutEntry
{
  int base_id;
  int slot_stride;
  std::string type{"float"};
  std::string description{};
  std::optional<std::string> unit{};
  std::vector<std::string> aliases{};
};

/// layout[group][key] -> ParameterLayoutEntry
using ParameterLayout = std::map<std::string, std::map<std::string, ParameterLayoutEntry>>;

inline const ParameterLayout DEFAULT_PARAMETER_LAYOUT = {
  {"pidf", {
      {"p", {13, 8, "float", "Proportional gain"}},
      {"i", {14, 8, "float", "Integral gain"}},
      {"d", {15, 8, "float", "Derivative gain"}},
      {"f", {16, 8, "float", "Feedforward gain (F parameter)"}},
    }},
  {"maxmotion", {
      {"cruise_velocity", {166, 5, "float", "MAXMotion cruise/max velocity",
          "RPM by default", {"cruisevelocity", "max_velocity", "maxvelocity"}}},
      {"max_acceleration", {167, 5, "float", "MAXMotion maximum acceleration",
          "RPM/s by default", {"maxaccel", "max_accel", "maxacceleration"}}},
      {"allowed_profile_error", {169, 5, "float", "MAXMotion allowed profile/closed-loop error",
          "rotations by default",
          {"allowedprofileerror", "allowed_error", "allowed_closed_loop_error"}}},
    }},
};

inline const std::map<std::string, int> PARAMETER_TYPE_CODE = {
  {"int", 1}, {"uint", 2}, {"float", 3}, {"bool", 4}, {"boolean", 4},
};

inline const std::map<int, std::string> PARAMETER_TYPE_NAME = {
  {1, "int"}, {2, "uint"}, {3, "float"}, {4, "bool"},
};

struct ParameterDefinition
{
  std::string group;
  std::string key;
  int slot;
  int parameter_id;
  std::string value_type;
  std::string description{};
  std::optional<std::string> unit{};

  int pair_start_id() const {return parameter_id & ~1;}
  int pair_index() const {return parameter_id - pair_start_id();}
  std::string read_frame_name() const
  {
    return "READ_PARAMETER_" + std::to_string(pair_start_id()) + "_AND_" +
           std::to_string(pair_start_id() + 1);
  }
};

/// Dictionary-like group supporting canonical names and aliases.
class ParameterGroup
{
public:
  ParameterGroup() = default;
  ParameterGroup(
    std::map<std::string, ParameterDefinition> canonical,
    std::map<std::string, std::string> aliases);

  static std::string normalize(const std::string & key);

  /// Lanza std::out_of_range (KeyError) si la clave/alias no existe.
  const ParameterDefinition & operator[](const std::string & key) const;
  bool contains(const std::string & key) const;

  auto begin() const {return canonical_.begin();}
  auto end() const {return canonical_.end();}
  std::size_t size() const {return canonical_.size();}
  const std::map<std::string, ParameterDefinition> & as_dict() const {return canonical_;}

private:
  std::map<std::string, ParameterDefinition> canonical_;
  std::map<std::string, std::string> aliases_;
};

}  // namespace sparkmax
