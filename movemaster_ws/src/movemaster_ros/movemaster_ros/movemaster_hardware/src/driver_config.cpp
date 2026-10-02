#include "movemaster_hardware/movemaster_driver.hpp"
#include <algorithm>
#include <fstream>
#include <set>
#include <utility>

namespace movemaster {
namespace {
// An unknown key is a typo, never a setting silently left at its default.
void only_keys(const Json &object, std::initializer_list<const char *> allowed) {
  for (const auto &item : object.items())
    if (std::none_of(allowed.begin(), allowed.end(), [&](const char *key) { return item.key() == key; }))
      throw std::invalid_argument(item.key() + " is not a known key");
}
const Json &field(const Json &object, const char *key) {
  if (!object.contains(key)) throw std::invalid_argument(std::string(key) + " is missing");
  return object.at(key);
}
const Json &object_field(const Json &object, const char *key) {
  const auto &value = field(object, key);
  if (!value.is_object()) throw std::invalid_argument(std::string(key) + " must be an object");
  return value;
}
int integer(const Json &object, const char *key) {
  const auto &value = field(object, key);
  if (!value.is_number_integer()) throw std::invalid_argument(std::string(key) + " must be an integer");
  const auto wide = value.get<std::int64_t>();
  if (wide < std::numeric_limits<int>::min() || wide > std::numeric_limits<int>::max())
    throw std::out_of_range(std::string(key) + " overflows int");
  return static_cast<int>(wide);
}
double number(const Json &object, const char *key) {
  const auto &value = field(object, key);
  if (!value.is_number()) throw std::invalid_argument(std::string(key) + " must be a number");
  return value.get<double>();
}
template <class Enum>
Enum choice(const Json &object, const char *key, std::initializer_list<std::pair<const char *, Enum>> options) {
  const auto &value = field(object, key);
  std::string names;
  for (const auto &[name, option] : options) {
    if (value.is_string() && value.get_ref<const std::string &>() == name) return option;
    names += (names.empty() ? "" : ", ") + std::string(name);
  }
  throw std::invalid_argument(std::string(key) + " must be one of: " + names);
}
// Prefixes errors with the block they come from, as in "spark.idle_mode must be ...".
template <class Parse>
void in(const std::string &block, Parse parse) {
  try { parse(); }
  catch (const std::exception &e) { throw std::invalid_argument(block + "." + e.what()); }
}
SlotConfig load_slot(const Json &s) {
  only_keys(s, {"pidf", "output_range", "maxmotion"});
  SlotConfig slot;
  slot.pidf = field(s, "pidf");
  if (s.contains("output_range")) {
    const auto &range = s.at("output_range");
    if (!range.is_array() || range.size() != 2 || !range[0].is_number() || !range[1].is_number())
      throw std::invalid_argument("output_range must be [min, max]");
    slot.output_min = range[0].get<double>();
    slot.output_max = range[1].get<double>();
  }
  if (s.contains("maxmotion")) slot.maxmotion = s.at("maxmotion");
  return slot;
}
JointConfig load_joint(const std::string &name, const Json &j) {
  if (!j.is_object()) throw std::invalid_argument("the joint entry must be an object");
  for (const auto *moved : {"slot", "pidf", "maxmotion"})
    if (j.contains(moved))
      throw std::invalid_argument(std::string(moved) + " moved in joints.json v2: PIDF and MAXMotion go in "
          "slots.<n>, the slot in control.slot (docs/PARAMETROS.md)");
  only_keys(j, {"can_id", "gear_ratio", "direction", "zero_offset_rad", "min_position_rad",
      "max_position_rad", "max_velocity_rad_s", "spark", "control", "slots"});
  JointConfig joint;
  joint.name = name;
  joint.device_id = integer(j, "can_id");
  joint.gear_ratio = number(j, "gear_ratio");
  joint.direction = integer(j, "direction");
  joint.zero_offset_rad = number(j, "zero_offset_rad");
  joint.min_position_rad = number(j, "min_position_rad");
  joint.max_position_rad = number(j, "max_position_rad");
  joint.max_velocity_rad_s = number(j, "max_velocity_rad_s");
  const auto &spark = object_field(j, "spark");
  in("spark", [&] {
    only_keys(spark, {"motor_type", "idle_mode", "current_limit_a"});
    if (field(spark, "motor_type") != "brushless")
      throw std::invalid_argument("motor_type must be \"brushless\": MoveMaster drives NEO motors, "
          "which brushed mode can damage");
    joint.spark.idle_mode = choice<IdleMode>(spark, "idle_mode",
        {{"coast", IdleMode::kCoast}, {"brake", IdleMode::kBrake}});
    joint.spark.current_limit_a = integer(spark, "current_limit_a");
  });
  const auto &control = object_field(j, "control");
  in("control", [&] {
    only_keys(control, {"mode", "slot", "max_following_error_rad"});
    joint.mode = choice<ControlMode>(control, "mode",
        {{"position", ControlMode::kPosition}, {"maxmotion", ControlMode::kMAXMotionPosition}});
    joint.slot = integer(control, "slot");
    joint.max_following_error_rad = number(control, "max_following_error_rad");
  });
  for (const auto &item : object_field(j, "slots").items()) {
    const auto &key = item.key();
    if (key.size() != 1 || key[0] < '0' || key[0] > '3')
      throw std::invalid_argument("slots are indexed \"0\" to \"3\", not \"" + key + "\"");
    if (!item.value().is_object()) throw std::invalid_argument("slots." + key + " must be an object");
    in("slots." + key, [&] { joint.slots.emplace(key[0] - '0', load_slot(item.value())); });
  }
  return joint;
}
}  // namespace

DriverConfig load_driver_config(const std::filesystem::path &config_path,
    const std::filesystem::path &spec_path, const std::string &channel,
    const std::vector<std::string> &joint_order) {
  std::ifstream file(config_path);
  if (!file) throw std::runtime_error("Cannot open joint configuration: " + config_path.string());
  Json data;
  file >> data;
  try {
    if (!data.is_object()) throw std::invalid_argument("the file must hold a JSON object");
    only_keys(data, {"period_s", "status_period_ms", "feedback_timeout_s", "response_timeout_s",
        "disable_settle_s", "max_cycle_gap_s", "parameter_layout", "joints"});
  } catch (const std::exception &e) {
    throw std::invalid_argument("joints.json: " + std::string(e.what()));
  }
  const auto &joints = field(data, "joints");
  if (!joints.is_object()) throw std::invalid_argument("joints must be an object indexed by joint name");
  DriverConfig config;
  config.spec_path = spec_path;
  config.channel = channel;
  config.period_s = data.value("period_s", config.period_s);
  config.feedback_timeout_s = data.value("feedback_timeout_s", config.feedback_timeout_s);
  config.response_timeout_s = data.value("response_timeout_s", config.response_timeout_s);
  config.disable_settle_s = data.value("disable_settle_s", config.disable_settle_s);
  config.max_cycle_gap_s = data.value("max_cycle_gap_s", config.max_cycle_gap_s);
  config.status_period_ms = data.value("status_period_ms", config.status_period_ms);
  config.parameter_layout = data.value("parameter_layout", Json(nullptr));
  std::vector<std::string> order = joint_order;
  if (order.empty()) for (const auto &entry : joints.items()) order.push_back(entry.key());
  if (order.size() != joints.size() || std::set<std::string>(order.begin(), order.end()).size() != order.size())
    throw std::invalid_argument("Configured joints must exactly match the ros2_control joint list");
  for (const auto &name : order) {
    try {
      if (!joints.contains(name)) throw std::invalid_argument("not in joints.json");
      config.joints.push_back(load_joint(name, joints.at(name)));
    } catch (const std::exception &e) {
      throw std::invalid_argument("Configuration for " + name + ": " + e.what());
    }
  }
  return config;
}
}  // namespace movemaster
