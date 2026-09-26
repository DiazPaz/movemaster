#include "movemaster_hardware/movemaster_driver.hpp"
#include <fstream>
#include <set>

namespace movemaster {
DriverConfig load_driver_config(const std::filesystem::path &config_path,
    const std::filesystem::path &spec_path, const std::string &channel,
    const std::vector<std::string> &joint_order) {
  std::ifstream file(config_path);
  if (!file) throw std::runtime_error("Cannot open joint configuration: " + config_path.string());
  Json data;
  file >> data;
  const auto &joints = data.at("joints");
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
      const auto &j = joints.at(name);
      const auto integer = [&](const char *key) {
        if (!j.at(key).is_number_integer()) throw std::invalid_argument(std::string(key) + " must be an integer");
        const auto value = j.at(key).get<std::int64_t>();
        if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
          throw std::out_of_range(std::string(key) + " overflows int");
        return static_cast<int>(value);
      };
      JointConfig joint;
      joint.name = name;
      joint.device_id = integer("can_id");
      joint.slot = integer("slot");
      joint.direction = integer("direction");
      joint.gear_ratio = j.at("gear_ratio").get<double>();
      joint.zero_offset_rad = j.at("zero_offset_rad").get<double>();
      joint.min_position_rad = j.at("min_position_rad").get<double>();
      joint.max_position_rad = j.at("max_position_rad").get<double>();
      joint.pidf = j.at("pidf");
      joint.maxmotion = j.at("maxmotion");
      config.joints.push_back(std::move(joint));
    } catch (const std::exception &e) {
      throw std::invalid_argument("Configuration for " + name + ": " + e.what());
    }
  }
  return config;
}
}  // namespace movemaster
