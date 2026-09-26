#include "movemaster_hardware/movemaster_hardware.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <set>

namespace movemaster {
using CallbackReturn = hardware_interface::CallbackReturn;
using Return = hardware_interface::return_type;
namespace {
void report(const std::exception &e) {
  RCLCPP_ERROR(rclcpp::get_logger("MovemasterHardware"), "%s", e.what());
}
}
CallbackReturn MovemasterHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  try {
    std::vector<std::string> order;
    for (const auto &joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "position")
        throw std::invalid_argument(joint.name + ": exactly one position command interface is required");
      std::set<std::string> states;
      for (const auto &interface : joint.state_interfaces) {
        if (interface.name != "position" && interface.name != "velocity" && interface.name != "current")
          throw std::invalid_argument(joint.name + ": supported states are position, velocity, current");
        if (!states.insert(interface.name).second) throw std::invalid_argument("Duplicate state interface");
      }
      if (!states.count("position") || !states.count("velocity"))
        throw std::invalid_argument(joint.name + ": position and velocity states are required");
      order.push_back(joint.name);
    }
    if (order.empty()) throw std::invalid_argument("No joints in ros2_control");
    const auto &params = info_.hardware_parameters;
    config_ = load_driver_config(params.at("joint_config_path"), params.at("spec_path"),
        params.count("can_interface") ? params.at("can_interface") : "can0", order);
    // Parse and validate now; CAN is opened only during on_configure().
    MoveMasterDriver validated(config_);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    positions_.assign(order.size(), nan);
    velocities_.assign(order.size(), nan);
    currents_.assign(order.size(), nan);
    commands_.assign(order.size(), nan);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &e) { report(e); return CallbackReturn::ERROR; }
}
std::vector<hardware_interface::StateInterface> MovemasterHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
    for (const auto &s : info_.joints[i].state_interfaces) {
      double *value = s.name == "position" ? &positions_[i] : s.name == "velocity" ? &velocities_[i] : &currents_[i];
      result.emplace_back(info_.joints[i].name, s.name, value);
    }
  return result;
}
std::vector<hardware_interface::CommandInterface> MovemasterHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
    result.emplace_back(info_.joints[i].name, "position", &commands_[i]);
  return result;
}
CallbackReturn MovemasterHardware::on_configure(const rclcpp_lifecycle::State &) {
  try {
    driver_ = std::make_unique<MoveMasterDriver>(config_);
    driver_->configure();
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &e) { stop(); report(e); return CallbackReturn::ERROR; }
}
void MovemasterHardware::copy_states() {
  for (std::size_t i = 0; i < positions_.size(); ++i) {
    const auto &s = driver_->states()[i];
    positions_[i] = s.position;
    velocities_[i] = s.velocity;
    currents_[i] = s.current;
  }
}
CallbackReturn MovemasterHardware::on_activate(const rclcpp_lifecycle::State &) {
  try {
    if (!driver_) throw std::logic_error("Hardware not configured");
    driver_->activate();
    copy_states();
    // Seed exported command storage from measured position, never from zero.
    std::copy(positions_.begin(), positions_.end(), commands_.begin());
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &e) { stop(); report(e); return CallbackReturn::ERROR; }
}
void MovemasterHardware::stop() noexcept { if (driver_) driver_->deactivate(); }
CallbackReturn MovemasterHardware::on_deactivate(const rclcpp_lifecycle::State &) {
  stop();
  return CallbackReturn::SUCCESS;
}
CallbackReturn MovemasterHardware::on_cleanup(const rclcpp_lifecycle::State &) {
  stop(); driver_.reset(); return CallbackReturn::SUCCESS;
}
CallbackReturn MovemasterHardware::on_shutdown(const rclcpp_lifecycle::State &) {
  stop(); driver_.reset(); return CallbackReturn::SUCCESS;
}
CallbackReturn MovemasterHardware::on_error(const rclcpp_lifecycle::State &) {
  stop(); driver_.reset(); return CallbackReturn::SUCCESS;
}
Return MovemasterHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!driver_) return Return::OK;
  try { driver_->read(); copy_states(); return Return::OK; }
  catch (const std::exception &e) { stop(); report(e); return Return::ERROR; }
}
Return MovemasterHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!driver_) return Return::OK;
  try { driver_->write(commands_); return Return::OK; }
  catch (const std::exception &e) { stop(); report(e); return Return::ERROR; }
}
}  // namespace movemaster
PLUGINLIB_EXPORT_CLASS(movemaster::MovemasterHardware, hardware_interface::SystemInterface)
