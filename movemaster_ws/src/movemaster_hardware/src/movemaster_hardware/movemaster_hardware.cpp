#include "movemaster_hardware/movemaster_hardware.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace movemaster_hardware
{

namespace
{

using Params = std::unordered_map<std::string, std::string>;
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

bool has(const Params & params, const std::string & key)
{
  const auto it = params.find(key);
  return it != params.end() && !it->second.empty();
}

std::string text(const Params & params, const std::string & key, const std::string & fallback)
{
  return has(params, key) ? params.at(key) : fallback;
}

double number(const Params & params, const std::string & key)
{
  if (!has(params, key)) {
    throw std::invalid_argument("Falta el parámetro '" + key + "'");
  }
  try {
    return std::stod(params.at(key));
  } catch (const std::exception &) {
    throw std::invalid_argument("Parámetro '" + key + "' no numérico: " + params.at(key));
  }
}

double number(const Params & params, const std::string & key, double fallback)
{
  return has(params, key) ? number(params, key) : fallback;
}

bool boolean(const Params & params, const std::string & key, bool fallback)
{
  if (!has(params, key)) {return fallback;}
  const auto & value = params.at(key);
  return value == "true" || value == "True" || value == "1";
}

/// Copia los parámetros presentes en `keys` (p. ej. p, i, d, f) al mapa.
std::map<std::string, double> numbers(const Params & params, std::initializer_list<const char *> keys)
{
  std::map<std::string, double> values;
  for (const char * key : keys) {
    if (has(params, key)) {values[key] = number(params, key);}
  }
  return values;
}

}  // namespace

// -----------------------------------------------------------------------------
// on_init: URDF -> DriverConfig
// -----------------------------------------------------------------------------

#ifdef MOVEMASTER_HARDWARE_PARAMS_API
hardware_interface::CallbackReturn MovemasterHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return init_from_urdf();
}
#else
hardware_interface::CallbackReturn MovemasterHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return init_from_urdf();
}
#endif

hardware_interface::CallbackReturn MovemasterHardware::init_from_urdf()
{
  try {
    const auto & hw = info_.hardware_parameters;
    config_ = movemaster::DriverConfig{};
    config_.channel = text(hw, "can_interface", "can0");
    config_.spec_path = text(hw, "frames_json",
        ament_index_cpp::get_package_share_directory("movemaster_hardware") +
        "/config/spark-frames-2.1.0.json");
    config_.slot = static_cast<int>(number(hw, "pid_slot", 0));
    config_.status_period_ms = static_cast<int>(number(hw, "status_period_ms", 20));
    config_.feedback_timeout_s = number(hw, "feedback_timeout", 0.300);
    config_.response_timeout_s = number(hw, "response_timeout", 0.500);
    config_.disable_settle_s = number(hw, "disable_settle_time", 0.500);
    config_.persist_parameters = boolean(hw, "persist_parameters", false);
    feedback_wait_timeout_s_ = number(hw, "feedback_wait_timeout", 2.0);

    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::invalid_argument(
                "'" + joint.name + "' debe tener una única command_interface 'position'");
      }
      for (const auto & state : joint.state_interfaces) {
        if (state.name != hardware_interface::HW_IF_POSITION &&
          state.name != hardware_interface::HW_IF_VELOCITY && state.name != HW_IF_CURRENT)
        {
          throw std::invalid_argument(
                  "'" + joint.name + "': state_interface no soportada '" + state.name + "'");
        }
      }

      const auto & p = joint.parameters;
      movemaster::JointConfig j;
      j.name = joint.name;
      j.device_id = static_cast<int>(number(p, "can_id"));
      j.gear_ratio = number(p, "gear_ratio", 1.0);
      j.offset = number(p, "offset", 0.0);
      j.pidf = numbers(p, {"p", "i", "d", "f"});
      j.motion_profile = numbers(p, {"max_acceleration", "cruise_velocity", "allowed_profile_error"});

      const auto & position = joint.command_interfaces[0];
      if (!position.min.empty() && !position.max.empty()) {
        j.position_limits = std::pair{std::stod(position.min), std::stod(position.max)};
      }
      config_.joints.push_back(std::move(j));
    }

    // Valida toda la configuración (y el JSON) ya en on_init.
    driver_ = std::make_unique<movemaster::MoveMasterDriver>(config_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Configuración inválida: %s", e.what());
    return CallbackReturn::ERROR;
  }

  const auto n = info_.joints.size();
  hw_positions_.assign(n, NaN);
  hw_velocities_.assign(n, NaN);
  hw_currents_.assign(n, NaN);
  hw_commands_.assign(n, NaN);

  RCLCPP_INFO(logger_, "%zu articulaciones en %s", n, config_.channel.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MovemasterHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    for (const auto & state : joint.state_interfaces) {
      double * value = state.name == hardware_interface::HW_IF_POSITION ? &hw_positions_[i] :
        state.name == hardware_interface::HW_IF_VELOCITY ? &hw_velocities_[i] : &hw_currents_[i];
      interfaces.emplace_back(joint.name, state.name, value);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MovemasterHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return interfaces;
}

// -----------------------------------------------------------------------------
// Ciclo de vida
// -----------------------------------------------------------------------------

hardware_interface::CallbackReturn MovemasterHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    // Un fallo queda enclavado en el driver: cada configuración usa uno nuevo.
    if (!driver_) {
      driver_ = std::make_unique<movemaster::MoveMasterDriver>(config_);
    }
    driver_->open();
    RCLCPP_INFO(logger_, "CAN %s abierto; configurando SPARK MAX...", config_.channel.c_str());
    driver_->initialize();
    if (!driver_->wait_for_feedback(feedback_wait_timeout_s_)) {
      throw std::runtime_error(
              driver_->fault().value_or("No se recibieron STATUS_0/STATUS_2 recientes"));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "on_configure: %s", e.what());
    driver_.reset();
    return CallbackReturn::ERROR;
  }
  update_states();
  RCLCPP_INFO(logger_, "SPARK MAX configurados (desarmados)");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  try {
    driver_->read();
    const auto targets = driver_->arm();
    update_states();
    hw_commands_ = targets;  // Mantener la PV medida: sin saltos al activar.
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "on_activate: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Ejes armados (heartbeat activo)");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (driver_) {driver_->disarm();}
  RCLCPP_INFO(logger_, "Ejes desarmados: el SPARK se deshabilita por su watchdog");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  driver_.reset();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  driver_.reset();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_error(
  const rclcpp_lifecycle::State &)
{
  if (driver_ && driver_->fault()) {
    RCLCPP_ERROR(logger_, "Fallo enclavado: %s", driver_->fault()->c_str());
  }
  driver_.reset();  // cierra el bus: deja de enviar heartbeat
  return CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Lazo de control
// -----------------------------------------------------------------------------

void MovemasterHardware::update_states()
{
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    const auto s = driver_->state(i);
    hw_positions_[i] = s.position;
    hw_velocities_[i] = s.velocity;
    hw_currents_[i] = s.current;
  }
}

hardware_interface::return_type MovemasterHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_) {
    return hardware_interface::return_type::ERROR;
  }
  driver_->read();
  update_states();
  if (driver_->fault()) {
    RCLCPP_ERROR(logger_, "read: %s", driver_->fault()->c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MovemasterHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    driver_->write(hw_commands_);
  } catch (const std::exception & e) {
    driver_->disarm();
    RCLCPP_ERROR(logger_, "write: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace movemaster_hardware

PLUGINLIB_EXPORT_CLASS(movemaster_hardware::MovemasterHardware, hardware_interface::SystemInterface)
