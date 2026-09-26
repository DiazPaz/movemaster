#include "movemaster_hardware/movemaster_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <limits>
#include <optional>
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
constexpr const char * kCurrentInterface = "current";
using Params = std::unordered_map<std::string, std::string>;

std::optional<std::string> param(const Params & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end()) {return std::nullopt;}
  return it->second;
}

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return "";}
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

// std::stod depende del locale; from_chars no.
double parseDouble(const std::string & raw, const std::string & key)
{
  const std::string text = trim(raw);
  double value = 0.0;
  const char * first = text.data();
  const char * last = text.data() + text.size();
  if (first != last && *first == '+') {++first;}
  const auto result = std::from_chars(first, last, value);
  if (text.empty() || result.ec != std::errc() || result.ptr != last) {
    throw std::invalid_argument("parámetro '" + key + "': número inválido '" + raw + "'");
  }
  return value;
}

long parseInteger(const std::string & raw, const std::string & key)
{
  const double value = parseDouble(raw, key);
  if (std::trunc(value) != value || std::fabs(value) > 1e9) {
    throw std::invalid_argument("parámetro '" + key + "': se esperaba un entero");
  }
  return static_cast<long>(value);
}

bool parseBool(const std::string & raw, const std::string & key)
{
  std::string text = trim(raw);
  std::transform(text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (text == "true" || text == "1" || text == "yes" || text == "on") {return true;}
  if (text == "false" || text == "0" || text == "no" || text == "off") {return false;}
  throw std::invalid_argument("parámetro '" + key + "': booleano inválido '" + raw + "'");
}

template<typename T, typename Parser>
T paramOr(const Params & params, const std::string & key, T fallback, Parser parser)
{
  const auto value = param(params, key);
  return value ? static_cast<T>(parser(*value, key)) : fallback;
}

std::optional<double> optionalDouble(const Params & params, const std::string & key)
{
  const auto value = param(params, key);
  if (!value) {return std::nullopt;}
  return parseDouble(*value, key);
}

std::string defaultFramesPath()
{
  return ament_index_cpp::get_package_share_directory("sparkmax_protocol") +
         "/spec/spark-frames-2.1.0";
}
}  // namespace

// -----------------------------------------------------------------------------
// on_init: URDF → DriverConfig (sin tocar el hardware)
// -----------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

hardware_interface::CallbackReturn MovemasterHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  namespace cfg = movemaster_driver::config;
  const auto & hw = info_.hardware_parameters;
  try {
    driver_config_ = movemaster_driver::DriverConfig{};
    driver_config_.can_interface = param(hw, "can_interface").value_or(cfg::kCanInterface);
    driver_config_.frames_json_path = param(hw, "frames_json").value_or("");
    if (trim(driver_config_.frames_json_path).empty()) {
      driver_config_.frames_json_path = defaultFramesPath();
    }
    driver_config_.status_period_ms =
      paramOr<int>(hw, "status_period_ms", cfg::kStatusPeriodMs, parseInteger);
    driver_config_.feedback_timeout = std::chrono::milliseconds(
      paramOr<long>(hw, "feedback_timeout_ms", 300, parseInteger));
    driver_config_.response_timeout = std::chrono::milliseconds(
      paramOr<long>(hw, "response_timeout_ms", 500, parseInteger));
    driver_config_.min_tx_period = std::chrono::microseconds(
      paramOr<long>(hw, "min_tx_period_us", 0, parseInteger));
    driver_config_.enable_status1 = paramOr<bool>(hw, "enable_status1", true, parseBool);
    driver_config_.persist_parameters =
      paramOr<bool>(hw, "persist_parameters", false, parseBool);
    driver_config_.trip_on_error_frame =
      paramOr<bool>(hw, "trip_on_error_frame", true, parseBool);
    feedback_wait_ = std::chrono::milliseconds(
      paramOr<long>(hw, "feedback_wait_ms", 2000, parseInteger));

    if (info_.joints.empty() || info_.joints.size() > cfg::kMaxJoints) {
      throw std::invalid_argument("se requieren entre 1 y " +
              std::to_string(cfg::kMaxJoints) + " articulaciones");
    }

    exports_velocity_.assign(info_.joints.size(), false);
    exports_current_.assign(info_.joints.size(), false);

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & joint = info_.joints[i];
      const auto & p = joint.parameters;
      const bool has_default = i < cfg::kJointCount;
      const std::string who = "articulación '" + joint.name + "': ";

      // Interfaces: sólo Position Control.
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::invalid_argument(who + "se requiere exactamente la command_interface "
                "'position' (Position Control)");
      }
      bool has_position_state = false;
      for (const auto & state : joint.state_interfaces) {
        if (state.name == hardware_interface::HW_IF_POSITION) {
          has_position_state = true;
        } else if (state.name == hardware_interface::HW_IF_VELOCITY) {
          exports_velocity_[i] = true;
        } else if (state.name == kCurrentInterface) {
          exports_current_[i] = true;
        } else {
          throw std::invalid_argument(who + "state_interface no soportada '" + state.name +
                  "' (position, velocity, current)");
        }
      }
      if (!has_position_state) {
        throw std::invalid_argument(who + "falta la state_interface 'position'");
      }

      movemaster_driver::JointConfig jc;
      jc.name = joint.name;

      const auto can_id = param(p, "can_id");
      if (!can_id && !has_default) {throw std::invalid_argument(who + "falta 'can_id'");}
      const long id = can_id ? parseInteger(*can_id, "can_id") : cfg::kCanIds[i];
      if (id < 1 || id > 62) {throw std::invalid_argument(who + "can_id debe estar entre 1 y 62");}
      jc.can_id = static_cast<uint8_t>(id);

      // Reductor: parámetro del URDF o valor dentro del código (movemaster_config.hpp).
      const auto gear = param(p, "gear_ratio");
      if (!gear && !has_default) {throw std::invalid_argument(who + "falta 'gear_ratio'");}
      jc.transmission.gear_ratio = gear ? parseDouble(*gear, "gear_ratio") : cfg::kGearRatios[i];
      jc.transmission.inverted = paramOr<bool>(p, "inverted",
          has_default ? cfg::kInverted[i] : false, parseBool);
      jc.transmission.offset_rad = paramOr<double>(p, "offset_rad",
          has_default ? cfg::kOffsetsRad[i] : 0.0, parseDouble);

      // Límites: <command_interface><param name="min|max">, luego <param> del joint,
      // luego movemaster_config.hpp.
      const auto & command = joint.command_interfaces[0];
      jc.min_position_rad = !command.min.empty() ? parseDouble(command.min, "min") :
        paramOr<double>(p, "min_position", has_default ? cfg::kMinPositionRad[i] :
          -std::numeric_limits<double>::infinity(), parseDouble);
      jc.max_position_rad = !command.max.empty() ? parseDouble(command.max, "max") :
        paramOr<double>(p, "max_position", has_default ? cfg::kMaxPositionRad[i] :
          std::numeric_limits<double>::infinity(), parseDouble);

      jc.pid_slot = paramOr<int>(p, "pid_slot", cfg::kPidSlot, parseInteger);
      jc.gains.p = paramOr<double>(p, "p", cfg::kDefaultP, parseDouble);
      jc.gains.i = paramOr<double>(p, "i", cfg::kDefaultI, parseDouble);
      jc.gains.d = paramOr<double>(p, "d", cfg::kDefaultD, parseDouble);
      jc.gains.f = paramOr<double>(p, "f", cfg::kDefaultF, parseDouble);
      jc.gains.i_zone = optionalDouble(p, "i_zone");
      jc.gains.d_filter = optionalDouble(p, "d_filter");
      jc.gains.output_min = optionalDouble(p, "output_min");
      jc.gains.output_max = optionalDouble(p, "output_max");

      driver_config_.joints.push_back(jc);
    }
    driver_config_.validate();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(logger_, "Configuración inválida en el URDF: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = info_.joints.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  hw_positions_.assign(n, nan);
  hw_velocities_.assign(n, nan);
  hw_currents_.assign(n, nan);
  hw_commands_.assign(n, nan);
  states_.assign(n, movemaster_driver::JointState{});
  clamp_warned_.assign(n, false);

  for (const auto & jc : driver_config_.joints) {
    RCLCPP_INFO(logger_,
      "%s: SPARK MAX ID %u, reductor %.4g:1%s, offset %.4f rad, límites [%.4f, %.4f] rad, "
      "slot %d (P=%g I=%g D=%g F=%g)",
      jc.name.c_str(), static_cast<unsigned>(jc.can_id), jc.transmission.gear_ratio,
      jc.transmission.inverted ? " invertido" : "", jc.transmission.offset_rad,
      jc.min_position_rad, jc.max_position_rad, jc.pid_slot, jc.gains.p, jc.gains.i,
      jc.gains.d, jc.gains.f);
  }
  RCLCPP_INFO(logger_, "MovemasterHardware: %zu SPARK MAX en %s (JSON: %s)", n,
    driver_config_.can_interface.c_str(), driver_config_.frames_json_path.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MovemasterHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    if (exports_velocity_[i]) {
      interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    if (exports_current_[i]) {
      interfaces.emplace_back(name, kCurrentInterface, &hw_currents_[i]);
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

#pragma GCC diagnostic pop

// -----------------------------------------------------------------------------
// Ciclo de vida
// -----------------------------------------------------------------------------

void MovemasterHardware::copyStates()
{
  driver_->readStates(states_.data(), states_.size());
  for (std::size_t i = 0; i < states_.size(); ++i) {
    const auto & s = states_[i];
    if (std::isfinite(s.position_rad)) {hw_positions_[i] = s.position_rad;}
    if (std::isfinite(s.velocity_rad_s)) {hw_velocities_[i] = s.velocity_rad_s;}
    if (std::isfinite(s.current_a)) {hw_currents_[i] = s.current_a;}
  }
}

void MovemasterHardware::shutdownDriver()
{
  if (driver_) {
    driver_->disarm();
    driver_->close();
  }
}

hardware_interface::CallbackReturn MovemasterHardware::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    driver_ = std::make_unique<movemaster_driver::MoveMasterDriver>(driver_config_);
    driver_->open();
    RCLCPP_INFO(logger_, "Configurando %zu SPARK MAX (Position Control)...",
      driver_->jointCount());
    driver_->configure();
    if (!driver_->waitForFeedback(feedback_wait_)) {
      throw movemaster_driver::DriverError(
              "no llegaron STATUS_0/STATUS_2 de todos los SPARK MAX" +
              (driver_->hasFault() ? ": " + driver_->fault() : std::string()));
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "on_configure falló: %s", error.what());
    shutdownDriver();
    driver_.reset();
    return hardware_interface::CallbackReturn::FAILURE;
  }
  copyStates();
  hw_commands_ = hw_positions_;
  fault_reported_ = false;
  RCLCPP_INFO(logger_, "SPARK MAX configurados y con telemetría");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!driver_) {return hardware_interface::CallbackReturn::ERROR;}
  try {
    driver_->arm();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "on_activate falló: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // El comando inicial es la posición medida: activar no mueve el robot.
  copyStates();
  hw_commands_ = hw_positions_;
  clamp_warned_.assign(clamp_warned_.size(), false);
  RCLCPP_INFO(logger_, "Movemaster armado (heartbeat activo)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (driver_) {driver_->disarm();}
  RCLCPP_INFO(logger_, "Movemaster desarmado (sin heartbeat; el SPARK se deshabilita por "
    "su watchdog)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  shutdownDriver();
  driver_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  shutdownDriver();
  driver_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovemasterHardware::on_error(const rclcpp_lifecycle::State &)
{
  if (driver_ && driver_->hasFault()) {
    RCLCPP_ERROR(logger_, "Fallo del Movemaster: %s", driver_->fault().c_str());
  }
  shutdownDriver();
  driver_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Lazo de control
// -----------------------------------------------------------------------------

hardware_interface::return_type MovemasterHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_) {return hardware_interface::return_type::OK;}
  copyStates();
  if (!driver_->checkWatchdog()) {
    if (!fault_reported_) {
      RCLCPP_ERROR(logger_, "Fallo enclavado: %s", driver_->fault().c_str());
      fault_reported_ = true;
    }
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MovemasterHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_) {return hardware_interface::return_type::OK;}
  if (driver_->isArmed()) {
    for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
      // Un comando NaN (ningún controlador activo) conserva la última referencia.
      if (!std::isfinite(hw_commands_[i])) {continue;}
      if (!driver_->setReference(i, hw_commands_[i]) && !clamp_warned_[i]) {
        RCLCPP_WARN(logger_, "%s: comando %.4f rad fuera de límites; se recorta",
          info_.joints[i].name.c_str(), hw_commands_[i]);
        clamp_warned_[i] = true;
      }
    }
  }
  if (!driver_->sendReferences()) {
    if (!fault_reported_) {
      RCLCPP_ERROR(logger_, "Fallo enclavado: %s", driver_->fault().c_str());
      fault_reported_ = true;
    }
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace movemaster_hardware

PLUGINLIB_EXPORT_CLASS(movemaster_hardware::MovemasterHardware, hardware_interface::SystemInterface)
