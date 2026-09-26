// movemaster_hardware.hpp
//
// MovemasterHardware: plugin hardware_interface::SystemInterface de ros2_control.
//
//   MovemasterHardware   (este plugin)       ← URDF <ros2_control>, controladores
//        │
//        ▼
//   MoveMasterDriver     (movemaster_driver) ← SPARK MAX, reductores, estado, referencias
//        │
//        ▼
//   SparkMaxProtocol / PositionProtocol (sparkmax_protocol) ← JSON de REV
//        │
//        ▼
//   SocketCAN → SPARK MAX
//
// Interfaces por articulación:
//   command: position [rad]
//   state:   position [rad], velocity [rad/s], current [A] (opcional)
//
// Ciclo de vida:
//   on_configure → abre CAN, configura cada SPARK y espera telemetría
//   on_activate  → arma manteniendo la posición medida (no manda a cero)
//   on_deactivate→ desarma (deja de enviar heartbeat)
//   on_cleanup / on_shutdown / on_error → desarma y cierra el bus

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "movemaster_driver/movemaster_driver.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace movemaster_hardware
{

class MovemasterHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MovemasterHardware)

  // Se usa la API clásica (HardwareInfo + export_*_interfaces) porque compila en
  // todas las versiones de Jazzy; en las más recientes emite avisos de obsolescencia.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state)
  override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period)
  override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period)
  override;

private:
  void copyStates();
  void shutdownDriver();

  rclcpp::Logger logger_{rclcpp::get_logger("MovemasterHardware")};
  movemaster_driver::DriverConfig driver_config_;
  std::unique_ptr<movemaster_driver::MoveMasterDriver> driver_;
  std::chrono::milliseconds feedback_wait_{2000};

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_currents_;
  std::vector<double> hw_commands_;
  std::vector<movemaster_driver::JointState> states_;
  std::vector<bool> exports_velocity_;
  std::vector<bool> exports_current_;
  std::vector<bool> clamp_warned_;
  bool fault_reported_{false};
};

}  // namespace movemaster_hardware
