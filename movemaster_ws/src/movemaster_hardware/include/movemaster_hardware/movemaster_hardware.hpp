// movemaster_hardware.hpp
//
// Plugin ros2_control (hardware_interface::SystemInterface) del Movemaster.
// Traduce el ciclo de vida de ros2_control al de MoveMasterDriver:
//
//   on_init       -> lee parámetros del URDF y valida (carga el JSON)
//   on_configure  -> open() + initialize() + wait_for_feedback()   [desarmado]
//   on_activate   -> arm(): SP = PV medida, comando = posición actual
//   read/write    -> STATUS_0/2  |  MAXMOTION_POSITION_SETPOINT + heartbeat
//   on_deactivate -> disarm(): deja de enviar heartbeat
//   on_cleanup / on_shutdown / on_error -> close()
//
// Interfaces por articulación:
//   command: position [rad]
//   state:   position [rad], velocity [rad/s] y, opcional, current [A]

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#define MOVEMASTER_HARDWARE_PARAMS_API 1  // ros2_control 4.x reciente: on_init(params)
#endif
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "movemaster_driver/movemaster_driver.hpp"

namespace movemaster_hardware
{

constexpr char HW_IF_CURRENT[] = "current";

class MovemasterHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MovemasterHardware)

#ifdef MOVEMASTER_HARDWARE_PARAMS_API
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  override;
#else
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
#endif
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  CallbackReturn init_from_urdf();
  void update_states();

  rclcpp::Logger logger_{rclcpp::get_logger("MovemasterHardware")};
  movemaster::DriverConfig config_;
  double feedback_wait_timeout_s_{2.0};
  std::unique_ptr<movemaster::MoveMasterDriver> driver_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_currents_;
  std::vector<double> hw_commands_;
};

}  // namespace movemaster_hardware
