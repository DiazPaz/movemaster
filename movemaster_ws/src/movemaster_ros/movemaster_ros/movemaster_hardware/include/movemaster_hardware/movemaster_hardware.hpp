#pragma once
#include "movemaster_hardware/movemaster_driver.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <chrono>

namespace movemaster {
class MovemasterHardware final : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MovemasterHardware)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
 private:
  DriverConfig config_;
  std::unique_ptr<MoveMasterDriver> driver_;
  std::vector<double> positions_, velocities_, currents_, commands_;
  // The gripper GPIO, gripper/opening: 0 closed to 1 open; NaN while there is no command.
  bool has_gripper_ = false;
  double gripper_opening_ = std::numeric_limits<double>::quiet_NaN();
  double gripper_command_ = std::numeric_limits<double>::quiet_NaN();
  // require_reference (the bringup sets it after spark_home --home): a joint without reference is
  // a fault, from the first configure on. Without it, the first configure takes the pose each
  // joint powered up in as its reference (zero_offset_rad there); a restart found by a later one
  // means the position was lost.
  bool require_reference_ = false;
  bool reference_taken_ = false;
  // Cycles the driver cut short because the CAN TX queue was full, as last warned about.
  long tx_queue_full_warned_ = 0;
  std::chrono::steady_clock::time_point tx_queue_full_warned_at_{};
  void warn_tx_queue_full();
  void copy_states();
  void stop() noexcept;
};
}  // namespace movemaster
