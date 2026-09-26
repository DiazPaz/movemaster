# movemaster_hardware

Plugin de ros2_control `movemaster_hardware/MovemasterHardware`
(`hardware_interface::SystemInterface`) sobre `MoveMasterDriver`.

- `description/urdf/movemaster.urdf.xacro` — robot provisional de 3 articulaciones.
- `description/urdf/movemaster.ros2_control.xacro` — bloque `<ros2_control>`; argumento
  `use_mock_hardware` para usar `mock_components/GenericSystem`.
- `config/movemaster_controllers.yaml` — `update_rate: 50`, `joint_state_broadcaster`,
  `joint_trajectory_controller` y `forward_position_controller`.
- `launch/movemaster.launch.py`.

Parámetros de `<hardware>`: `can_interface`, `frames_json` (vacío = el instalado por
`sparkmax_protocol`), `status_period_ms`, `feedback_timeout_ms`, `response_timeout_ms`,
`feedback_wait_ms`, `min_tx_period_us`, `enable_status1`, `persist_parameters`,
`trip_on_error_frame`.

Parámetros de cada `<joint>` (si faltan se usa `movemaster_config.hpp`): `can_id`,
`gear_ratio`, `inverted`, `offset_rad`, `pid_slot`, `p`, `i`, `d`, `f`, `i_zone`,
`d_filter`, `output_min`, `output_max`, y los límites en
`<command_interface name="position">` (`min`/`max`).

Interfaces: command `position` [rad]; state `position` [rad], `velocity` [rad/s],
`current` [A] (opcional).

Se usa la API clásica (`on_init(HardwareInfo)`, `export_*_interfaces()`), que compila
en todas las versiones de Jazzy; las más recientes la marcan como obsoleta, por eso esas
funciones se compilan con `-Wdeprecated-declarations` desactivado.
