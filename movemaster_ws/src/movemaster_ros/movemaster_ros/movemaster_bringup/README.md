# movemaster_bringup

MoveMaster con un solo comando, y al encender la Raspberry Pi sin terminal.

```bash
ros2 launch movemaster_bringup bringup.launch.py                                   # brazo y trayectorias
ros2 launch movemaster_bringup bringup.launch.py pendant:=/dev/ttyUSB0 rviz:=true  # en el banco
```

| Incluye | Cuándo |
|---|---|
| `movemaster_control`: `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller` y `jog_controller` inactivo | Siempre |
| `movemaster_teleop`: `jog_node`; el switch MANUAL/AUTO del pendant elige el controlador | Con `pendant:=<puerto>` |
| RViz con el modelo del brazo | Con `rviz:=true` (necesita pantalla) |

`use_mock_hardware`, `joint_config` y `can_interface` pasan a `movemaster_control`.

`systemd/movemaster.service` levanta `can0` a 1 Mbit/s y lanza este launch al
encender. Instalación, comandos y por qué no se reinicia solo:
[MANUAL §11.5](../docs/MANUAL.md#115-arrancar-al-encender-la-raspberry-pi).
