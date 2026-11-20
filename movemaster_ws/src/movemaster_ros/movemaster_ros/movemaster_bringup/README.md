# movemaster_bringup

MoveMaster con un solo comando, y al encender la Raspberry Pi sin terminal.

```bash
ros2 launch movemaster_bringup bringup.launch.py                                   # brazo y trayectorias
ros2 launch movemaster_bringup bringup.launch.py pendant:=/dev/ttyUSB0 rviz:=true  # en el banco
```

| Incluye | Cuándo |
|---|---|
| `spark_home --home`: referencia los ejes que la perdieron contra su tope mecánico y lleva el brazo a Home (`home_deg` de `dh.yaml`). **El brazo se mueve solo.** | Primero, con `homing:=true` (por defecto) y el brazo real |
| `movemaster_control`: `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller` y `jog_controller` inactivo | Siempre; con homing, solo si terminó bien, y entonces un eje sin referencia es un fallo (`require_reference:=true`) |
| `movemaster_teleop`: `jog_node`; el switch MANUAL/AUTO del pendant elige el controlador | Con `pendant:=<puerto>`, junto con `movemaster_control` |
| RViz con el modelo del brazo | Con `rviz:=true` (necesita pantalla), junto con `movemaster_control` |

Si el homing falla, no arranca nada más y el launch se cierra; la salida de
`spark_home` dice por qué. `homing:=false` arranca sin homing: cada eje toma
como referencia la postura en que se encendió. Homing y su configuración:
[MANUAL §7.3](../docs/MANUAL.md#73-spark_home-reinicios-topes-mecánicos-y-homing).

`use_mock_hardware`, `joint_config` y `can_interface` pasan a `movemaster_control`;
con `use_mock_hardware:=true` no hay homing.

`systemd/movemaster.service` levanta `can0` a 1 Mbit/s y lanza este launch al
encender, homing incluido: el brazo se mueve en cuanto enciende la Raspberry Pi.
Instalación, comandos y por qué no se reinicia solo:
[MANUAL §11.5](../docs/MANUAL.md#115-arrancar-al-encender-la-raspberry-pi).
