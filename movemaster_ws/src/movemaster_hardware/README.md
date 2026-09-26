# movemaster_hardware

Plugin **ros2_control** del Movemaster y sus dos librerías C++ propias. Es la
traducción a C++ de `main/sparkmax_json_protocol.py` (protocolo) y de
`main/teach_pendant_backend.py` (lógica de eje), extendida a 3 SPARK MAX.

```
MovemasterHardware   (plugin, hardware_interface::SystemInterface)
        │   ciclo de vida ros2_control ↔ driver, interfaces position/velocity/current
        ▼
MoveMasterDriver     (librería movemaster_driver, sin ROS)
        │   3 ejes, rad ↔ rotaciones, setup verificado, STATUS_0/2, SP + heartbeat, watchdog
        ▼
SparkMaxProtocol     (librería sparkmax_protocol, sin ROS)
        │   JSON REV → FrameSpec / SignalCodec / CANPacket / ParameterDefinition / ParameterGroup
        ▼
SocketCAN (can0)  →  3 × SPARK MAX (MAXMotion Position; PID y perfil corren en el SPARK)
```

## Archivos

| Ruta | Contenido |
|---|---|
| `include/sparkmax_protocol/can_packet.hpp` | `CANPacket` (+ conversión a `can_frame`) |
| `include/sparkmax_protocol/signal_codec.hpp` | `SpecError`, `SignalSpec`, `SignalCodec` |
| `include/sparkmax_protocol/frame_spec.hpp` | `FrameSpec`, `SparkFrameDatabase` |
| `include/sparkmax_protocol/parameter.hpp` | `DEFAULT_PARAMETER_LAYOUT`, `PARAMETER_TYPE_CODE/NAME`, `ParameterDefinition`, `ParameterGroup` |
| `include/sparkmax_protocol/sparkmax_motion_protocol.hpp` | `SparkMAXMotionProtocol` |
| `include/sparkmax_protocol/can_bus.hpp` | `CanBus` (interfaz) y `SocketCanBus` (reemplaza a python-can) |
| `include/movemaster_driver/movemaster_driver.hpp` | `MoveMasterDriver`, `DriverConfig`, `JointConfig` |
| `include/movemaster_hardware/movemaster_hardware.hpp` | `MovemasterHardware` |
| `config/spark-frames-2.1.0.json` | JSON de tramas REV (el mismo de `main/`) |
| `urdf/movemaster.ros2_control.xacro` | Bloque `<ros2_control>` de ejemplo |
| `test/` | gtest: bytes de referencia de Python y driver contra un bus simulado |

## Correspondencia Python → C++

| Python | C++ |
|---|---|
| `spark["pidf"][0]["p"]` | `spark.pidf(0)["p"]` |
| `spark["maxmotion"][0]["cruisevelocity"]` | `spark.maxmotion(0)["cruisevelocity"]` (mismos alias) |
| `spark["frames"]["STATUS_2"]` | `spark.frames["STATUS_2"]` |
| `spark["setpoint"]` | `spark.setpoint()` |
| `dict` de señales | `SignalValues` = `std::map<std::string, double>` (bool = 0/1) |
| `bytes` | `Bytes` = `std::vector<uint8_t>` |
| `dict` de `decode_parameter_write_response` | `ParameterWriteResult` |
| `describe()` → `dict` | `describe()` → `nlohmann::ordered_json` |
| `ValueError` / `KeyError` / `TimeoutError` | `std::invalid_argument` / `std::out_of_range` / `sparkmax::TimeoutError` |
| `bus` de python-can | `sparkmax::CanBus&` (`SocketCanBus` en el robot) |
| `TeachPendantBackend` (1 eje, hilo propio) | `MoveMasterDriver` (N ejes, lo temporiza ros2_control) |

Detalles preservados a propósito: `int(round(x))` de Python redondea *half to
even* y en C++ se usa `std::nearbyint` (no `std::round`); los floats viajan
como float32 IEEE; se validan `encodedMin/encodedMax`; los bits no indicados
se envían en cero; se rechazan señales big-endian al codificar. Las pruebas
comparan byte a byte con la salida de la librería Python.

## Ciclo de vida

| ros2_control | MoveMasterDriver | CAN |
|---|---|---|
| `on_init` | construye y valida la configuración; carga el JSON | — |
| `on_configure` | `open()` → `initialize()` → `wait_for_feedback()` | stop follower, encoder primario (9), factores 1.0 (112/113), sin wrapping (149), períodos STATUS (158/160), habilita STATUS_0/2, PIDF y perfil; cada paso espera y verifica su ACK |
| `on_activate` | `arm()`: SP = PV medida; comando = posición actual | — |
| `read()` | drena la recepción | STATUS_0 (corriente), STATUS_2 (posición, velocidad) |
| `write()` | `write(comandos)` | 3 × `MAXMOTION_POSITION_SETPOINT` + 1 heartbeat |
| `on_deactivate` | `disarm()` | deja de enviar heartbeat |
| `on_cleanup` / `on_error` / `on_shutdown` | destruye el driver (cierra el socket) | — |

Como en el backend Python: el heartbeat (`0x01011840`, 8 × `FF`) solo se
envía armado; el setpoint siempre va antes del heartbeat; un fallo (NACK,
timeout, trama de error, watchdog de STATUS de 300 ms) queda enclavado y
desarma. Para recuperarse, ros2_control pasa por `on_error` y un nuevo
`configure` crea un driver nuevo. La deshabilitación la decide el watchdog
del SPARK al dejar de recibir heartbeat: **no es un freno ni un paro de
emergencia**.

El período lo marca `controller_manager` (`update_rate`). Usa 50–100 Hz para
mantener el heartbeat a ≤ 20 ms como en el programa validado. Si el lazo se
detiene, se detiene el heartbeat y el SPARK se deshabilita.

## Parámetros (URDF)

`<hardware>`:

| Parámetro | Defecto | Descripción |
|---|---|---|
| `can_interface` | `can0` | Interfaz SocketCAN (el bitrate lo fija `ip link`) |
| `frames_json` | JSON del paquete | Ruta alternativa al JSON de tramas |
| `pid_slot` | `0` | Slot PIDF/MAXMotion (0–3) |
| `status_period_ms` | `20` | Período de STATUS_0 y STATUS_2 |
| `feedback_timeout` | `0.3` | Watchdog de telemetría [s] |
| `response_timeout` | `0.5` | Espera por cada ACK de configuración [s] |
| `disable_settle_time` | `0.5` | Silencio tras el último heartbeat antes de reconfigurar [s] |
| `feedback_wait_timeout` | `2.0` | Espera de STATUS recientes en `on_configure` [s] |
| `persist_parameters` | `false` | Guarda en flash tras configurar (RESULT_CODE 0; 255 = seguir esperando) |

`<joint>`: `can_id` (obligatorio), `gear_ratio` (rotaciones de motor por
rotación de articulación; negativo invierte el sentido), `offset` [rad],
`p`, `i`, `d`, `f`, `max_acceleration` [RPM/s], `cruise_velocity` [RPM],
`allowed_profile_error` [rot]. Los `min`/`max` de la `command_interface`
`position` [rad] se convierten a rotaciones y se validan como los
`position_limits` del backend. `arm()` exige P, I, D, F, aceleración y
velocidad crucero para cada eje.

Interfaces: command `position` [rad]; state `position` [rad], `velocity`
[rad/s] y opcional `current` [A] (se publica en `/dynamic_joint_states`).

Conversión: `rot_motor = (q − offset) · gear_ratio / 2π` y
`q̇ = RPM · 2π / 60 / gear_ratio`. Los factores de conversión del SPARK quedan
en 1.0, igual que en Python, así que SPARK y host trabajan en rotaciones y RPM.

## Compilar y probar en la Raspberry Pi 5 (ROS 2 Jazzy)

```bash
cd ~/movemaster/movemaster_ws
rosdep install --from-paths src --ignore-src -y   # ros2_control, nlohmann-json3-dev
colcon build --packages-select movemaster_hardware
colcon test --packages-select movemaster_hardware --event-handlers console_direct+
source install/setup.bash
```

Las pruebas no necesitan hardware ni `can0`. Para usarlo con el robot,
incluye `urdf/movemaster.ros2_control.xacro` en el URDF, ajusta
`gear_ratio`/`offset`/límites/ganancias (los valores del archivo son de
ejemplo) y lanza `controller_manager` con `joint_state_broadcaster` y
`joint_trajectory_controller` (siguiente paso).

## Notas

- `JointTrajectoryController` envía posiciones interpoladas en cada ciclo;
  MAXMotion vuelve a perfilar hacia cada nueva referencia con sus límites de
  aceleración/velocidad, así que el seguimiento tendrá algo de retardo si esos
  límites son más bajos que los de la trayectoria de MoveIt.
- El heartbeat es global (habilita todos los SPARK del bus): este plugin debe
  ser el único emisor. No ejecutes el teach pendant Python sobre `can0` a la
  vez; en la arquitectura ROS el teach pendant hablará con ros2_control.
- El driver no es thread-safe: ros2_control invoca `read`, `write` y
  las transiciones de un mismo componente de forma serializada.
- Si la versión instalada de ros2_control ofrece `on_init(HardwareComponentInterfaceParams)`, se usa;
  si no, `on_init(HardwareInfo)` (se detecta al compilar).
