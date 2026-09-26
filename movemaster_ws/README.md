# movemaster_ws — pila C++ / ros2_control para los SPARK MAX

Port a C++ de `sparkmax_json_protocol.py` y `teach_pendant_backend.py`, organizado
en tres paquetes con la siguiente estructura:

```
MovemasterHardware                 [plugin C++]            src/movemaster_hardware
        │
        ▼
MoveMasterDriver                   [librería C++ propia]   src/movemaster_driver
        │
        ├── manejo de los SPARK MAX (hasta 6; configuración actual: 3)
        ├── conversiones rad ↔ rotaciones (reductor paramétrico)
        ├── lectura de estado (STATUS_0/1/2)
        └── envío de referencias (POSITION_SETPOINT + heartbeat)
        │
        ▼
SparkMaxProtocol / PositionProtocol [librería C++ propia]  src/sparkmax_protocol
        │
        ├── parser del JSON          (json.hpp)
        ├── FrameSpec                (frame_spec.hpp, + SparkFrameDatabase)
        ├── SignalCodec              (signal_codec.hpp)
        ├── CANPacket                (can_packet.hpp)
        ├── ParameterDefinition      (parameter.hpp)
        └── ParameterGroup           (parameter.hpp)
        │
        ▼
SocketCAN                          (socketcan.hpp: CanTransport / SocketCanTransport)
        │
        ▼
3 × SPARK MAX (arquitectura para 6)
```

| Paquete | Qué contiene | Depende de ROS |
|---|---|---|
| `sparkmax_protocol` | Parser JSON propio, codec de señales, tramas, catálogo de parámetros, `SparkMaxProtocol`, `MAXMotionProtocol`, **`PositionProtocol`**, transporte SocketCAN, `spec/spark-frames-2.1.0` | No |
| `movemaster_driver` | `MoveMasterDriver`, `GearTransmission`, `movemaster_config.hpp` (reductores y valores por defecto), demo de banco | No |
| `movemaster_hardware` | Plugin `movemaster_hardware/MovemasterHardware` (`SystemInterface`), URDF/xacro, controladores y launch | Sí (Jazzy) |

`movemaster_ros` (Python) no se modificó.

## Position Control (`PositionProtocol`)

`PositionProtocol` es una variante de `SparkMaxProtocol` que sólo expone el tipo de
control **Position Control** del SPARK MAX: la trama `POSITION_SETPOINT` de
`spark-frames-2.1.0`. Las posiciones de bits, tipos y escalas se leen del JSON:

| Bits | Señal | Tipo | Nota |
|---|---|---|---|
| 0–31 | `SETPOINT` | float | rotaciones del motor |
| 32–47 | `ARBITRARY_FEEDFORWARD` | int16 × 0.0009765923 | ±32 V |
| 48–49 | `PID_SLOT` | uint | 0..3 |
| 50 | `ARBITRARY_FEEDFORWARD_UNITS` | uint | 0 = V, 1 = duty cycle |
| 51–63 | `RESERVED` | uint | 0 |

Enviar `POSITION_SETPOINT` también selecciona `ControlType = Position` en el SPARK.
El PID lo ejecuta el SPARK con el grupo de parámetros `pidf` del slot elegido
(P=13, I=14, D=15, F=16 + 8·slot, igual que la librería Python). A diferencia de
MAXMotion, **no hay perfil de movimiento en el SPARK**: la trayectoria suave la
debe generar el host (p.ej. `joint_trajectory_controller`); un escalón grande de
referencia se traduce en un golpe al reductor.

`MAXMotionProtocol` conserva el port 1:1 de `SparkMAXMotionProtocol` por si se
vuelve a MAXMotion.

## Reductores (conversión paramétrica)

`GearTransmission` (`movemaster_driver/gear_transmission.hpp`):

```
gear_ratio = vueltas del motor por vuelta de la articulación   (100:1 → 100)
motor_rot  = dir · (q − offset) · gear_ratio / 2π
q          = offset + dir · motor_rot · 2π / gear_ratio
motor_rpm  = dir · q̇ · gear_ratio · 60 / 2π
```

Los reductores de los 3 SPARK MAX son variables dentro del código, en
`movemaster_driver/include/movemaster_driver/movemaster_config.hpp`:

```cpp
inline constexpr double kGearRatioJoint1 = 1.0;   // TODO: reducción real
inline constexpr double kGearRatioJoint2 = 1.0;
inline constexpr double kGearRatioJoint3 = 1.0;
```

Para varias etapas: `GearTransmission::ratioFromStages({5.0, 4.0})` = 20.
Un `<param name="gear_ratio">` en el URDF sobrescribe el valor del código.
El SPARK sigue trabajando en rotaciones/RPM del **motor** (factores de conversión
= 1), así que las ganancias PID validadas en el banco no cambian de escala.

## Compilar

Requisitos: Ubuntu 24.04 + ROS 2 Jazzy con `ros2_control` y `ros2_controllers`.

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro
cd movemaster_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sparkmax_protocol movemaster_driver movemaster_hardware
colcon test --packages-select sparkmax_protocol movemaster_driver && colcon test-result --verbose
source install/setup.bash
```

Las dos librerías no dependen de ROS; también se pueden compilar con CMake puro
(p.ej. para pruebas en la Raspberry sin ROS) incluyéndolas con `add_subdirectory()`.

## Uso

```bash
sudo ip link set can0 up type can bitrate 1000000   # el bitrate lo fija la interfaz

# Prueba de banco sin ROS: configura los 3 SPARK e imprime telemetría
ros2 run movemaster_driver movemaster_driver_demo \
  $(ros2 pkg prefix sparkmax_protocol)/share/sparkmax_protocol/spec/spark-frames-2.1.0 can0
# ... y lleva el eje 0 a 0.5 rad con rampa de 0.2 rad/s
ros2 run movemaster_driver movemaster_driver_demo <json> can0 0 0.5 0.2

# ros2_control
ros2 launch movemaster_hardware movemaster.launch.py                          # hardware real
ros2 launch movemaster_hardware movemaster.launch.py use_mock_hardware:=true  # sin CAN
```

Ciclo de vida del plugin:

| Transición | Qué hace |
|---|---|
| `on_init` | Lee URDF → `DriverConfig` (valores faltantes: `movemaster_config.hpp`). No toca el bus. |
| `on_configure` | Abre CAN, configura cada SPARK confirmando cada escritura, espera STATUS_0/STATUS_2. |
| `on_activate` | Arma: la referencia inicial es la **posición medida** (no manda a cero). |
| `read` | Copia posición [rad], velocidad [rad/s] y corriente [A]; watchdog de telemetría. |
| `write` | Recorta a límites, convierte a rotaciones del motor, envía `POSITION_SETPOINT` ×N + heartbeat. |
| `on_deactivate` | Desarma: deja de enviar heartbeat. |
| `on_cleanup` / `on_error` / `on_shutdown` | Desarma y cierra el bus. |

Configuración de cada SPARK en `on_configure` (idéntica a `teach_pendant_backend.py`):
`STOP_FOLLOWER_MODE` → sensor primario (9 = 1) → factores de posición/velocidad = 1
(112, 113) → sin wrapping (149) → periodos STATUS_0/2 (158, 160) →
`SET_STATUSES_ENABLED` (STATUS_0, 1, 2) → P, I, D, F del slot (y opcionalmente
i_zone, d_filter, output_min, output_max). Cada `PARAMETER_WRITE` se confirma con
ID, tipo, RESULT_CODE = 0 y el mismo valor de 32 bits; cualquier rechazo o timeout
enclava un fallo. `persist_parameters=true` guarda en flash al final (opcional).

## Seguridad y limitaciones

- El heartbeat (`0x01011840`, 8 × `0xFF`) es el del ejemplo validado en la Raspberry
  Pi 5, no está en el JSON y es **global al bus**: sólo debe haber un emisor. No
  ejecutar a la vez `movemaster_ros`/`teach_pendant_backend.py` y este plugin en `can0`.
- Desarmar deja de enviar heartbeat; el SPARK se deshabilita por su watchdog de
  firmware. **No es un paro de emergencia.**
- Si STATUS_0 o STATUS_2 de cualquier eje no llega en 300 ms estando armado, el
  driver se desarma y enclava el fallo (`read()`/`write()` devuelven `ERROR`).
- Los IDs `i_zone`, `d_filter`, `output_min`, `output_max` (17–20 + 8·slot) siguen la
  tabla SparkParameters de REV, pero no se han validado en el banco; sólo se escriben
  si se configuran. Verifíquelos antes de usarlos.
- Las pruebas automáticas (paridad con Python, bus simulado de 3 SPARK, sanitizers)
  no sustituyen la prueba en hardware: `SocketCanTransport` y el plugin dentro de
  `controller_manager` deben verificarse en la Raspberry con los motores sin carga.
