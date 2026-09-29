# Diagrama de clases

Clases declaradas en `include/movemaster_hardware/` e implementadas en `src/`.
Se organizan en cuatro capas, de ROS hacia el hardware:

| Capa | Clases | Archivos |
|---|---|---|
| 1. Adaptador ros2_control | `MovemasterHardware` | `movemaster_hardware.hpp`, `movemaster_hardware.cpp` |
| 2. Driver | `MoveMasterDriver`, `DriverConfig`, `JointConfig`, `JointState`, `load_driver_config()` | `movemaster_driver.hpp`, `movemaster_driver.cpp`, `driver_config.cpp` |
| 3. Protocolo SPARK | `SparkMAXMotionProtocol` (con la clase anidada `Access`), `SparkFrameDatabase`, `FrameSpec`, `ParameterGroup`, `ParameterDefinition`, `SignalCodec`, `CANPacket`, `CANBus`, `SpecError`, `TimeoutError` | `sparkmax_json_protocol.hpp`, `sparkmax_json_protocol.cpp` |
| 4. Transporte CAN | `SocketCAN` | `socketcan.hpp`, `socketcan.cpp` |

`CANBus` y `CANPacket` se declaran en el header del protocolo, pero son el
contrato que implementa la capa de transporte.

## Vista general

Solo nombres y relaciones.

```mermaid
classDiagram
  direction TB

  class SystemInterface["hardware_interface::SystemInterface"] {
    <<ros2_control>>
  }
  class MovemasterHardware {
    <<plugin>>
  }
  class load_driver_config["load_driver_config()"] {
    <<función libre>>
  }
  class MoveMasterDriver
  class DriverConfig {
    <<struct>>
  }
  class JointConfig {
    <<struct>>
  }
  class JointState {
    <<struct>>
  }
  class SparkMAXMotionProtocol
  class Access["SparkMAXMotionProtocol::Access"] {
    <<clase anidada>>
  }
  class SparkFrameDatabase
  class FrameSpec
  class ParameterGroup
  class ParameterDefinition {
    <<struct>>
  }
  class SignalCodec {
    <<utilidad>>
  }
  class CANPacket {
    <<struct>>
  }
  class CANBus {
    <<interfaz>>
  }
  class SocketCAN {
    <<final>>
  }
  class SimulatedSparkBus {
    <<prueba>>
  }

  SystemInterface <|-- MovemasterHardware
  MovemasterHardware *-- "0..1" MoveMasterDriver : driver_
  MovemasterHardware *-- DriverConfig : config_
  MovemasterHardware ..> load_driver_config : on_init
  load_driver_config ..> DriverConfig : crea
  MoveMasterDriver *-- DriverConfig : config_
  DriverConfig *-- "1..6" JointConfig : joints
  MoveMasterDriver *-- "1..6" JointState : states_
  MoveMasterDriver *-- "1..6" SparkMAXMotionProtocol : protocols_
  MoveMasterDriver *-- "0..1" CANBus : bus_
  SparkMAXMotionProtocol *-- "4" Access : _access
  SparkMAXMotionProtocol *-- SparkFrameDatabase : frames
  SparkMAXMotionProtocol *-- "8" ParameterGroup : _pidf y _maxmotion
  SparkMAXMotionProtocol ..> CANBus : usa CANBus&
  SparkFrameDatabase *-- "*" FrameSpec : frames
  ParameterGroup *-- "*" ParameterDefinition : _canonical
  FrameSpec ..> SignalCodec : usa
  FrameSpec ..> CANPacket : crea
  CANBus <|.. SocketCAN
  CANBus <|.. SimulatedSparkBus
```

Cómo leerlo:

- Rombo negro: composición. La clase del rombo es dueña de la otra, por valor
  o con `unique_ptr`. La etiqueta es el miembro que la guarda; `1..6` significa
  una por articulación.
- Triángulo hueco con línea continua: herencia. Con línea discontinua: implementa
  una interfaz.
- Flecha discontinua: dependencia; usa, crea o llama sin guardar la instancia.

Por claridad, esta vista omite relaciones que sí están en el diagrama completo:
el driver crea el `SocketCAN`, guarda un `CANPacket` constante (el heartbeat) y
usa `SignalCodec` para cuantizar a float32; `CANBus` transporta `CANPacket`;
`SpecError` y `TimeoutError` heredan de `std::runtime_error`.
`SimulatedSparkBus` no forma parte de `src/`: es el bus simulado de
`tests/driver_test.cpp`.

## Quién es dueño del bus

La sección 1 del [README](../README.md) resume la dependencia como
`MovemasterHardware → MoveMasterDriver → SparkMAXMotionProtocol → SocketCAN`.
En el código, el protocolo no es dueño del bus:

- `MoveMasterDriver` guarda el bus (`bus_`) y un `SparkMAXMotionProtocol` por
  articulación (`protocols_`).
- El protocolo solo arma y decodifica tramas. Recibe el bus prestado
  (`CANBus&`) en sus ayudantes síncronos; el driver solo usa
  `write_parameter()`, durante `configure()`, para esperar cada ACK.
- En `read()` y `write()` es el driver quien llama a `bus_->recv()` y
  `bus_->send()`.
- `CANBus` es el punto de inyección: el constructor del driver acepta un
  `unique_ptr<CANBus>`. Si llega vacío, `configure()` abre un `SocketCAN` con
  filtros; las pruebas inyectan `SimulatedSparkBus`.

## Diagrama completo

Atributos y métodos de las clases de `include/movemaster_hardware/`, salvo
destructores, copias eliminadas y los `begin()`, `end()` y `size()` de los
contenedores. Notación: `+` público, `-` privado, `#` protegido; subrayado,
`static`; cursiva, virtual pura. Los tipos se abrevian: sin `std::`, sin
`const&` y sin valores por defecto. Los alias y constantes del namespace
(`Json`, `Bytes`, `DEFAULT_PARAMETER_LAYOUT`, …) están en [API.md](API.md).

```mermaid
classDiagram
  direction TB

  %% Capa 1 · movemaster_hardware.hpp / .cpp
  class SystemInterface["hardware_interface::SystemInterface"] {
    <<ros2_control>>
    #HardwareInfo info_
  }
  class MovemasterHardware {
    <<plugin>>
    -DriverConfig config_
    -unique_ptr~MoveMasterDriver~ driver_
    -vector~double~ positions_
    -vector~double~ velocities_
    -vector~double~ currents_
    -vector~double~ commands_
    +on_init(HardwareInfo info) CallbackReturn
    +export_state_interfaces() vector~StateInterface~
    +export_command_interfaces() vector~CommandInterface~
    +on_configure(State) CallbackReturn
    +on_activate(State) CallbackReturn
    +on_deactivate(State) CallbackReturn
    +on_cleanup(State) CallbackReturn
    +on_shutdown(State) CallbackReturn
    +on_error(State) CallbackReturn
    +read(Time, Duration) return_type
    +write(Time, Duration) return_type
    -copy_states() void
    -stop() void
  }

  %% Capa 2 · movemaster_driver.hpp / movemaster_driver.cpp / driver_config.cpp
  class MoveMasterDriver {
    -DriverConfig config_
    -unique_ptr~CANBus~ bus_
    -vector~unique_ptr~SparkMAXMotionProtocol~~ protocols_
    -vector~JointState~ states_
    -vector~optional~time_point~~ status0_at_
    -vector~optional~time_point~~ status2_at_
    -optional~time_point~ last_tx_
    -bool configured_
    -bool active_
    -string fault_
    -const CANPacket heartbeat_
    +MoveMasterDriver(DriverConfig config, unique_ptr~CANBus~ bus)
    +configure() void
    +activate() void
    +deactivate() void
    +read() void
    +write(vector~double~ positions_rad) void
    +states() vector~JointState~
    +active() bool
    +fault() string
    +radians_to_rotations(double radians, JointConfig joint) double$
    +rotations_to_radians(double rotations, JointConfig joint) double$
    +rpm_to_rad_s(double rpm, JointConfig joint) double$
    -validate_config() void
    -ensure_healthy() void
    -trip(string message) void
    -write_checked(SparkMAXMotionProtocol protocol, ParameterDefinition parameter, Json value) void
    -exchange(SparkMAXMotionProtocol protocol, string request, string response, Json values) Json
    -receive(CANPacket packet) void
    -feedback_fresh() bool
    -wait_feedback() void
    -check_cycle_gap() void
    -checked_target(double radians, JointConfig joint) double
  }
  class DriverConfig {
    <<struct>>
    +path spec_path
    +string channel
    +double period_s
    +double feedback_timeout_s
    +double response_timeout_s
    +double disable_settle_s
    +double max_cycle_gap_s
    +int status_period_ms
    +Json parameter_layout
    +vector~JointConfig~ joints
  }
  class JointConfig {
    <<struct>>
    +string name
    +int device_id
    +int slot
    +double gear_ratio
    +int direction
    +double zero_offset_rad
    +double min_position_rad
    +double max_position_rad
    +Json pidf
    +Json maxmotion
  }
  class JointState {
    <<struct>>
    +double position
    +double velocity
    +double current
    +bool primary_heartbeat_lock
  }
  class load_driver_config["load_driver_config()"] {
    <<función libre>>
    +load_driver_config(path config_path, path spec_path, string channel, vector~string~ joint_order) DriverConfig$
  }

  %% Capa 3 · sparkmax_json_protocol.hpp / .cpp
  class SparkMAXMotionProtocol {
    <<alias SparkMaxProtocol>>
    +int device_id
    +int slot_count
    +SparkFrameDatabase frames
    +Json parameter_layout
    -Slots _pidf
    -Slots _maxmotion
    -map~string, Access~ _access
    +SparkMAXMotionProtocol(path json_path, int device_id, Json parameter_layout, int slot_count)
    +operator[](string key) Access
    +pack_parameter_value(Json value, string value_type) uint32_t$
    +unpack_parameter_value(uint32_t raw_u32, string value_type) Json$
    +parameter_write_packet(ParameterDefinition parameter, Json value) CANPacket
    +parameter_read_packet(ParameterDefinition parameter) CANPacket
    +maxmotion_setpoint_packet(double setpoint, int slot, double arbitrary_feedforward, int arbitrary_feedforward_units) CANPacket
    +decode_parameter_write_response(Bytes data) Json
    +decode_parameter_read_response(ParameterDefinition parameter, Bytes data) Json
    +_send_packet(CANBus bus, CANPacket packet) void$
    +write_parameter(CANBus bus, ParameterDefinition parameter, Json value, double timeout, bool verify) Json
    +send_setpoint(CANBus bus, double setpoint, int slot, double arbitrary_feedforward, int arbitrary_feedforward_units) CANPacket
    +configure_slot(CANBus bus, int slot, Json pidf, Json maxmotion, double timeout) Json
    +describe() Json
    -_build_group_by_slot(string group) Slots
  }
  class Access["SparkMAXMotionProtocol::Access"] {
    <<clase anidada>>
    -Slots* slots_
    -SparkFrameDatabase* frames_
    -FrameSpec* frame_
    +Access(Slots* slots)
    +Access(SparkFrameDatabase* frames)
    +Access(FrameSpec* frame)
    +operator[](int slot) ParameterGroup
    +operator[](string name) FrameSpec
    +as_frame() FrameSpec
  }
  class SparkFrameDatabase {
    +path path
    +Json raw
    +map~string, FrameSpec~ frames
    +SparkFrameDatabase(path json_path)
    +operator[](string key) FrameSpec
    +contains(string key) bool
    +frames_version() string
    +device_info() Json
    +find(string text) map~string, FrameSpec~
  }
  class FrameSpec {
    +string key
    +string section
    -Json _spec
    +FrameSpec(string key, string section, Json spec)
    +operator[](string name) Json
    +get(string name, Json fallback) Json
    +base_arb_id() uint32_t
    +length_bytes() int
    +signals() Json
    +rtr() bool
    +arbitration_id(int device_id) uint32_t
    +encode_payload(Json values, bool require_all) Bytes
    +decode_payload(Bytes data) Json
    +packet(int device_id, Json values) CANPacket
  }
  class ParameterGroup {
    -Definitions _canonical
    -map~string, string~ _aliases
    +ParameterGroup(Definitions canonical, map~string, string~ aliases)
    +_normalize(string key) string$
    +operator[](string key) ParameterDefinition
    +as_dict() Definitions
  }
  class ParameterDefinition {
    <<struct>>
    +string group
    +string key
    +int slot
    +int parameter_id
    +string value_type
    +string description
    +optional~string~ unit
    +pair_start_id() int
    +pair_index() int
    +read_frame_name() string
  }
  class SignalCodec {
    <<utilidad>>
    +_require_little_endian(Json signal_spec) void$
    +_float_to_u32(double value) uint32_t$
    +_u32_to_float(uint32_t value) double$
    +encode_bits(Json signal_spec, Json decoded_value) uint64_t$
    +decode_bits(Json signal_spec, uint64_t raw_bits) Json$
  }
  class CANPacket {
    <<struct>>
    +uint32_t arbitration_id
    +Bytes data
    +bool is_extended_id
    +bool is_remote_frame
    +optional~int~ dlc
    +optional~string~ frame_name
    +to_socketcan() can_frame
    +from_socketcan(can_frame frame) CANPacket$
    +repr() string
  }
  class CANBus {
    <<interfaz>>
    +send(CANPacket packet, double timeout) void*
    +recv(double timeout) optional~CANPacket~*
  }
  class SpecError {
    <<excepción>>
  }
  class TimeoutError {
    <<excepción>>
  }
  class runtime_error["std::runtime_error"]

  %% Capa 4 · socketcan.hpp / .cpp
  class SocketCAN {
    <<final>>
    -int fd_
    +SocketCAN(string channel)
    +set_filters(vector~uint32_t~ extended_ids) void
    +send(CANPacket packet, double timeout) void
    +recv(double timeout) optional~CANPacket~
    -ready(short events, double timeout) bool
  }

  %% Relaciones
  SystemInterface <|-- MovemasterHardware
  MovemasterHardware *-- "0..1" MoveMasterDriver : driver_
  MovemasterHardware *-- DriverConfig : config_
  MovemasterHardware ..> load_driver_config : on_init
  load_driver_config ..> DriverConfig : crea
  MoveMasterDriver *-- DriverConfig : config_
  DriverConfig *-- "1..6" JointConfig : joints
  MoveMasterDriver *-- "1..6" JointState : states_
  MoveMasterDriver *-- "1..6" SparkMAXMotionProtocol : protocols_
  MoveMasterDriver *-- "0..1" CANBus : bus_
  MoveMasterDriver ..> SocketCAN : crea en configure()
  MoveMasterDriver *-- CANPacket : heartbeat_
  MoveMasterDriver ..> SignalCodec : float32
  SparkMAXMotionProtocol *-- SparkFrameDatabase : frames
  SparkMAXMotionProtocol *-- "8" ParameterGroup : _pidf y _maxmotion
  SparkMAXMotionProtocol *-- "4" Access : _access
  SparkMAXMotionProtocol ..> CANBus : usa CANBus&
  SparkFrameDatabase *-- "*" FrameSpec : frames
  ParameterGroup *-- "*" ParameterDefinition : _canonical
  FrameSpec ..> SignalCodec : usa
  FrameSpec ..> CANPacket : crea
  CANBus ..> CANPacket : send y recv
  CANBus <|.. SocketCAN
  runtime_error <|-- SpecError
  runtime_error <|-- TimeoutError
```

Lo que hace cada callback de ros2_control está en la tabla de la sección 6 del
[README](../README.md).

Ambos diagramas se comprobaron con Mermaid 10.9 y 11.17. GitHub los dibuja
directamente; en otro visor, pega el bloque en <https://mermaid.live>.
