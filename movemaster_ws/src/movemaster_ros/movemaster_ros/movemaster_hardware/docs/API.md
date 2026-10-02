# Correspondencia Python → C++

Namespace C++: `movemaster`. Header principal:
`movemaster_hardware/sparkmax_json_protocol.hpp`.

Se utilizan `nlohmann::ordered_json` (`Json`), `std::vector<uint8_t>` (`Bytes`),
`std::optional` y excepciones estándar. `ordered_json` mantiene el orden de las
claves del JSON y de los diccionarios suministrados para configuración.

| Python | Equivalente C++ |
|---|---|
| `CANPacket` y sus seis atributos | Struct con los mismos nombres: `arbitration_id`, `data`, `is_extended_id`, `is_remote_frame`, `dlc`, `frame_name`. |
| `CANPacket.to_python_can()` | `CANPacket.to_socketcan()`: devuelve `struct can_frame`; no existe dependencia de Python en producción. |
| `CANPacket.__repr__()` | `CANPacket::repr()`, representación informativa. |
| `SpecError` | Excepción `SpecError`, derivada de `std::runtime_error`. |
| `SignalCodec` | `_require_little_endian`, `_float_to_u32`, `_u32_to_float`, `encode_bits`, `decode_bits`. |
| `FrameSpec(key, section, spec)` | Misma construcción; `key`, `section` y `_spec` conservados; `_spec` es privado. |
| Propiedades de `FrameSpec` | Métodos `base_arb_id()`, `length_bytes()`, `signals()`, `rtr()`. |
| Métodos de `FrameSpec` | `arbitration_id`, `encode_payload`, `decode_payload`, `packet`. `get()` también disponible. |
| `SparkFrameDatabase` | `path`, `raw`, `frames`; `frames_version()`, `device_info()` y `find()`. |
| `ParameterDefinition` | Mismos campos; `pair_start_id()`, `pair_index()` y `read_frame_name()`. |
| `ParameterGroup` | `_canonical`, `_aliases`, `_normalize()`, `operator[]`, `as_dict()`. |
| `SparkMAXMotionProtocol` | `device_id`, `slot_count`, `frames`, `parameter_layout`, `_pidf`, `_maxmotion`, `_access`, `_build_group_by_slot()`. Miembros internos privados. |
| Empaquetado de parámetros | `pack_parameter_value`, `unpack_parameter_value`. |
| Construcción de tramas | `parameter_write_packet`, `parameter_read_packet`, `maxmotion_setpoint_packet`. Extensión C++: `position_setpoint_packet` y `setpoint_packet(ControlMode, ...)`, ver abajo. |
| Decodificación de respuestas | `decode_parameter_write_response`, `decode_parameter_read_response`. |
| Helpers síncronos | `_send_packet`, `write_parameter`, `send_setpoint`, `configure_slot`. Reciben `CANBus&`. |
| Introspección | `describe()` devuelve `Json`. |
| `DEFAULT_PARAMETER_LAYOUT` | Copia exacta del diccionario original, incluyendo aliases y descripciones. |
| `PARAMETER_TYPE_CODE`, `PARAMETER_TYPE_NAME` | Mismos tipos y códigos. |

## Acceso tipo diccionario

```cpp
spark["pidf"][0]["p"];
spark["maxmotion"][0]["cruisevelocity"];
spark["frames"]["MAXMOTION_POSITION_SETPOINT"];
spark["setpoint"].as_frame();
spark.frames["STATUS_2"].decode_payload(bytes);
```

`__getitem__` se convierte en `operator[]`; `__len__` en `size()`;
`__iter__` en `begin()/end()`. El recorrido de `std::map` devuelve parejas
clave/valor ordenadas por clave, como es habitual en C++; no reproduce el
orden de inserción de todos los `Mapping` Python. Esto no cambia los payloads.
Las propiedades Python pasan a getters con paréntesis. Los argumentos
nombrados pasan a argumentos posicionales con valores por defecto.

`describe()` representa los slots como claves JSON de texto, equivalente a
serializar el diccionario Python. La respuesta de `write_parameter()` contiene
`parameter` serializado como sus campos, en lugar de un objeto Python.

El protocolo no es copiable porque su fachada `_access` referencia sus propios
miembros. Puede almacenarse mediante `std::unique_ptr`, como hace el driver.
Los paquetes y definiciones sí son valores copiables. El nombre de una trama
no viaja por CAN: `from_socketcan()` no puede recuperarlo por sí solo.

## Comportamiento conservado

- ID extendido: `(base_arb_id & ~0x3F) | device_id`.
- CAN clásico de hasta ocho bytes; señales little-endian.
- `float32` IEEE-754, enteros con signo, escala y offset.
- Redondeo de enteros a empate par, como `round()` de Python.
- Señales no suministradas se codifican como cero; `require_all=true` exige todas.
- Peticiones RTR: payload vacío, DLC conservado según JSON.
- Parámetros float enviados como sus bits de 32 bits dentro del campo `VALUE`.
- Lectura por pares y selección de `FIRST_PARAMETER_VALUE`/`SECOND_PARAMETER_VALUE`.
- Cuatro slots por defecto y aliases originales de MAXMotion.
- MAXMotion Position como modo de la API original. C++ agrega Position: ver
  [Extensiones C++](#extensiones-c).
- `write_parameter()` conserva timeout, filtro de ID/parámetro, `verify`,
  `requested_value`, `parameter`, `value_matches` y tolerancias para floats.
- Los helpers síncronos consumen `recv()` y requieren propiedad exclusiva del
  transporte, igual que advertía la librería Python.

No se traslada `TeachPendantBackend` entero: sus hilos, `Command`, colas y UI
pertenecen a otra capa. Se reutilizan su secuencia de configuración, lectura de
STATUS_0/2, heartbeat y estrategia de deshabilitación dentro del driver.

## Límites explícitos de la adaptación

- `to_python_can()` se reemplaza por `to_socketcan()`; no se mantiene un método
  de nombre Python que prometa devolver un objeto de una biblioteca ausente.
- Las clases C++ usan tipos de 64 bits como máximo para bits/señales, frente a
  los enteros de precisión arbitraria de Python. Esto cubre el JSON adjunto.
- No se soportan señales big-endian, floats de ancho distinto a 32 o CAN FD.
  Se añaden verificaciones para rechazar definiciones fuera de la trama y
  evitar desplazamientos indefinidos en C++.
- Mensajes y tipos concretos de algunas excepciones cambian a los equivalentes
  C++ (`invalid_argument`, `out_of_range`, `SpecError`, `TimeoutError`).
- Los bytes de red se verifican exactamente. La equivalencia no significa
  identidad de objetos, de representación `repr`, ni cobertura de toda entrada
  arbitraria aceptada por conversiones dinámicas de Python.
- Los constructores de paquetes del protocolo no imponen límites mecánicos;
  esa validación pertenece al driver, que también rechaza valores no finitos.
- Los helpers del protocolo conservan la respuesta de errores del SPARK para
  que el llamador la inspeccione. El driver, al configurar hardware, exige
  además ACK exitoso, tipo esperado y coincidencia exacta del valor float32.

## Extensiones C++

No existen en la librería Python; su paridad se comprueba igual contra ella.

- `enum class ControlMode { kPosition, kMAXMotionPosition }` y
  `setpoint_frame_name(mode)`: el modo lo elige la trama del setpoint.
- `position_setpoint_packet(setpoint, slot, ff, units)`: mismas validaciones y
  campos que `maxmotion_setpoint_packet`, sobre `POSITION_SETPOINT`. La prueba
  diferencial la compara con `frames["POSITION_SETPOINT"].packet(...)` de Python.
- `setpoint_packet(mode, ...)`: los dos anteriores detrás de un solo método.

En `spark_setup.hpp`, fuera del protocolo:

- `rev::kMotorType`, `rev::kIdleMode`, `rev::kSmartCurrentStallLimit`,
  `rev::kSmartCurrentFreeLimit`, `rev::output_min(slot)`, `rev::output_max(slot)`
  y los parámetros de unidades y telemetría que ya usaba el driver. Su
  `description` es el nombre del parámetro en `spec/SparkParameters-v0.1.2.md`, y
  `rev::kBrushless` y `rev::kMainEncoder` son los valores de enumeración que se
  escriben. `setup_test` compara todo con esa tabla. Resumen en
  [PARAMETROS.md](PARAMETROS.md).
- `SparkSetup`: `exchange`, `write` (exige ACK con tipo y valor), `write_baseline`,
  `write_slot`, `reset_safe_parameters`, `persist_parameters` y `commission`.
  Esperan respuesta: solo para configurar, con el bus en exclusiva.
- `survey_bus`: escucha sin transmitir qué SPARK responden y si hay heartbeat.

## Métodos propios del driver

`configure()`, `activate()`, `deactivate()`, `read()`, `write()`,
`set_control()`, `controls()`, `states()`, `active()`, `fault()`,
`position_factor()`, `velocity_factor()`, `joint_to_spark()` y
`spark_to_joint()`: los dos primeros son los factores de conversión que el
driver escribe en el SPARK desde `gear_ratio`; los otros aplican el sentido y
el cero. Además, `load_driver_config()` lee `joints.json` v3 y
`validate_driver_config()` lo valida. El driver tiene un único dueño; no se llama concurrentemente desde la
HMI y `controller_manager`. La futura HMI deberá enviar sus comandos a la capa
ROS que controle estas interfaces.

`configure()` escribe en RAM el baseline (tipo de motor, idle mode y límite de
corriente), las unidades, la telemetría y todos los slots de cada eje. Nunca
persiste: la flash la escribe solo `spark_commission`. No cambia las inversiones
del controlador; el sentido lo da `direction`.
