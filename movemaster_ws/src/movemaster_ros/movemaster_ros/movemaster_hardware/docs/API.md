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
| Construcción de tramas | `parameter_write_packet`, `parameter_read_packet`, `maxmotion_setpoint_packet`. |
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
- MAXMotion Position como único modo de control de la API de alto nivel.
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

## Métodos propios del driver

`configure()`, `activate()`, `deactivate()`, `read()`, `write()`, `states()`,
`active()`, `fault()`, `radians_to_rotations()`, `rotations_to_radians()` y
`rpm_to_rad_s()`. El driver tiene un único dueño; no se llama concurrentemente
desde la HMI y `controller_manager`. La futura HMI deberá enviar sus comandos
a la capa ROS que controle estas interfaces.

`configure()` modifica parámetros en RAM y no cambia motor type, inversiones
del controlador, límites de corriente ni brake/coast. Se parte del SPARK ya
configurado para el motor utilizado en el banco probado.
