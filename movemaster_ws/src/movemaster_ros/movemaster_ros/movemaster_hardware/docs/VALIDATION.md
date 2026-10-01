# Validación realizada — 26 de septiembre de 2026

## Actualización 0.2.0: parámetros por nivel, modo Position y puesta en marcha

1 de octubre de 2026. Linux x86_64, CMake 3.28, Python 3.11, sin ROS ni CAN.

| Compilación | Resultado |
|---|---|
| g++ 13.3, Debug, `-Wall -Wextra -Wpedantic -Werror` en todos los targets | Sin avisos; CTest **5/5**. |
| clang 18.1, Debug, mismas opciones | Sin avisos; CTest **5/5**. |
| g++ 13.3 con AddressSanitizer y UBSan | CTest **5/5**, sin errores de memoria ni comportamiento indefinido. |

Qué comprueba cada prueba nueva o ampliada:

- **Paridad Python/C++: 1,904/1,904 casos.** Los 63 nuevos comparan
  `position_setpoint_packet` con el `packet()` genérico de la referencia sobre
  `POSITION_SETPOINT`: CAN IDs 0, 1, 3, 6 y 63, slots 0 a 3, tres valores y tres
  entradas inválidas. La referencia Python sigue sin cambios.
- **`driver_fake_bus`.** `configure()` escribe en RAM el baseline, los factores
  en 1.0 y todos los slots (PIDF, rango de salida y MAXMotion solo donde existe),
  y nunca envía `RESET_SAFE_PARAMETERS` ni `PERSIST_PARAMETERS`. Modo y slot por
  eje desde la activación, `set_control()` en caliente sin enclavar fallos al
  rechazar, guarda de error de seguimiento sin transmitir nada, y control
  conservado al reactivar.
- **`config_and_commissioning` (nueva).** Los dos `joints.json` del repositorio
  cargan y validan. 21 errores del cargador dan su mensaje, incluido el del
  formato anterior. La puesta en marcha sigue el orden restablecer → baseline →
  persistir, con los números mágicos del spec (36292 y 15011) y 255 como "en
  curso". Un ACK fallido no llega a persistir, y un rechazo o un silencio se
  reportan. `spark_commission` revisa sin transmitir, guarda solo los ejes
  nombrados y se niega si falta un SPARK o hay heartbeat. La escucha del bus no
  transmite.
- **`console_input_and_cycle`.** `mode`, `slot`, el límite de escalón en
  Position y mantener la posición medida al cambiar de modo.

Mutaciones: se introdujeron 13 errores a propósito en una copia del paquete, y
cada uno hizo fallar al menos una prueba:

1. Position enviado como MAXMotion.
2. Guarda de error de seguimiento eliminada.
3. Límite de corriente a velocidad libre sin escribir.
4. Persistir antes del baseline.
5. Claves desconocidas aceptadas.
6. Cruise velocity sin tope.
7. Consola sin mantener posición al cambiar de modo.
8. 255 tomado como éxito.
9. Solo el slot activo escrito.
10. Rango de salida sin escribir.
11. MAXMotion en un slot sin perfil.
12. Selección de ejes de `spark_commission`.
13. Heartbeat ajeno ignorado.

La revisión del código encontró un error real en la selección de ejes de
`spark_commission`: tras guardar el último eje nombrado, los siguientes también
se habrían guardado. Se corrigió y la prueba de la herramienta lo cubre.

`movemaster_control`: pytest **6/6** con el URDF tomando `max_velocity_rad_s`, y
flake8 con las reglas de ament sin observaciones. La emulación del launch y de
`on_init` acepta los dos `joints.json` y rechaza el formato anterior con el
mensaje de migración.

**Pendiente en el equipo de destino:**

- Los IDs 2, 6, 19, 20, 59 y 60 (tipo de motor, idle mode, rango de salida y
  límite de corriente) salen de la tabla de REV, pero no se han probado con un
  SPARK real. El ACK verifica tipo y valor.
- El comportamiento real de `RESET_SAFE_PARAMETERS` y `PERSIST_PARAMETERS`: sus
  tiempos y el código 255.
- El modo Position con el motor: estabilidad del PID y el valor adecuado de
  `max_following_error_rad`.
- Compilación con colcon y carga del plugin en ROS. La interfaz del plugin no
  cambió.

## Actualización 0.1.2: consola MAXMotion de un eje

`examples/maxmotion_console.cpp` compila en C++17 como target CMake y también
con g++ directamente contra las dos librerías estáticas. La compilación directa
se verificó con `-Wall -Wextra -Wpedantic -Werror`.

CTest: **4/4 pruebas correctas**, incluida la comparación de 1,841 casos contra
Python. La prueba nueva ejecuta el mismo bucle de la consola con un driver
simulado y entrada por pipe. Comprueba:

- Rechazo de SP antes de habilitar; posición medida como objetivo inicial.
- Continuidad de las escrituras durante una pausa de 120 ms a mitad de un comando.
- Conversión de rotaciones de motor a radianes articulares con reducción y sentido.
- Comandos `on`, `sp`, `pv`, `off` y `q`.
- Deshabilitación ante una referencia fuera de límites, EOF o error del driver.
- Rechazo de números mal formados y restauración de flags del descriptor de entrada.

La consola no se probó con el SPARK físico. Estas pruebas ejercitan su entrada
y ciclo; las pruebas existentes del driver ejercitan el protocolo CAN simulado.

## Actualización 0.1.1: enlace de librerías

Se reprodujeron exactamente los dos `undefined symbol` reportados por el usuario
con la compilación compartida anterior y una librería de prueba con el mismo
SONAME pero sin las exportaciones afectadas. Esa librería se colocó al principio
de `LD_LIBRARY_PATH`. Ambos ejecutables terminaron con código 127.

Después de cambiar el protocolo y driver a `STATIC` con PIC:

- Compilación nueva en Release: correcta.
- CTest con la librería incompatible aún en `LD_LIBRARY_PATH`: **3/3 correctas**.
- Comparación contra Python: **1,841/1,841 casos coinciden**.
- `readelf -d` de `protocol_demo`, `protocol_oracle`, `driver_test` y
  `driver_monitor`: sin dependencias dinámicas de `libsparkmax_protocol.so`
  ni `libmovemaster_driver.so`.
- La prueba Python muestra ahora el stderr del oracle. Se comprobó con el
  ejecutable anterior bajo la colisión: informa `undefined symbol` y código 127.
- La solución temporal que antepone el `build/` correcto a `LD_LIBRARY_PATH`
  también permitió ejecutar el demo anterior en la reproducción.

Esto valida la corrección frente a la colisión reproducida; la ruta que carga
el equipo del usuario no se inspeccionó. La compilación/carga ROS, ARM64 y la
validación física siguen pendientes, como se detalla más abajo.

## Ejecutado en este entorno

Linux x86_64, g++ 13.3.0, C++17, CMake 4.4.3 y nlohmann/json 3.12.0.
Compilación independiente de ROS en modo Debug, con `-Wall -Wextra -Wpedantic`.
El protocolo y el driver también se compilaron inicialmente con `-Werror`.

| Prueba | Resultado |
|---|---|
| Compilación de `sparkmax_protocol` | Correcta. |
| Compilación de `movemaster_driver`, SocketCAN y configuración | Correcta. |
| Compilación de los ejemplos y herramientas de prueba | Correcta. |
| Comparación Python/C++ | **1,841 de 1,841 casos coinciden**. |
| Tramas cubiertas | **328**, todas las incluidas en el JSON. |
| Pruebas del driver con transporte simulado | Correctas, con 1, 3 y 6 ejes. |
| Ejemplo de protocolo sin abrir CAN | Correcto; SP 0.5 rot → `00 00 00 3F 00 00 00 00`. |
| Sintaxis JSON/XML | Correcta. |
| Catálogo de clases y métodos explícitos Python | Correspondencia revisada; adaptación de `to_python_can` documentada. |
| Archivo REV y Python de referencia | Iguales byte a byte a los adjuntos. |

La prueba diferencial ejecuta el módulo Python original como referencia, sin
reescribirlo, y compara sus respuestas con un ejecutable C++ independiente.
La única adaptación de transporte en la prueba reemplaza `to_python_can()` por
un paquete de prueba para poder comprobar el helper síncrono sin instalar
`python-can`. El código del codec Python permanece intacto.

Se compara exactamente el contenido de los paquetes: arbitration ID, bytes,
DLC, flags extended/RTR y nombre de trama. En resultados numéricos de
decodificación se admite tolerancia relativa/absoluta de `1e-12`. NaN e infinito
de payloads aleatorios se normalizan a `null` únicamente en el transporte JSON
de la prueba; el driver rechaza telemetría no finita.

Cobertura: codificación con valores mínimos, máximos y muestras intermedias;
decodificación de las 328 tramas; redondeo a empate par; enteros de 1 a 64 bits;
float32; escalas; slots 0..3; CAN IDs 0, 1, 3, 6 y 63; aliases; lectura por pares;
escritura y respuesta de parámetros; timeout; `verify`; catálogo alternativo;
errores de rango, longitud, slot y endian; representación nativa SocketCAN.
No constituye una demostración exhaustiva para toda entrada Python posible.

Las pruebas del driver verifican:

- Ausencia de habilitación durante configuración.
- Conversión de radianes, RPM, reducción, sentido y offset.
- Referencia inicial igual a la posición medida en cada eje.
- Referencias de todos los ejes antes del heartbeat global.
- Envío en cada ciclo aunque llegue algo antes de `period_s`, como en el lazo
  de periodo fijo del `controller_manager`, y descarte de ráfagas a menos de
  medio periodo.
- Detención de envíos estando inactivo y reactivación con feedback nuevo.
- Rechazo de un vector inválido antes de enviar cualquiera de sus referencias.
- Rechazo de IDs duplicados y configuraciones inválidas.
- Fallos de ACK por tipo, valor, resultado o falta de respuesta.
- Pérdida de telemetría de un eje, tramas malformadas y valores no finitos.
- Error de transmisión e intervalo excesivo entre ciclos.
- Enclavamiento del fallo y ausencia de rehabilitación automática.

## Pendiente en el equipo de destino

**No se compiló ni cargó el plugin con ROS 2**: este entorno no dispone de
`hardware_interface`, `pluginlib`, `rclcpp` ni `colcon`. Sus firmas se revisaron
contra la documentación oficial de Jazzy. La primera verificación pendiente
es `colcon build` y carga real mediante `controller_manager` en la Raspberry Pi.

**No se abrió un SocketCAN real ni virtual**: no hay interfaz CAN utilizable y
el entorno impide consultar/configurar enlaces de red. Se compiló el transporte
y se probaron sus conversiones a/de `struct can_frame`; el driver se ejercitó
con una implementación simulada de `CANBus`.

No se probó sobre ARM64, con los SPARK físicos, con carga mecánica, con el
watchdog real del firmware, ni con trayectorias MoveIt/JTC. Tampoco se midieron
latencias máximas, ocupación del bus o sincronización multieje. La validación
física que hizo el usuario corresponde al Python original.

## Reproducir

Desde la raíz del paquete:

```bash
cmake -S . -B build -DMOVEMASTER_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

La comparación diferencial también puede ejecutarse sola:

```bash
python3 tests/differential_test.py --oracle build/protocol_oracle
```

Hashes SHA-256 de los archivos originales utilizados:

| Archivo adjunto | SHA-256 |
|---|---|
| `spark-frames-2.1(1).0` | `bb065bcd59cf192d6a16e088aeec4c44634155716d8bc9e11f2ac302a0d64717` |
| `sparkmax_json_protocol(1).py` | `dd3fc192a99da15c13c24bc778047b48d3482ea04c20a50d7f61f7b2fa8e7190` |
| `teach_pendant_backend(1).py` | `450d3665908ab9fcf7e85d523eb0a821a568bf7cdb759ef13bcb450aa3f3d18e` |
