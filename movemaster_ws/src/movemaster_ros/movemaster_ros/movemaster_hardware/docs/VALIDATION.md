# Validación realizada — 26 de septiembre de 2026

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
