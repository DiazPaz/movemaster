# Consola C++ para enviar SP con MAXMotion

Archivo: `examples/maxmotion_console.cpp`. Utiliza las librerías del paquete;
no necesita nodos ROS. Se ejecuta en Linux con SocketCAN.

## Compilar y ejecutar

Desde la carpeta que contiene `CMakeLists.txt`:

```bash
cmake -S . -B build_fixed -DMOVEMASTER_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build_fixed -j2 --target maxmotion_console
./build_fixed/maxmotion_console spec/spark-frames-2.1.0 config/joints.json can0
```

`config/joints.json` debe contener **un solo joint**, con CAN ID real y todos
los valores mecánicos, PIDF y MAXMotion completados. Usa los parámetros ya
validados en tu motor. El programa los configura en RAM al iniciar, igual que
`driver_monitor`; no guarda en flash. La plantilla con `null` no es ejecutable.

Si incorporas únicamente este archivo a la versión 0.1.1, añade a tu
`CMakeLists.txt` estas dos líneas, después de crear `movemaster_driver`:

```cmake
add_executable(maxmotion_console examples/maxmotion_console.cpp)
target_link_libraries(maxmotion_console PRIVATE movemaster_driver)
```

Para compilar directamente con g++ usando las librerías estáticas ya construidas:

```bash
g++ -std=c++17 -O2 -Iinclude examples/maxmotion_console.cpp \
  build_fixed/libmovemaster_driver.a build_fixed/libsparkmax_protocol.a \
  -pthread -o build_fixed/maxmotion_console
```

Se requiere tener los headers de `nlohmann-json3-dev` instalados, igual que para
compilar el resto del paquete.

## Uso interactivo

| Comando | Acción |
|---|---|
| `on` | Habilita con la posición medida como objetivo inicial. |
| `sp 0.5` | Envía la posición absoluta de 0.5 rotaciones del encoder del motor. |
| `sp -0.25` | Envía la posición absoluta de -0.25 rotaciones, si los límites la permiten. |
| `pv` | Muestra la última posición recibida en rotaciones del motor y radianes articulares, corriente y estado de habilitación. |
| `off` | Deja de transmitir referencias y heartbeat. |
| `q` | Deshabilita y sale. Ctrl+C, SIGTERM o EOF también terminan la consola. |

Los setpoints son **absolutos**, no incrementos. `sp 0.5` no significa avanzar
media vuelta desde la posición actual, sino ir a la lectura 0.5 del encoder.
Tampoco son radianes: la consola convierte rotaciones a radianes para llamar al
driver, y el driver aplica la conversión inversa para construir MAXMotion.
Por ejemplo, con una reducción 10:1, media vuelta de motor corresponde a 1/20
de vuelta de la articulación, además del sentido y offset configurados.

Escribe un comando por línea. Antes de `on`, un SP se rechaza y no se guarda.
Al volver a habilitar se obtiene otra posición medida; no se recupera un
objetivo anterior. `on` habilita inmediatamente, por lo que el motor puede
aplicar torque para sostener su posición.

El bucle lee el teclado con `O_NONBLOCK`, procesa como máximo una línea por
ciclo y llama continuamente a `driver.read()` y `driver.write()`. Por ello el
heartbeat sigue activo mientras escribes o esperas. Se usa `period_s` del JSON
(20 ms por defecto). Un único hilo es propietario del driver y del CAN.

Una posición fuera de límites deshabilita el eje. Una entrada con sintaxis
incorrecta se rechaza conservando el último SP. Un fallo del driver termina
el programa y detiene los envíos; requiere investigar el fallo y reiniciar.
`pv` muestra la última lectura; estando deshabilitado no certifica su frescura.

## Condiciones del banco

- Interfaz CAN preparada con el bitrate utilizado en la prueba Python.
- Solo el SPARK del ensayo en el bus que recibe el heartbeat global.
- Backend Python, otra consola y cualquier otro emisor de heartbeat cerrados.
- Límites, calibración y ganancias correspondientes al montaje real.

`off`, `q`, Ctrl+C y los errores interrumpen los envíos; el firmware determina
el tiempo de deshabilitación por su watchdog. No son una parada inmediata ni
una parada de emergencia, y deshabilitar puede liberar torque de sostén.

El ejemplo es de tiempo real blando. El driver detecta pausas excesivas del
ciclo; no se garantiza latencia determinista ni sincronización ROS/MoveIt.

Fuente para la entrada no bloqueante:
[Linux read(2)](https://man7.org/linux/man-pages/man2/read.2.html).
