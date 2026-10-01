# Consola C++ para mover un SPARK en Position o MAXMotion

Archivo: `examples/spark_console.cpp` (antes `maxmotion_console.cpp`). Utiliza
las librerías del paquete; no necesita nodos ROS. Se ejecuta en Linux con
SocketCAN.

## Compilar y ejecutar

Desde la carpeta que contiene `CMakeLists.txt`:

```bash
cmake -S . -B build -DMOVEMASTER_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target spark_console
./build/spark_console spec/spark-frames-2.1.0 config/joints.json can0
```

`config/joints.json` debe contener **un solo joint**, con CAN ID real y todos
sus valores completos (ver [PARAMETROS.md](PARAMETROS.md)). Al iniciar, la
consola configura en RAM el baseline, las unidades y todos los slots del eje,
igual que `driver_monitor`; no guarda nada en flash. Arranca con el modo y el
slot del bloque `control`.

Para compilar directamente con g++ usando las librerías estáticas ya construidas:

```bash
g++ -std=c++17 -O2 -Iinclude examples/spark_console.cpp \
  build/libmovemaster_driver.a build/libsparkmax_protocol.a \
  -pthread -o build/spark_console
```

Se requiere tener los headers de `nlohmann-json3-dev` instalados, igual que para
compilar el resto del paquete.

## Uso interactivo

| Comando | Acción |
|---|---|
| `on` | Habilita con la posición medida como objetivo inicial. |
| `sp 0.5` | Envía la posición absoluta de 0.5 rotaciones del encoder del motor. |
| `sp -0.25` | Envía la posición absoluta de -0.25 rotaciones, si los límites la permiten. |
| `mode position` | Cambia a Position: el PID del slot persigue el SP sin perfil. |
| `mode maxmotion` | Cambia a MAXMotion: el SPARK perfila el movimiento con el slot. |
| `slot 1` | Cambia de slot (de preset), conservando el modo. |
| `pv` | Muestra la última posición recibida en rotaciones del motor y radianes articulares, corriente, modo, slot y estado de habilitación. |
| `off` | Deja de transmitir referencias y heartbeat. |
| `q` | Deshabilita y sale. Ctrl+C, SIGTERM o EOF también terminan la consola. |

Los setpoints son **absolutos**, no incrementos. `sp 0.5` no significa avanzar
media vuelta desde la posición actual, sino ir a la lectura 0.5 del encoder.
Tampoco son radianes: la consola convierte rotaciones a radianes para llamar al
driver, y el driver aplica la conversión inversa para construir el setpoint.
Por ejemplo, con una reducción 10:1, media vuelta de motor corresponde a 1/20
de vuelta de la articulación, además del sentido y offset configurados.

Escribe un comando por línea. Antes de `on`, un SP se rechaza y no se guarda.
Al volver a habilitar se obtiene otra posición medida; no se recupera un
objetivo anterior. `on` habilita inmediatamente, por lo que el motor puede
aplicar torque para sostener su posición.

### Modos y slots

- **Position no tiene perfil.** Un SP lejano llevaría el PID a su salida
  máxima. Por eso, en modo `position`, la consola solo acepta un SP a menos de
  `max_following_error_rad` de la posición medida; si no, lo rechaza y el eje
  sigue donde estaba. Sirve para ver la respuesta del PID a escalones pequeños
  al ajustar ganancias. Para movimientos largos usa `mode maxmotion`.
- **Cambiar de modo mantiene la posición medida.** Con el eje habilitado, el
  SP pendiente del modo anterior se descarta: así pasar de un movimiento
  MAXMotion a Position nunca produce un salto.
- **El slot se valida en el driver.** Un slot que no está en `joints.json`, o
  MAXMotion en un slot sin bloque `maxmotion`, se rechaza con `Sin cambios:` y
  el motivo; el modo y el slot anteriores siguen activos.

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
