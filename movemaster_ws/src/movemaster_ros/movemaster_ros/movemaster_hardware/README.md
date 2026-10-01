# MoveMaster: protocolo C++, driver y plugin de hardware

Versión 0.2.0: cada dato de los SPARK tiene su lugar. En flash quedan solo el
CAN ID, el tipo de motor, el idle mode y el límite de corriente, que guarda la
herramienta nueva `spark_commission`. Todo lo demás está en `joints.json`, que
pasa al formato v2 con `spark`, `control` y hasta cuatro `slots` por eje. Se
agrega el modo **Position** junto a MAXMotion, elegible por eje y en caliente.
La consola pasa a llamarse `spark_console` y suma los comandos `mode` y `slot`.
Diseño, tabla de parámetros y cómo pasar un `joints.json` anterior:
[docs/PARAMETROS.md](docs/PARAMETROS.md).

Versión 0.1.2: se añade `examples/maxmotion_console.cpp` (hoy
`spark_console.cpp`), una consola para enviar SP absolutos en rotaciones de
motor a un SPARK MAX. Instrucciones en [docs/SPARK_CONSOLE.md](docs/SPARK_CONSOLE.md).

Versión 0.1.1: corrección del enlace de las librerías. El protocolo y el driver
se compilan como librerías estáticas con PIC y se incorporan a sus consumidores;
el plugin ROS sigue siendo una librería compartida para pluginlib. Los ejemplos
y pruebas ya no cargan `libsparkmax_protocol.so` ni `libmovemaster_driver.so`
desde un overlay o una instalación anterior.

Si tienes la versión anterior, puedes actualizar únicamente `CMakeLists.txt`
para aplicar la corrección de enlace; `tests/differential_test.py` añade además
el diagnóstico del ejecutable cuando falla. Conserva tu configuración de ejes.
Compila en un directorio nuevo para evitar mezclar productos anteriores:

```bash
cmake -S . -B build_fixed -DMOVEMASTER_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build_fixed -j2
ctest --test-dir build_fixed --output-on-failure
```

Detalles del problema original y diagnóstico:
[docs/LINKING_FIX.md](docs/LINKING_FIX.md).

Primera etapa de la arquitectura. Contiene una traducción del protocolo Python,
un driver para **1 a 6 SPARK MAX** y el plugin `MovemasterHardware` para
`ros2_control`. Cada eje se controla en **MAXMotion Position** o en
**Position**, con el slot que elijas.

**Objetivo del plugin: ROS 2 Jazzy, Linux, C++17.** Las librerías y los ejemplos
tienen una compilación independiente de ROS. El nodo `controller_manager`, con
su launch y sus controladores, está en el paquete vecino `movemaster_control`.
Todavía no hay modelo cinemático ni configuración de MoveIt. La guía completa de
uso está en [../docs/MANUAL.md](../docs/MANUAL.md) y los términos en
[../docs/DICCIONARIO.md](../docs/DICCIONARIO.md).

## 1. Qué hay en el paquete

| Archivo | Responsabilidad |
|---|---|
| `include/movemaster_hardware/sparkmax_json_protocol.hpp` | API traducida: clases, atributos públicos y métodos del protocolo. |
| `src/sparkmax_json_protocol.cpp` | Parser JSON, codec de bits, parámetros y MAXMotion. |
| `include/movemaster_hardware/socketcan.hpp`, `src/socketcan.cpp` | Transporte SocketCAN de Linux; apertura/cierre y tramas CAN clásicas. |
| `include/movemaster_hardware/spark_setup.hpp`, `src/spark_setup.cpp` | Parámetros de REV fuera del catálogo Python, escrituras con ACK, baseline, slots, puesta en marcha y escucha del bus. |
| `include/movemaster_hardware/movemaster_driver.hpp`, `src/movemaster_driver.cpp` | Inicialización de los SPARK, conversiones, modos y slots, estado, referencias y heartbeat. |
| `src/driver_config.cpp` | Lectura estricta de `joints.json` v2. |
| `include/movemaster_hardware/movemaster_hardware.hpp`, `src/movemaster_hardware.cpp` | Adaptación al ciclo de vida de `hardware_interface::SystemInterface`. |
| `movemaster_hardware.xml` | Registro del plugin para pluginlib. |
| `config/joints.json`, `config/joints.example.json` | El eje del banco, y un ejemplo de tres ejes con varios slots y valores ilustrativos. |
| `config/ros2_control.xacro` | Macro que declara el bloque `<ros2_control>` a partir de `joints.json`; admite hardware simulado. |
| `spec/spark-frames-2.1.0` | Tu archivo REV JSON, sin cambios de contenido. |
| `spec/SparkParameters-v0.1.2.md` | Tabla de parámetros del SPARK (ID, tipo, fábrica), sin cambios; las pruebas comparan con ella los IDs del código. |
| `examples/protocol_demo.cpp` | Construcción de tramas sin abrir CAN. |
| `examples/driver_monitor.cpp` | Configuración RAM y lectura de telemetría, sin habilitación. |
| `examples/spark_console.cpp` | Habilitación y envío interactivo de SP para un eje, en Position o MAXMotion. |
| `examples/spark_commission.cpp` | Puesta en marcha: restablece, escribe el baseline y lo persiste. |
| `tests/` | Comparación contra Python, y pruebas del driver, la configuración y la puesta en marcha con transporte simulado. |

La dependencia es:

`MovemasterHardware → MoveMasterDriver → SparkMAXMotionProtocol → SocketCAN`.

El protocolo prepara e interpreta paquetes; el driver decide cuándo enviarlos.
`SparkMaxProtocol` es un alias de `SparkMAXMotionProtocol`, de modo que se puede
usar el nombre del diagrama conservando el nombre del programa original.

El diagrama de clases, con todos los miembros y quién es dueño de qué, está en
[docs/CLASS_DIAGRAM.md](docs/CLASS_DIAGRAM.md).

## 2. Compilar primero sin ROS ni motores

En Ubuntu/Raspberry Pi con Ubuntu, instalar las dependencias de compilación:

```bash
sudo apt update
sudo apt install build-essential cmake nlohmann-json3-dev python3
```

Desde el directorio del paquete:

```bash
cmake -S . -B build -DMOVEMASTER_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/protocol_demo spec/spark-frames-2.1.0
```

La última orden **no transmite nada**. Para CAN ID 1, slot 0 y SP de 0.5
rotaciones debe mostrar, entre otros:

```text
CANPacket(frame=MAXMOTION_POSITION_SETPOINT, id=0x02050201, dlc=8, data=00 00 00 3F 00 00 00 00)
CANPacket(frame=PARAMETER_WRITE, id=0x02053801, dlc=5, data=0D CD CC 4C 3D)
CANPacket(frame=READ_PARAMETER_12_AND_13, id=0x02053D81, dlc=8, data=<RTR/no data>)
```

La segunda línea corresponde únicamente al ejemplo de codificar P=0.05;
**ese valor no se aplica como ganancia por defecto al robot**.

## 3. La traducción del protocolo

Ejemplo equivalente al Python:

```cpp
#include "movemaster_hardware/sparkmax_json_protocol.hpp"

movemaster::SparkMAXMotionProtocol spark("/ruta/spark-frames-2.1.0", 1);

const auto &p = spark["pidf"][0]["p"];
const auto &accel = spark["maxmotion"][0]["max_acceleration"];
const auto &same_accel = spark["maxmotion"][0]["maxaccel"]; // alias original

auto gain_packet = spark.parameter_write_packet(p, 0.05);
auto read_packet = spark.parameter_read_packet(p);
auto target_packet = spark.maxmotion_setpoint_packet(0.5, 0);
auto position_packet = spark.position_setpoint_packet(0.5, 1);  // extensión C++
auto summary = spark.describe();
```

Se conservan `CANPacket`, `SpecError`, `SignalCodec`, `FrameSpec`,
`SparkFrameDatabase`, `ParameterDefinition`, `ParameterGroup` y
`SparkMAXMotionProtocol`, además de los tres catálogos de parámetros/tipos.
Los nombres, direcciones, longitudes, escalas y señales se siguen leyendo del
JSON; los IDs semánticos de parámetros permanecen en el catálogo configurable.

Los cambios de sintaxis inevitables de Python a C++, el listado de métodos y
las limitaciones están en [docs/API.md](docs/API.md).

## 4. Configurar tus ejes

Parte de `config/joints.json` (un eje) o de `config/joints.example.json` (tres
ejes, valores ilustrativos). Los CAN IDs deben coincidir con los configurados
físicamente. La referencia completa de `joints.json` v2 está en
[docs/PARAMETROS.md](docs/PARAMETROS.md#jointsjson-v2).

| Campo por eje | Significado |
|---|---|
| `can_id` | ID único del SPARK, entre 0 y 63. |
| `gear_ratio` | Vueltas del motor por vuelta de la articulación; siempre positivo. |
| `direction` | `1` o `-1`, según el sentido del encoder respecto a la articulación ROS. |
| `zero_offset_rad` | Posición articular cuando el encoder del motor indica cero. |
| `min_position_rad`, `max_position_rad` | Límites calibrados de la articulación, en radianes. |
| `max_velocity_rad_s` | Velocidad máxima de la articulación; límite del URDF y tope de toda cruise velocity. |
| `spark` | `motor_type` (`"brushless"`), `idle_mode` y `current_limit_a`: lo que la puesta en marcha guarda en flash. |
| `control` | `mode` y `slot` de arranque, y `max_following_error_rad` para el modo Position. |
| `slots."0"` a `slots."3"` | `pidf`, `output_range` opcional y `maxmotion` opcional (RPM, RPM/s, rotaciones del **motor**). |

Se usa una articulación independiente por SPARK. Para seis ejes, agregar
`joint_4`, `joint_5` y `joint_6` tanto al JSON como al bloque `ros2_control`;
el código no cambia. El orden del plugin sigue el URDF y se comprueba que sus
nombres coincidan exactamente con los del JSON.

Con `G = gear_ratio`, `d = direction` y `q0 = zero_offset_rad`:

```text
q_rad = q0 + d * motor_rotations * (2*pi) / G
motor_rotations = d * (q_rad - q0) * G / (2*pi)
velocity_rad_s = d * motor_rpm * (2*pi) / (60*G)
```

No se implementa homing. Si el encoder relativo cambia su referencia al
reiniciar, hay que restablecer la calibración antes de activar. Una transmisión
diferencial o varios ejes acoplados requieren una conversión adicional; el
modelo actual utiliza una reducción y un sentido por articulación.

Se conserva el parámetro `f` en ID `16 + 8*slot`. No se cambia de nombre ni se
reinterpretan sus unidades; deben coincidir con tu ajuste y firmware probados.

## 5. Probar el driver desde terminal

Con la interfaz SocketCAN ya configurada a la velocidad de tu banco probado:

```bash
./build/driver_monitor /ruta/spark-frames-2.1.0 /ruta/joints.json can0
```

Esta prueba escribe y confirma configuración en RAM, habilita la publicación
de STATUS_0/2 y muestra posición, velocidad y corriente. **No envía heartbeat
de habilitación ni referencias de movimiento.** No guarda parámetros en flash.

El driver reproduce del backend: salida de follower mode, encoder primario,
factores de posición/velocidad iguales a 1, wrapping deshabilitado, períodos
STATUS_0/2, PIDF y perfil MAXMotion. Además escribe el baseline y, en cada slot
configurado, el rango de salida. Verifica ID de respuesta, tipo, resultado y
valor confirmado. Las respuestas y la telemetría tienen un único consumidor.

La flash solo la escribe `spark_commission`, una vez por SPARK:

```bash
./build/spark_commission /ruta/spark-frames-2.1.0 /ruta/joints.json can0          # revisa, no transmite
./build/spark_commission /ruta/spark-frames-2.1.0 /ruta/joints.json can0 --apply  # restablece, escribe y persiste
```

En una futura aplicación C++ sin ROS, el orden de uso es:

```cpp
auto config = movemaster::load_driver_config("/ruta/joints.json", "/ruta/spark-frames-2.1.0");
movemaster::MoveMasterDriver driver(config);
driver.configure();               // Configura, no habilita.
driver.activate();                // Habilita manteniendo la posición medida.
// En cada ciclo, con la frecuencia configurada:
driver.read();
// driver.write(objetivos_en_radianes); // Un valor por eje, en el orden configurado.
// driver.set_control(0, movemaster::ControlMode::kPosition, 1); // Eje 0 en Position, slot 1.
driver.deactivate();              // Deja de transmitir heartbeat y referencias.
```

Este fragmento muestra la API; **no es un bucle de control completo**. El
plugin es quien proporciona esa integración con el ciclo de `ros2_control`.

El heartbeat se conserva exactamente: ID `0x01011840`, ocho bytes `FF`.
Es global: todos los SPARK conectados deben pertenecer al conjunto controlado,
y el backend Python u otro emisor de heartbeat debe estar cerrado.
Primero se cargan las referencias de **todos** los ejes y después se envía el
heartbeat. Al activar, las referencias iniciales son las posiciones medidas.

Desactivar o detectar un fallo deja de emitir heartbeat; el watchdog del
firmware determina cuándo se pierde la habilitación. Esto no implementa una
parada de emergencia, ni detención instantánea, ni sujeción contra gravedad.
Las protecciones físicas del banco siguen siendo necesarias.

## 6. Compilar el plugin en ROS 2 Jazzy

El paquete vive en `movemaster_ws/src/movemaster_ros/movemaster_ros/`, dentro del
paquete Python `movemaster_ros`. colcon no busca paquetes dentro de otro paquete;
`movemaster_ws/colcon_defaults.yaml` le indica esa carpeta, así que colcon debe
ejecutarse desde `movemaster_ws/`. rosdep necesita las dos rutas.
Con ROS 2 Jazzy instalado:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/movemaster/movemaster_ws
rosdep install --from-paths src src/movemaster_ros/movemaster_ros --ignore-src -r -y
colcon build --packages-select movemaster_hardware --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select movemaster_hardware
colcon test-result --verbose
```

El plugin se identifica como `movemaster_hardware/MovemasterHardware`. Utiliza
la interfaz explícita `on_init(HardwareInfo)` y `export_*_interfaces`, conservada
en Jazzy; algunas versiones recientes pueden emitir avisos de deprecación.
La documentación de las firmas está enlazada abajo. No se afirma compatibilidad
compilada con Rolling/Kilted u otras distribuciones.

Dentro del URDF del robot, incluir la macro:

```xml
<xacro:include filename="$(find movemaster_hardware)/config/ros2_control.xacro"/>
<xacro:movemaster_ros2_control
    name="MovemasterSystem"
    spec_path="$(find movemaster_hardware)/spec/spark-frames-2.1.0"
    joint_config_path="/ruta/absoluta/joints.json"
    can_interface="can0"
    use_mock_hardware="false"/>
```

La macro declara las articulaciones de `joint_config_path`, en el orden del
archivo, que es justo lo que `on_init()` exige; esas articulaciones deben existir
en el URDF. Con `use_mock_hardware="true"` usa `mock_components/GenericSystem`
con las mismas interfaces, sin CAN ni motores. La macro es un fragmento de
hardware, no un modelo geométrico: `movemaster_control` incluye un URDF
provisional con las articulaciones y el launch del `controller_manager`.

| Callback | Comportamiento |
|---|---|
| `on_init()` | Valida interfaces, configuración y JSON. No abre CAN. |
| `on_configure()` | Abre CAN, configura los SPARK (baseline, unidades y slots) y espera ACKs. |
| `on_activate()` | Espera feedback nuevo de todos los ejes; carga PV como SP y habilita. |
| `read()` | Recibe con presupuesto limitado; actualiza `position`, `velocity` y `current`. |
| `write()` | Valida todas las referencias; convierte radianes a rotaciones; transmite el SP de cada eje en su modo y slot, y el heartbeat. |
| `on_deactivate()` | Deja de emitir referencias y heartbeat. |
| `on_cleanup()`, `on_shutdown()`, `on_error()` | Desactiva y libera el transporte. |

`current` está en amperes y es una interfaz adicional; **no es `effort` ni torque**.
La integración posterior del broadcaster decidirá cómo publicar ese dato.

El ciclo recomendado inicialmente es 50 Hz. No hay un hilo de heartbeat
independiente: si se detiene el ciclo de control también dejan de enviarse los
heartbeats. Un intervalo excesivo, una referencia inválida, un error de CAN o
telemetría vencida enclava un fallo; hay que limpiar y configurar de nuevo.
La recepción no bloquea en `read()`/`write()` y el presupuesto de RX es finito.
Los callbacks de configuración/activación sí pueden esperar. Esta versión usa
JSON y memoria dinámica en ejecución; es **tiempo real blando**, sin garantía
de latencia determinista.

## 7. Qué falta para trayectorias con MoveIt

MAXMotion genera perfiles dentro de cada SPARK. Un
`JointTrajectoryController` también interpola referencias temporizadas; usar
ambos exige validar seguimiento y sincronización de articulaciones. El modo
Position deja el perfil solo al JTC, que es lo habitual con MoveIt, pero aún no
se ha probado en el robot; el driver lo protege con `max_following_error_rad`.
No se garantiza todavía la ejecución sincronizada de una trayectoria multieje
de MoveIt. Cambiar el modo desde ROS, y no solo desde `joints.json`, es un paso
pendiente.

El siguiente paso práctico es completar la configuración de **un solo eje**,
compilar en la Raspberry Pi, comprobar STATUS_0/2 sin habilitar y verificar la
conversión de unidades. Después se valida la activación manteniendo posición
y movimientos limitados en el banco, antes de integrar los controladores ROS.

## 8. Validación y fuentes

El detalle verificable está en [docs/VALIDATION.md](docs/VALIDATION.md).
Las pruebas automatizadas no sustituyen la validación con tu firmware y robot.

- Fuente principal: los tres archivos adjuntos del usuario. El protocolo Python
  de referencia incluido en `tests/reference` conserva sus bytes originales.
- [ros2_control Jazzy: escritura de componentes de hardware](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html).
- [ros2_control Jazzy: firmas de HardwareComponentInterface](https://control.ros.org/jazzy/doc/api/hardware__component__interface_8hpp_source.html).
- [Linux: documentación de SocketCAN](https://docs.kernel.org/networking/can.html).

El nombre del mantenedor de `package.xml` es un marcador para completar antes
de publicar el paquete.
