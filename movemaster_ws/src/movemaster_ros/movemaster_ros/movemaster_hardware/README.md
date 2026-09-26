# MoveMaster: protocolo C++, driver y plugin de hardware

Primera etapa de la arquitectura. Contiene una traducción del protocolo Python,
un driver para **1 a 6 SPARK MAX** y el plugin `MovemasterHardware` para
`ros2_control`. Se conserva **MAXMotion Position Control**.

**Objetivo del plugin: ROS 2 Jazzy, Linux, C++17.** Las librerías y los ejemplos
tienen una compilación independiente de ROS. No se incluyen todavía nodos,
launch files, modelo cinemático ni configuración de MoveIt.

## 1. Qué hay en el paquete

| Archivo | Responsabilidad |
|---|---|
| `include/movemaster_hardware/sparkmax_json_protocol.hpp` | API traducida: clases, atributos públicos y métodos del protocolo. |
| `src/sparkmax_json_protocol.cpp` | Parser JSON, codec de bits, parámetros y MAXMotion. |
| `include/movemaster_hardware/socketcan.hpp`, `src/socketcan.cpp` | Transporte SocketCAN de Linux; apertura/cierre y tramas CAN clásicas. |
| `include/movemaster_hardware/movemaster_driver.hpp`, `src/movemaster_driver.cpp` | Inicialización de los SPARK, ACKs, conversiones, estado, referencias y heartbeat. |
| `src/driver_config.cpp` | Lectura de la configuración mecánica y de control. |
| `include/movemaster_hardware/movemaster_hardware.hpp`, `src/movemaster_hardware.cpp` | Adaptación al ciclo de vida de `hardware_interface::SystemInterface`. |
| `movemaster_hardware.xml` | Registro del plugin para pluginlib. |
| `config/joints.example.json` | Plantilla de tres ejes; completar los valores `null`. |
| `config/ros2_control.xacro` | Macro para incorporar el plugin al URDF futuro. |
| `spec/spark-frames-2.1.0` | Tu archivo REV JSON, sin cambios de contenido. |
| `examples/protocol_demo.cpp` | Construcción de tramas sin abrir CAN. |
| `examples/driver_monitor.cpp` | Configuración RAM y lectura de telemetría, sin habilitación. |
| `tests/` | Comparación contra Python y pruebas del driver con transporte simulado. |

La dependencia es:

`MovemasterHardware → MoveMasterDriver → SparkMAXMotionProtocol → SocketCAN`.

El protocolo prepara e interpreta paquetes; el driver decide cuándo enviarlos.
`SparkMaxProtocol` es un alias de `SparkMAXMotionProtocol`, de modo que se puede
usar el nombre del diagrama conservando el nombre del programa original.

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

Copiar `config/joints.example.json` a una ruta propia y sustituir **todos** los
`null`. La plantilla se rechaza mientras esté incompleta. Los CAN IDs 1, 2 y 3
son ejemplos: deben coincidir con los configurados físicamente.

| Campo por eje | Significado |
|---|---|
| `can_id` | ID único del SPARK, entre 0 y 63. |
| `slot` | Slot PID/MAXMotion, 0 a 3. |
| `gear_ratio` | Vueltas del motor por vuelta de la articulación; siempre positivo. |
| `direction` | `1` o `-1`, según el sentido del encoder respecto a la articulación ROS. |
| `zero_offset_rad` | Posición articular cuando el encoder del motor indica cero. |
| `min_position_rad`, `max_position_rad` | Límites calibrados de la articulación, en radianes. |
| `pidf` | `p`, `i`, `d`, `f`, ya ajustados para ese motor/eje y firmware. |
| `maxmotion.cruise_velocity` | Velocidad máxima del **motor** en RPM. |
| `maxmotion.max_acceleration` | Aceleración del **motor** en RPM/s. |
| `maxmotion.allowed_profile_error` | Error permitido en rotaciones del **motor**. |

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
STATUS_0/2, PIDF y perfil MAXMotion. Verifica ID de respuesta, tipo, resultado y
valor confirmado. Las respuestas y la telemetría tienen un único consumidor.

En una futura aplicación C++ sin ROS, el orden de uso es:

```cpp
auto config = movemaster::load_driver_config("/ruta/joints.json", "/ruta/spark-frames-2.1.0");
movemaster::MoveMasterDriver driver(config);
driver.configure();               // Configura, no habilita.
driver.activate();                // Habilita manteniendo la posición medida.
// En cada ciclo, con la frecuencia configurada:
driver.read();
// driver.write(objetivos_en_radianes); // Un valor por eje, en el orden configurado.
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

Colocar este directorio como `~/movemaster_ws/src/movemaster_hardware`.
Con ROS 2 Jazzy instalado:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/movemaster_ws
rosdep install --from-paths src --ignore-src -r -y
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

Más adelante, dentro del URDF del robot, incluir la macro:

```xml
<xacro:include filename="$(find movemaster_hardware)/config/ros2_control.xacro"/>
<xacro:movemaster_ros2_control
    name="MoveMasterSystem"
    spec_path="$(find movemaster_hardware)/spec/spark-frames-2.1.0"
    joint_config_path="/ruta/absoluta/joints.json"
    can_interface="can0"/>
```

Los joints `joint_1..3` deben existir en ese URDF. La macro entregada es un
fragmento de hardware, no un modelo geométrico o cinemático del robot.

| Callback | Comportamiento |
|---|---|
| `on_init()` | Valida interfaces, configuración y JSON. No abre CAN. |
| `on_configure()` | Abre CAN, configura los SPARK y espera ACKs. |
| `on_activate()` | Espera feedback nuevo de todos los ejes; carga PV como SP y habilita. |
| `read()` | Recibe con presupuesto limitado; actualiza `position`, `velocity` y `current`. |
| `write()` | Valida todas las referencias; convierte radianes a rotaciones; transmite SP y heartbeat. |
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
ambos exige validar seguimiento y sincronización de articulaciones. Esta
etapa conserva tu modo probado para ensayos de hardware y objetivos
articulares. No garantiza todavía la ejecución sincronizada de una trayectoria
multieje de MoveIt, ni que los perfiles internos respeten su temporización.

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
