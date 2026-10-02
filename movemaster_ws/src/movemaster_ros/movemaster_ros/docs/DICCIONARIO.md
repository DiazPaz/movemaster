# Diccionario del nodo controller_manager de MoveMaster

Términos en orden alfabético. Cada uno dice qué es y dónde aparece en este
proyecto. Los pasos de uso están en [MANUAL.md](MANUAL.md).

[A](#a) · [B](#b) · [C](#c) · [D](#d) · [E](#e) · [F](#f) · [G](#g) · [H](#h) · [I](#i) · [J](#j) · [L](#l) · [M](#m) · [N](#n) · [O](#o) · [P](#p) · [R](#r) · [S](#s) · [T](#t) · [U](#u) · [W](#w) · [X](#x) · [Y](#y) · [Z](#z)

---

## A

**Acción (action)**
Mecanismo de ROS 2 para tareas largas: el cliente envía un objetivo, recibe
avance y al final un resultado (éxito o motivo de fallo). El JTC ofrece la
acción `/joint_trajectory_controller/follow_joint_trajectory`, que usan MoveIt y
`ros2 action send_goal`.

**ACK**
Respuesta del SPARK que confirma un parámetro escrito. En `configure()` el
driver espera el ACK de cada parámetro y comprueba resultado, tipo y valor
exacto; si no coincide, falla con `Parameter ACK mismatch`.

**activate / on_activate**
Transición del ciclo de vida que habilita el hardware. En MoveMaster: espera
`STATUS_0` y `STATUS_2` recientes de todos los ejes, toma la posición medida
como setpoint y envía el primer heartbeat. Ver *Ciclo de vida*.

**allowed_profile_error**
Campo de `slots.<n>.maxmotion` en `joints.json`: error permitido por el perfil
MAXMotion, en rotaciones del motor.

**ament / ament_cmake / ament_python**
Sistema de construcción de ROS 2. `ament_cmake` es para paquetes CMake
(`movemaster_hardware`, `movemaster_control`); `ament_python`, para paquetes
Python (`movemaster_ros`). `ament_package()` al final de un `CMakeLists.txt`
genera los archivos que ROS necesita para encontrar el paquete.

**Ament index**
Registro de archivos en `install/*/share/ament_index/` con el que ROS encuentra
paquetes y plugins sin buscar por todo el disco. `ros2 pkg prefix` y pluginlib
lo consultan.

**Arbitration ID (ID CAN)**
Identificador de 29 bits de cada trama CAN extendida. Indica el tipo de trama
y el dispositivo: `(arbId del JSON & ~0x3F) | can_id`. Ejemplo: `STATUS_2` del
SPARK 1 es `0x0205B881`.

## B

**base_link**
Primer eslabón del URDF, fijo al mundo. En `movemaster.urdf.xacro` todas las
articulaciones cuelgan de él en cadena.

**base-paths**
Opción de colcon con las carpetas donde buscar paquetes. En
`colcon_defaults.yaml` vale `[src, src/movemaster_ros/movemaster_ros]`.

**Baseline**
Lo que se guarda en la flash de cada SPARK además del CAN ID: tipo de motor,
idle mode y límite de corriente. Bloque `spark` de `joints.json`. Lo persiste
`spark_commission` y el driver lo vuelve a escribir en RAM al configurar.
[PARAMETROS.md](../movemaster_hardware/docs/PARAMETROS.md).

**Broadcaster**
Controlador que solo lee estados y los publica, sin mandar comandos. El
`JointStateBroadcaster` es uno.

**build/, install/, log/**
Carpetas que crea colcon en `movemaster_ws/`: compilación intermedia, archivos
instalados (lo que usa `ros2 run` y `ros2 launch`) y registros. Se pueden
borrar para compilar desde cero.

## C

**Ciclo de vida (lifecycle)**
Estados por los que pasa un componente de hardware o un controlador:
`unconfigured → inactive → active`, con `finalized` al apagar. Las transiciones
llaman a `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`,
`on_error`, `on_shutdown` de `MovemasterHardware`. Tabla en MANUAL §2.5.

**CallbackReturn**
Valor que devuelve cada transición del ciclo de vida: `SUCCESS`, `FAILURE` o
`ERROR`.

**CAN (Controller Area Network)**
Bus serie de dos hilos (CAN-H y CAN-L) usado por los SPARK MAX a 1 Mbit/s.
"CAN clásico" son tramas de hasta 8 bytes de datos. Necesita una resistencia de
120 Ω en cada extremo del bus.

**can0**
Nombre de la interfaz CAN en Linux. Se levanta con `ip link` (MANUAL §6) y se
elige con el argumento `can_interface`.

**can_id**
Campo de `joints.json`: CAN ID del SPARK de ese eje, 0–63, único en el bus. Se
asigna al SPARK con REV Hardware Client y queda en su flash.

**CANBus**
Interfaz C++ del plugin (`sparkmax_json_protocol.hpp`) con `send()` y `recv()`.
La implementan `SocketCAN` (real) y `SimulatedSparkBus` (pruebas).

**candump / can-utils**
`candump can0` muestra en vivo las tramas del bus. Viene en el paquete
`can-utils`.

**CANPacket**
Estructura C++ con una trama CAN: ID, datos, tipo de ID, DLC y nombre de trama.

**CMake / CMakeLists.txt**
Herramienta que genera la compilación de C++. `CMakeLists.txt` describe qué
librerías y ejecutables crear, con qué dependencias, y qué instalar. Explicado
bloque a bloque en MANUAL §13.3 y §13.4.

**colcon**
Herramienta que compila todos los paquetes de un workspace en orden de
dependencias (`colcon build`), corre sus pruebas (`colcon test`) y los lista
(`colcon list`). Se ejecuta desde `movemaster_ws/`.

**colcon_defaults.yaml**
Archivo en `movemaster_ws/` con opciones por defecto de colcon. Aquí añade
`src/movemaster_ros/movemaster_ros` a las carpetas de búsqueda, porque colcon no
entra en el paquete Python `movemaster_ros`. MANUAL §13.1.

**COLCON_IGNORE**
Archivo cuya sola existencia hace que colcon y rosdep ignoren la carpeta. Está
en `src/movemaster_hardware` (versión anterior del plugin).

**Command interface (interfaz de comando)**
Valor que un controlador escribe y el hardware ejecuta. MoveMaster exporta una
por eje: `joint_N/position`, en radianes.

**Componente de hardware**
Lo que `ros2_control` carga desde el bloque `<ros2_control>` del URDF. El de
MoveMaster se llama `MovemasterSystem` y es de tipo `system` (varios ejes en un
mismo componente).

**configure / on_configure**
Transición que prepara el hardware sin habilitarlo. En MoveMaster: abre CAN,
espera `disable_settle_s` y en cada SPARK envía `STOP_FOLLOWER_MODE`, el
baseline, los parámetros de unidades y telemetría, `SET_STATUSES_ENABLED` y
todos sus slots (PIDF, rango de salida y MAXMotion), todo con ACK. Solo en RAM.

**control (joints.json)**
Bloque de cada eje con el modo (`position` o `maxmotion`) y el slot con que se
activa, y `max_following_error_rad`. En caliente se cambian con `set_control`.

**Control Type (ID 5)**
Parámetro del SPARK con el modo de control activo. MoveMaster no lo escribe:
cada trama de setpoint lo fija.

**Controlador**
Plugin que el `controller_manager` ejecuta en cada ciclo dentro de `update()`:
lee interfaces de estado y escribe interfaces de comando. Aquí: JTC y JSB.

**controller_manager**
Nodo de `ros2_control` (`/controller_manager`, ejecutable `ros2_control_node`)
que carga el hardware y los controladores, y ejecuta el lazo
`read → update → write` a `update_rate` Hz. Es el "nodo" que arranca
`movemaster_control`.

**ctest / CTest**
Corredor de pruebas de CMake. `ctest --test-dir build` corre las cinco
pruebas del plugin. `colcon test` lo usa por debajo.

**cruise_velocity**
Campo de `slots.<n>.maxmotion` en `joints.json`: velocidad máxima del motor en
RPM en modo MAXMotion. Convertida a la articulación
(`cruise_velocity · 2π / 60 / gear_ratio` rad/s) no puede superar
`max_velocity_rad_s`.

**current**
Interfaz de estado con la corriente del motor en amperes, leída de `STATUS_0`.
Se publica en `/dynamic_joint_states`. No es par (`effort`).

## D

**depend / exec_depend / test_depend / buildtool_depend**
Tipos de dependencia en `package.xml`: para compilar y ejecutar; solo ejecutar;
solo pruebas; herramienta de compilación. MANUAL §13.2.

**direction**
Campo de `joints.json`: entero `1` o `-1`. Invierte el sentido del motor
respecto a la articulación. Debe ser entero (`1`, no `1.0`).

**disable_settle_s**
Campo global de `joints.json` (0.5 s): pausa sin transmitir antes de configurar
y antes de reactivar, para que expire un heartbeat anterior.

**DLC**
Número de bytes de datos de una trama CAN (0–8).

**Driver (MoveMasterDriver)**
Clase C++ central del plugin (`movemaster_driver.cpp`). Hace la configuración,
las conversiones de unidades, la validación de límites, los watchdogs y el
envío de setpoints y heartbeat, en el modo y slot de cada eje. No tiene hilos:
actúa cuando lo llaman.

**driver_monitor**
Ejemplo que configura los SPARK y muestra la telemetría sin habilitar los
motores. MANUAL §7.1.

**/dynamic_joint_states**
Tópico del JSB con todas las interfaces de estado de cada articulación,
incluida `current`.

## E

**effort**
Interfaz estándar de par/fuerza en ROS. MoveMaster no la tiene; el límite
`effort="0"` del URDF existe solo porque URDF lo exige.

**Enclavamiento de fallo (fault latch)**
Cuando el driver detecta un problema guarda el motivo en `fault_`, deja de
enviar heartbeat y rechaza cualquier operación posterior. Solo se sale
destruyendo el driver: `ros2_control` lo hace en `on_error`, y se vuelve con
`set_hardware_component_state … active`. MANUAL §12.

**Error de seguimiento (max_following_error_rad)**
Campo de `control`: en modo `position`, distancia máxima entre un objetivo y la
posición medida. Más lejos, el driver no envía nada y enclava un fallo. Protege
de saltos (el modo Position no tiene perfil) y detecta un eje que dejó de
seguir, por ejemplo por un choque.

**exchange**
Método de `SparkSetup`: envía una petición y espera su respuesta (por ejemplo
`STOP_FOLLOWER_MODE` → `STOP_FOLLOWER_MODE_RESPONSE`).

**Factor de conversión (position/velocity conversion factor)**
Parámetros 112 y 113 del SPARK: multiplican lo que reporta y lo que entiende de
cada setpoint. El driver los deja siempre en 1.0, así que el SPARK trabaja en
rotaciones y RPM del motor y la reducción solo está en `gear_ratio`.
[PARAMETROS.md](../movemaster_hardware/docs/PARAMETROS.md).

## F

**feedback_timeout_s**
Campo global de `joints.json` (0.3 s): edad máxima de `STATUS_0`/`STATUS_2`
estando activo. Si se supera: `STATUS_0/2 watchdog expired`.

**Firmware**
Programa interno del SPARK MAX. Las tramas de `spark-frames-2.1.0` requieren
firmware 25 o posterior.

**Flash / RAM (persistir)**
El driver escribe la configuración solo en la RAM del SPARK: se pierde al
apagarlo y se repite en cada `configure`. En flash solo quedan el CAN ID y el
baseline, que guarda `spark_commission` (`PERSIST_PARAMETERS`). [PARAMETROS.md](../movemaster_hardware/docs/PARAMETROS.md).

**FollowJointTrajectory**
Tipo de acción (`control_msgs/action/FollowJointTrajectory`) del JTC: recibe una
`JointTrajectory` y devuelve si se completó.

**FrameSpec**
Clase C++ que representa una trama del JSON de REV y sabe codificarla y
decodificarla.

## G

**gear_ratio**
Campo de `joints.json`: vueltas del motor por cada vuelta de la articulación
(reducción). Siempre positivo; el sentido lo da `direction`. Es la única
reducción del sistema: los factores de conversión del SPARK quedan en 1.0.

**GenericSystem (mock_components)**
Hardware simulado de `ros2_control`: copia el comando al estado sin física. Se
activa con `use_mock_hardware:=true`.

## H

**hardware_interface**
Paquete de `ros2_control` que define `SystemInterface`, las interfaces de
estado y comando, y `mock_components`.

**Heartbeat**
Trama global `0x01011840` con 8 bytes `FF` que el driver envía en cada ciclo
activo después de los setpoints. Mientras llega, los SPARK están habilitados;
si deja de llegar, su watchdog los deshabilita. Solo debe haber un emisor.

## I

**Idle mode (coast / brake)**
Qué hace el SPARK cuando su salida es neutra, como al deshabilitarse: `coast`
deja girar el motor libre; `brake` lo cortocircuita y frena el brazo, pero no
lo sostiene. De fábrica, `coast`. Campo `spark.idle_mode`; parte del baseline.

**info_ (HardwareInfo)**
Miembro que `MovemasterHardware` hereda de `SystemInterface`: contiene las
articulaciones, interfaces y parámetros (`spec_path`, `joint_config_path`,
`can_interface`) del bloque `<ros2_control>`.

**initial_value**
Parámetro de una interfaz de estado en el URDF. Solo lo usa el hardware
simulado como posición inicial; el plugin real lee los encoders.

**Inverted (ID 45)**
Parámetro del SPARK que invierte el sentido del motor. MoveMaster no lo usa: el
sentido lo da `direction`. La puesta en marcha lo deja en `false`, así que
conviene revisar el sentido después.

## J

**Joint (URDF)**
Articulación del URDF: une dos *links*, con tipo (`revolute`), eje, posición y
límites. En MoveMaster se generan desde `joints.json`.

**JointState / /joint_states**
Mensaje y tópico estándar con nombre, posición, velocidad y esfuerzo de cada
articulación. Lo publica el JSB; lo leen `robot_state_publisher`, RViz y
MoveIt.

**JointStateBroadcaster (JSB)**
Controlador de `ros2_controllers` que publica `/joint_states` y
`/dynamic_joint_states` a partir de las interfaces de estado.

**JointTrajectory**
Mensaje con nombres de articulaciones y una lista de puntos (posiciones y
`time_from_start`). Es lo que recibe el JTC.

**JointTrajectoryController (JTC)**
Controlador de `ros2_controllers` que recibe trayectorias, las interpola y en
cada ciclo escribe la posición deseada de cada eje. Configurado en
`movemaster_controllers.yaml`.

**joints.json**
Configuración de los ejes en `movemaster_hardware/config/`. Es la única fuente
de las articulaciones: la lee el plugin y de ella se generan el URDF, los
`joints` del JTC y el `update_rate`. Formato v2: cada eje con `spark`,
`control` y `slots`; una clave desconocida es un error. MANUAL §4.

**joint_config_path / joint_config**
Ruta a `joints.json`. `joint_config` es el argumento del launch y del xacro;
`joint_config_path`, el parámetro que recibe el plugin en el URDF.

## L

**Launch file / ros2 launch**
Programa (aquí en Python) que arranca varios nodos con sus parámetros:
`ros2 launch movemaster_control movemaster_control.launch.py`. Los argumentos
se pasan como `nombre:=valor`.

**Lazo de control (read / update / write)**
Lo que el `controller_manager` repite cada `1/update_rate` s: leer el hardware,
ejecutar los controladores y escribir al hardware. MANUAL §2.2.

**Librería estática / compartida**
Una estática (`.a`) se copia dentro del programa al enlazar; una compartida
(`.so`) se carga al ejecutar. El protocolo y el driver son estáticos y quedan
dentro de `libmovemaster_hardware.so`, que es compartida porque pluginlib la
carga en tiempo de ejecución.

**Límite de corriente (current_limit_a)**
Corriente máxima del motor en A (*smart current limit* de REV, IDs 59 y 60), la
misma en parada y a velocidad libre. De fábrica son 80 A en parada y 20 A libre.
Campo `spark.current_limit_a`; parte del baseline.

**Límites (min_position_rad / max_position_rad)**
Rango calibrado de cada articulación en `joints.json`. El driver rechaza
objetivos fuera de él (y enclava un fallo); el URDF los usa como `lower` y
`upper`.

**Link (URDF)**
Cuerpo rígido del URDF. En el modelo provisional son eslabones sin geometría
(`joint_N_link`).

## M

**MAXMotion**
Modo de control de posición del SPARK MAX que genera internamente un perfil de
movimiento con la velocidad de crucero y la aceleración máximas del slot. Por
eso el eje sigue al setpoint con algo de retraso. Se elige enviando
`MAXMOTION_POSITION_SETPOINT`.

**max_acceleration**
Campo de `slots.<n>.maxmotion` en `joints.json`: aceleración máxima del motor en
RPM/s.

**max_cycle_gap_s**
Campo global de `joints.json` (0.1 s): tiempo máximo sin transmitir estando
activo. Si el lazo se detiene más: `Control loop gap exceeded`.

**max_velocity_rad_s**
Campo de `joints.json`: velocidad máxima de la articulación en rad/s. Es el
límite de velocidad del URDF, con el que planea MoveIt, y ninguna
`cruise_velocity` puede superarlo.

**maxmotion_console**
Nombre anterior de `spark_console`.

**Mock hardware / use_mock_hardware**
Argumento del launch y del macro: `true` usa `mock_components/GenericSystem` en
lugar del plugin real, para probar sin CAN ni motores.

**Modo de control (position / maxmotion)**
Cómo persigue el SPARK cada setpoint. `maxmotion`: genera un perfil con los
límites del slot. `position`: su PID va directo al setpoint, sin perfil; sirve
para trayectorias ya perfiladas, como las de MoveIt. No hay que escribir ningún
parámetro: la trama de cada setpoint fija el modo (Control Type), así que
cambiarlo es inmediato. Campo `control.mode`.

**MoveIt**
Planificador de movimientos de ROS 2 (cinemática, colisiones, trayectorias).
Enviará trayectorias al JTC por la acción `follow_joint_trajectory`. Aún no
está configurado en el proyecto.

**movemaster_control**
Paquete que configura y arranca el nodo `controller_manager`: launch, YAML de
controladores y URDF provisional.

**movemaster_hardware**
Paquete del plugin: protocolo, driver, SocketCAN, `MovemasterHardware`,
ejemplos y pruebas. Está en `src/movemaster_ros/movemaster_ros/`.

**MovemasterHardware**
Clase C++ del plugin (`movemaster_hardware.cpp`) que implementa
`hardware_interface::SystemInterface` y adapta el ciclo de vida y el lazo de
`ros2_control` al driver.

**movemaster_node / movemaster_ros**
Paquete Python anterior con un nodo de prueba que habla con un SPARK directamente.
No debe ejecutarse a la vez que el `controller_manager`: también usa `can0`.

**MovemasterSystem**
Nombre del componente de hardware en el bloque `<ros2_control>`. Es el que se usa
en `ros2 control set_hardware_component_state MovemasterSystem …`.

## N

**NEO**
Motor sin escobillas de REV que mueve cada eje; su encoder interno da la
posición y velocidad de `STATUS_2`.

**nlohmann json**
Librería C++ de JSON usada por el protocolo y el driver
(`nlohmann-json3-dev`).

**Nodo (node)**
Proceso o unidad de ROS 2 con nombre, que publica, se suscribe y ofrece
servicios. `/controller_manager`, `/robot_state_publisher`.

## O

**OpaqueFunction**
Acción de launch que ejecuta una función Python cuando los argumentos ya
tienen valor. El launch de MoveMaster la usa para leer `joints.json` y generar
el URDF y los parámetros.

**output_range**
Campo opcional de `slots.<n>`: ciclo de trabajo mínimo y máximo del PID,
`[-1, 1]` si falta. Limita cuánto puede empujar el motor ese slot.

## P

**package.xml**
Manifiesto de un paquete ROS: nombre, versión, responsable, licencia y
dependencias. MANUAL §13.2.

**Parámetro (ROS) / ros__parameters**
Valor de configuración de un nodo. En un YAML, va bajo
`<nombre_del_nodo>: ros__parameters:`.

**period_s**
Campo global de `joints.json` (0.020 s): periodo del lazo. El launch pone
`update_rate = 1/period_s`; el driver descarta envíos a menos de medio periodo.

**PERSIST_PARAMETERS**
Trama que copia a flash **todos** los parámetros de la RAM del SPARK. Por eso
`spark_commission` restablece antes con `RESET_SAFE_PARAMETERS`. El driver
nunca la envía.

**PIC (POSITION_INDEPENDENT_CODE)**
Opción de compilación necesaria para meter una librería estática dentro de una
compartida.

**PIDF**
Ganancias del lazo de posición del SPARK: proporcional, integral, derivativa y
*feedforward*. Campo `pidf` de cada slot; las usan los dos modos.

**Plugin / pluginlib**
Clase C++ compilada en una librería que otro programa carga por nombre en
tiempo de ejecución. pluginlib es el mecanismo de ROS para hacerlo. El
`controller_manager` carga `movemaster_hardware/MovemasterHardware`, y los
controladores también son plugins.

**PLUGINLIB_EXPORT_CLASS**
Macro al final de `movemaster_hardware.cpp` que registra la clase en la
librería para pluginlib.

**position**
Interfaz de comando (objetivo) y de estado (medida) de cada eje, en radianes.
No confundir con el modo de control `position`.

**protocol_demo**
Ejemplo que muestra tramas codificadas sin abrir CAN.

**Puesta en marcha (spark_commission)**
Paso que se hace una vez por SPARK: restablece sus parámetros a fábrica
(conserva CAN ID, tipo de motor e idle mode), escribe el baseline y lo guarda
en flash. MANUAL §6.1.

**PV / SP**
*Process value* (posición medida) y *setpoint* (posición objetivo).

## R

**read() / write()**
Métodos del plugin que el `controller_manager` llama en cada ciclo: `read()`
recibe la telemetría y `write()` envía setpoints y heartbeat.

**RESET_SAFE_PARAMETERS**
Trama que devuelve a fábrica casi todos los parámetros del SPARK, excepto CAN
ID, tipo de motor, idle mode, deadband PWM y offset del duty cycle. Primer paso
de la puesta en marcha.

**Resource manager**
Parte del `controller_manager` que carga los componentes de hardware y reparte
sus interfaces entre los controladores.

**response_timeout_s**
Campo global de `joints.json` (0.5 s): espera máxima de cada respuesta del
SPARK al configurar y de la telemetría al activar.

**REV Hardware Client**
Aplicación de REV para asignar el CAN ID, actualizar el firmware y revisar la
configuración del SPARK MAX por USB-C. El resto del baseline lo guarda
`spark_commission`.

**robot_description**
Parámetro y tópico (`/robot_description`) con el URDF como texto. En Jazzy el
`controller_manager` lo lee del tópico que publica `robot_state_publisher`.

**robot_state_publisher**
Nodo que publica `/robot_description` y, a partir de `/joint_states`, la
posición de cada eslabón en TF.

**ros2 control (CLI)**
Comandos para inspeccionar y operar el `controller_manager`:
`list_controllers`, `list_hardware_components`, `list_hardware_interfaces`,
`switch_controllers`, `set_hardware_component_state`.

**ros2_control / `<ros2_control>`**
Marco de ROS 2 para conectar hardware y controladores. El bloque
`<ros2_control>` del URDF dice qué plugin de hardware cargar, con qué
parámetros y qué interfaces tiene cada articulación.

**ros2_control_node**
Ejecutable del paquete `controller_manager` que crea el nodo
`/controller_manager` y su hilo del lazo.

**ros2_controllers**
Colección de controladores estándar (JTC, JSB y otros) instalada con ROS.

**rosdep**
Herramienta que lee los `package.xml` e instala las dependencias del sistema
(`rosdep install --from-paths …`).

**Rotaciones de motor**
Unidad del SPARK para posición (vueltas del encoder del motor). El driver
convierte a radianes de articulación con `gear_ratio`, `direction` y
`zero_offset_rad`.

**RViz**
Visualizador 3D de ROS para ver el robot, TF y trayectorias.

## S

**set_control**
Método del driver para cambiar el modo y el slot de un eje en caliente; aplica
desde el siguiente `write()`. En `spark_console`: `mode` y `slot`.

**Setpoint (POSITION_SETPOINT / MAXMOTION_POSITION_SETPOINT)**
Trama con la posición objetivo, en rotaciones de motor, y el slot (`PID_SLOT`).
La trama elige el modo de control. Se envía una por eje en cada ciclo activo.

**slot**
Cada SPARK tiene cuatro (0–3); cada uno guarda PIDF, rango de salida y un
perfil MAXMotion: son los presets del eje. En `joints.json`, `slots."0"` a
`slots."3"`; `control.slot` elige el de arranque. Viaja en cada setpoint, así
que cambiarlo es inmediato.

**SocketCAN**
Interfaz de Linux para CAN y clase C++ del plugin (`socketcan.cpp`) que abre
`can0` sin bloqueo, con filtros para las tramas que interesan.

**source / setup.bash**
`source install/setup.bash` agrega el workspace al entorno de la terminal para
que ROS encuentre sus paquetes. Hay que hacerlo en cada terminal nueva.

**spawner**
Programa de `controller_manager` que pide cargar, configurar y activar
controladores: `ros2 run controller_manager spawner <controladores>`. El launch
lo usa para el JSB y el JTC.

**SPARK MAX**
Controlador de motor de REV Robotics, uno por eje, conectado por CAN.

**spark-frames-2.1.0 (spec)**
JSON de REV con la descripción de todas las tramas CAN del SPARK. El protocolo
lo lee al construirse; no se modifica.

**spark_commission**
Herramienta de la puesta en marcha. Sin `--apply` solo escucha el bus; con
`--apply` restablece, escribe el baseline y lo persiste en cada SPARK.

**spark_console**
Ejemplo interactivo para mover un eje sin ROS (`on`, `sp`, `mode`, `slot`,
`pv`, `off`, `q`). MANUAL §7.2.

**SparkParameters-v0.1.2.md**
Tabla en `movemaster_hardware/spec/` con todos los parámetros del SPARK: ID,
tipo, valor de fábrica, descripción y enumeraciones. `setup_test` comprueba
contra ella cada parámetro que escribe el código.

**SparkSetup**
Clase C++ con los intercambios que esperan respuesta (escribir parámetros con
ACK, restablecer, persistir). La usan `configure()` y `spark_commission`.

**STATUS_0 / STATUS_2**
Tramas periódicas del SPARK. `STATUS_0`: corriente, tensión, temperatura y si
recibe heartbeat. `STATUS_2`: posición y velocidad del encoder.

**status_period_ms**
Campo global de `joints.json` (20 ms): cada cuánto envía el SPARK `STATUS_0` y
`STATUS_2`.

**State interface (interfaz de estado)**
Valor que el hardware publica y los controladores leen. MoveMaster exporta
`position`, `velocity` y `current` por eje.

**symlink-install**
Opción de `colcon build` que instala enlaces a los archivos en lugar de copias:
los cambios en `joints.json`, YAML o launch se ven sin recompilar.

**SystemInterface**
Clase base de `ros2_control` para hardware con varias articulaciones.
`MovemasterHardware` hereda de ella.

## T

**TF**
Sistema de ROS que publica la posición de cada marco de referencia (eslabón)
en el tiempo. Lo alimenta `robot_state_publisher`.

**Tiempo real blando**
El lazo intenta cumplir su periodo pero sin garantía estricta de latencia. El
driver detecta pausas largas (`max_cycle_gap_s`) y enclava un fallo.

**time_from_start**
Campo de cada punto de una trayectoria: en qué momento, desde el inicio, debe
alcanzarse esa posición.

**Tópico (topic)**
Canal con nombre por el que un nodo publica mensajes y otros los reciben
(`/joint_states`).

## U

**update_rate**
Frecuencia del lazo del `controller_manager` en Hz. El launch la fija en
`1/period_s` (50 Hz por defecto).

**URDF**
Formato XML que describe el robot: eslabones, articulaciones, límites y el
bloque `<ros2_control>`. En MoveMaster se genera con xacro desde
`movemaster.urdf.xacro` y `joints.json`.

## W

**Watchdog**
Vigilancia que actúa si algo deja de llegar a tiempo. En el SPARK, deshabilita
el motor si falta el heartbeat. En el driver, enclava un fallo si falta
telemetría (`feedback_timeout_s`) o se detiene el lazo (`max_cycle_gap_s`).

**Workspace (movemaster_ws)**
Carpeta con `src/` y las carpetas que genera colcon. Se compila desde su raíz.

## X

**xacro**
Lenguaje de macros que genera URDF: propiedades, condiciones, inclusiones y
lectura de YAML/JSON. Archivos `.xacro` de este proyecto:
`movemaster_control/urdf/movemaster.urdf.xacro` y
`movemaster_hardware/config/ros2_control.xacro`.

## Y

**YAML / JSON**
Formatos de texto para datos. YAML se usa para parámetros de ROS
(`movemaster_controllers.yaml`, `colcon_defaults.yaml`); JSON, para
`joints.json` y el `spec` de REV. Todo JSON válido es también YAML válido, por
eso xacro puede leer `joints.json`.

## Z

**zero_offset_rad**
Campo de `joints.json`: ángulo de la articulación cuando el encoder del motor
marca cero. Como el encoder es relativo y no hay homing, hay que verificarlo
tras apagar los SPARK.
