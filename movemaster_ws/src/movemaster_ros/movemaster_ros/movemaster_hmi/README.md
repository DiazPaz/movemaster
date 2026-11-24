# movemaster_hmi

La pantalla táctil de MoveMaster (PySide6, 800x480) y su **API al robot**. El HMI
no conoce ROS: todo lo que pide al brazo y todo lo que sabe de él pasa por un
`RobotClient` (`movemaster_hmi/robot_client.py`). Ángulos en **grados, con Home
en 0**, uno por eje en el orden de `joints.json`. Una **pose** del TCP es
`[X, Y, Z, PITCH, ROLL]`, en mm y grados
([movemaster_kinematics](../movemaster_kinematics/README.md)).

## La API

```python
from movemaster_hmi import robot_client
from movemaster_hmi.robot_model import load_joints, load_jog_axes

client = robot_client.connect(load_joints(joints_json), load_jog_axes(jog_yaml),
                              robot_client.load_arm(joints_json))
client.positions_changed.connect(...)        # y las demás señales
client.start()
client.move_to([0, 30, -45, 0, 0], speed_percent=25)
client.move_to_pose([250, 50, 200, -90, 30], speed_percent=25)
client.run_program(steps, speed_percent=25)  # pasos waypoint, pose, gripper y wait
client.stop()
```

| Método | Qué hace |
|---|---|
| `move_to(grados, velocidad %)` | Lleva todos los ejes a sus ángulos; arrancan y paran juntos. |
| `go_home(velocidad %)` | `move_to` a 0 en todos los ejes. |
| `move_to_pose(pose, velocidad %)` | Lleva el TCP a la pose: la cinemática inversa da los ángulos (la solución más cercana a donde está el brazo) y se mueve como `move_to`, punto a punto. |
| `pose_of(grados)` | La pose del TCP con los ejes en esos ángulos (cinemática directa). |
| `joints_for(pose, cerca_de=None)` | Los ángulos de una pose, sin moverse; `ValueError` dice por qué no se alcanza. |
| `set_tool_length(mm)`, `tool_length()` | Otra herramienta solo para esta sesión (APPLY TCP); la que queda es `tool.length_mm` de `dh.yaml`. |
| `run_program(pasos, velocidad %)` | Ejecuta en orden `{'type': 'waypoint', 'joints': [...]}`, `{'type': 'pose', 'pose': [...]}`, `{'type': 'gripper', 'closed': bool}` y `{'type': 'wait', 'seconds': s}`. Antes de arrancar convierte cada pose en ángulos, partiendo del paso anterior: una pose inalcanzable se rechaza sin mover nada. |
| `set_gripper(cerrado)` | Abre o cierra el gripper. |
| `stop()` | Detiene el movimiento o el programa; el brazo se sostiene donde está. |

| Señal | Cuándo |
|---|---|
| `positions_changed(list)` | Ángulos medidos, unas 10 veces por segundo. |
| `mode_changed(str)` | Quién manda: `offline`, `stopped` (falla), `auto` (el HMI puede mover) o `manual` (el pendant hace jog). |
| `selection_changed(x, y)` | Ejes que el pendant tiene seleccionados en su joystick. |
| `gripper_changed(bool)` | El gripper se abrió o cerró. |
| `busy_changed(bool)`, `step_started(i)`, `finished(ok, mensaje)` | Avance de un movimiento o programa. |

Una orden que el robot no puede tomar (sin robot, en MANUAL, con una falla o ya
moviéndose) lanza `RobotError` con el motivo para el operador; un ángulo fuera
de los límites de `joints.json` o una pose inalcanzable, `ValueError`.

La duración de cada movimiento se calcula para que ningún eje pase su velocidad
ni su aceleración: `max_velocity_rad_s` y `max_acceleration_rad_s2` de
`joints.json` o, en modo MAXMotion, además la `cruise_velocity` y
`max_acceleration` de su slot. Al 100 % el eje más lento va a su límite.

Por debajo (`ros_robot_client.py`) usa solo lo que el bringup ya ofrece:

| Para | Usa |
|---|---|
| Posiciones y gripper | `/joint_states`, `/dynamic_joint_states` |
| Modo | `/controller_manager/list_controllers` |
| Selección del pendant | `/pendant/joy` (lo publica `jog_node`, el único que lee el puerto del ESP32) |
| Movimientos | acción `/joint_trajectory_controller/follow_joint_trajectory` |
| Gripper | `/gripper_controller/commands` |

Sin ROS instalado, `connect()` devuelve un cliente fuera de línea: el HMI abre
igual, para diseñar pantallas en una laptop.

## La pestaña CARTESIAN

| Parte | Qué hace |
|---|---|
| TARGET TCP POSE | X, Y, Z (mm), PITCH y ROLL (°) a donde ir. Al escribir, abajo a la derecha aparecen los ángulos que tendrían los ejes (`JOINTS: ...`) o por qué no se alcanza (`UNREACHABLE: ...`). |
| CURRENT TCP POSE | La pose medida del TCP, en vivo. |
| USE CURRENT | Copia la pose actual a los campos. |
| ADD TO PROGRAM | Agrega la pose como paso `POSE` del programa (o guarda el paso que se está editando). |
| MOVE TO POSE | Mueve el brazo; mientras se mueve, el botón dice STOP. En MANUAL no aparece. |

Un paso `POSE` del programa se edita con **EDIT**, que lo abre en esta pestaña.
**APPLY TCP** (CONFIGURATION) cambia la longitud de la herramienta solo para la
sesión; la permanente es `tool.length_mm` en `movemaster_description/config/dh.yaml`.

## Instalar (una vez, en la Raspberry Pi)

PySide6 no viene con ROS. El entorno virtual debe ver los paquetes del sistema
(rclpy, numpy, OpenCV, PyYAML); `rosdep install` instala OpenCV con apt
(`python3-opencv`):

```bash
python3 -m venv --system-site-packages ~/movemaster/hmi_venv
~/movemaster/hmi_venv/bin/pip install -r ~/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hmi/requirements.txt
```

Si ya tienen un entorno creado sin `--system-site-packages`, no verá `rclpy` y
el HMI abrirá fuera de línea: vuelvan a crearlo con esa opción.

OpenCV no se instala con pip: `opencv-python` trae numpy 2, que tapa al numpy
1.26 del sistema, y los mensajes de ROS, compilados con numpy 1, dejan de
importarse. Si ya lo instalaron en el entorno:
`~/movemaster/hmi_venv/bin/pip uninstall -y opencv-python numpy`.

## Correr

Con el bringup, en la misma terminal (necesita pantalla):

```bash
ros2 launch movemaster_bringup bringup.launch.py pendant:=/dev/ttyUSB0 hmi:=true hmi_python:=~/movemaster/hmi_venv/bin/python
```

Solo el HMI, con el robot ya arrancado en otra terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/movemaster/movemaster_ws/install/setup.bash
~/movemaster/hmi_venv/bin/python -m movemaster_hmi
```

## Qué cambió en el HMI

- Muestra las **posiciones medidas** del brazo en grados reales, con los límites
  de `joints.json` en campos y sliders; ya no las estima desde el joystick.
- **MOVE ROBOT**, **GO HOME** y **RUN PROGRAM** mueven el brazo; mientras se
  mueve, el mismo botón dice **STOP**. La velocidad es la de MOVEMENT SPEED
  (25 % si está vacía).
- El encabezado dice el estado: OFFLINE, READY, MOVING, STEP n/m, MANUAL o FAULT.
- El switch del pendant manda: en MANUAL la pestaña MOVEMENT pasa a FREEDRIVE y
  el HMI no mueve el brazo; para moverlo desde el HMI, el switch va en AUTO.
- **CALIBRATE HOME** pasó a **GO HOME**: el homing lo hace el bringup al
  arrancar.
- El HMI ya no abre el puerto serie del pendant. La escena oculta se abre y se
  juega con el teclado; el modo Konami de la ESP32 no está disponible.

## Pruebas

```bash
cd movemaster_hmi && python3 -m pytest test
```

`test_robot_model.py` (límites, velocidades y duraciones) y `test_program.py`
(orden, fallas y paro de un programa) no necesitan ROS ni Qt.
