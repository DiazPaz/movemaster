# movemaster_kinematics

Cinemática directa e inversa del MoveMaster, en Python y sin ROS: de los
ángulos de las articulaciones a la pose de la punta de la herramienta (`fk`), y
de una pose a los ángulos (`ik`). La usan el HMI (pestaña **CARTESIAN**) y la
terminal.

La inversa es **analítica** (de forma cerrada), como la de un controlador
industrial: exacta, instantánea y siempre da la misma respuesta. No usa MoveIt
ni iteraciones.

## De dónde sale la geometría

| Dato | Archivo | Lo comparten |
|---|---|---|
| Tabla DH y desfases (Home en q = 0) | `movemaster_description/config/dh.yaml` → `chain` | URDF, RViz |
| Longitud de la herramienta (TCP) | `movemaster_description/config/dh.yaml` → `tool.length_mm` | URDF (marco `tcp`) |
| Límites y velocidad de cada eje | `movemaster_hardware/config/joints.json` | driver, URDF, HMI |

No hay números copiados: si cambia la herramienta, se cambia `tool.length_mm` y
todo (URDF, RViz, HMI) se entera.

## La pose: cinco números

| Campo | Unidad | Significado |
|---|---|---|
| X, Y, Z | m (mm en el HMI) | El TCP en `base_link`: la punta de la herramienta, `tool.length_mm` delante de la brida (`tool0`). |
| PITCH | rad (° en el HMI) | Inclinación del eje de la herramienta en el plano vertical del brazo: 0 apunta horizontal hacia fuera, −90° hacia abajo, +90° hacia arriba; más allá de ±90°, de regreso hacia la cintura. |
| ROLL | rad (° en el HMI) | Giro de la herramienta sobre su propio eje, antihorario mirando la punta desde la brida; 0 en Home. |

Con cinco ejes **no hay yaw**: la herramienta siempre queda en el plano
vertical que pasa por el eje de la cintura y el TCP, el plano que gira la
cintura. Por eso una pose tiene cinco números y no seis.

En Home (q = 0): X 157.4, Y 0, Z 255 mm, PITCH −90°, ROLL 0°.

## Cómo resuelve la inversa

1. Con PITCH sale la dirección de la herramienta; restando la herramienta y la
   muñeca (150 + 65 mm) desde el TCP queda el **centro de la muñeca**.
2. La **cintura** apunta el brazo hacia el TCP, o en sentido contrario para
   alcanzar por detrás inclinando el hombro.
3. **Hombro y codo** llevan el centro de la muñeca como un brazo de dos
   eslabones (codo arriba o codo abajo).
4. La **inclinación de muñeca** completa el PITCH y el **faceplate** el ROLL.

De esas soluciones descarta las que salen de los límites de `joints.json` y
devuelve la que las articulaciones alcanzan **primero** desde donde están (la
que menos tarda el eje más lento). Con el rango del codo, en la práctica solo
hay una. El faceplate tiene dos vueltas de rango: el ROLL se alcanza en la
vuelta más cercana, sin desenrollar.

Cuando no hay solución dice por qué:

| Mensaje | Causa |
|---|---|
| `Out of reach: the wrist would be N mm beyond what the arm reaches.` | Demasiado lejos (o demasiado cerca del hombro). |
| `Out of the joint limits: joint_2 would need -33.5 deg, its range is ...` | Geométricamente alcanzable, pero un eje tendría que salir de su rango. |
| `The tcp is on the waist axis: ...` | El TCP sobre el eje de la cintura: ahí el ángulo de la cintura no está definido. |

## Desde la terminal

En mm y grados, sin el robot:

```bash
ros2 run movemaster_kinematics kinematics fk 0 0 0 0 0
# X 157.4 mm  Y 0.0 mm  Z 255.0 mm  pitch -90.0 deg  roll 0.0 deg

ros2 run movemaster_kinematics kinematics ik 250 50 200 -90 30
# J1 11.31 deg  J2 27.20 deg  J3 -15.97 deg  J4 -11.24 deg  J5 30.00 deg

ros2 run movemaster_kinematics kinematics ik 250 0 450 0 0
# Unreachable: Out of the joint limits: joint_2 would need -33.5 deg, its range is -26.0 to 119.7 deg.
```

`--near J1 J2 J3 J4 J5` dice desde dónde parten las articulaciones (por defecto,
Home); `--dh-config` y `--joint-config` usan otros archivos.

## Desde Python

```python
import math
from movemaster_kinematics import Arm, Pose, Unreachable

arm = Arm.from_files()                       # dh.yaml y joints.json instalados
pose = arm.fk([0.0] * 5)                     # Pose(x, y, z, pitch, roll) en m y rad
q = arm.ik(Pose(0.25, 0.05, 0.20, math.radians(-90), math.radians(30)), near=q_actual)
```

El HMI no la usa directo: pasa por su API (`RobotClient.move_to_pose`,
`pose_of`, `joints_for`), en mm y grados. Ver
[movemaster_hmi/README.md](../movemaster_hmi/README.md).

## El movimiento

`move_to_pose` resuelve los ángulos y mueve **articulación por articulación**
(punto a punto), como `move_to`: todas arrancan y terminan juntas, pero el TCP no
sigue una línea recta entre las dos poses.

## Pruebas

```bash
cd movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_kinematics
python3 -m pytest -q test
```

Comprueban Home y la pose cero DH, que la longitud de la herramienta viene de
`dh.yaml`, que `ik(fk(q))` devuelve el mismo `q` en miles de posturas dentro de
los límites, que la solución más cercana gana, que alcanza por detrás, que
respeta los límites y que cada pose imposible dice por qué. La prueba de
`movemaster_description` comprueba que el marco `tcp` del URDF está en el mismo
lugar.
