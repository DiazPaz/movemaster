# movemaster_description

Modelo del brazo MoveMaster para ROS 2 Jazzy: el URDF que usan el
`controller_manager`, RViz y, después, MoveIt. Se genera con xacro a partir de
dos archivos, sin listas que mantener a mano:

| Archivo | Qué aporta |
|---|---|
| `config/dh.yaml` | La tabla Denavit-Hartenberg del brazo, la postura Home y los radios de los eslabones. |
| `movemaster_hardware/config/joints.json` | Qué ejes tienen SPARK MAX, con sus límites y su velocidad máxima. |

Guía completa: [docs/MANUAL.md](../docs/MANUAL.md) (§4.4 pose cero, Home y homing,
§9.1 vista previa, §13.6 el xacro). Términos:
[docs/DICCIONARIO.md](../docs/DICCIONARIO.md).

## La tabla DH

DH estándar, `T(i-1 → i) = Rz(θi) · Tz(di) · Tx(ai) · Rx(αi)`, con θi = qi, la
posición del eje en ROS, sin desfases:

| Eje | `label` | a (mm) | α | d (mm) | Home |
|---|---|---|---|---|---|
| `joint_1` | cintura (`waist`) | 0 | −90° | 250 | 0° |
| `joint_2` | hombro (`shoulder`) | 220 | 0° | 0 | −90° |
| `joint_3` | codo (`elbow`) | 157.4 | 0° | 0 | 90° |
| `joint_4` | inclinación de muñeca (`wrist_pitch`) | 0 | −90° | 0 | 0° |
| `joint_5` | giro del faceplate (`wrist_roll`) | 0 | 0° | 150 | 0° |

- **Pose cero** (todos los ejes en 0): el brazo y el antebrazo horizontales al
  frente y el faceplate hacia abajo, con su marca al frente. `tool0` (el centro
  de la cara del faceplate) está en (0.3774, 0, 0.100) m de `base_link`, con su
  eje z hacia abajo.
- **Home** (0, −90°, 90°, 0, 0): el brazo vertical, el antebrazo horizontal al
  frente y el faceplate hacia abajo; `tool0` en (0.1574, 0, 0.320) m. El homing
  termina ahí.
- **Positivos:** cintura antihorario vista desde arriba; hombro, codo e
  inclinación de muñeca inclinan el brazo hacia adelante; faceplate antihorario
  visto de frente.

## Cómo se arma el URDF

```text
base_link ─ joint_1 ─ waist_link ─ joint_1_dh ─ dh_frame_1 ─ joint_2 ─ shoulder_link ─ … ─ tool0
           (θ1, z)               (Tz(d1)·Tx(a1)·Rx(α1))
```

- `joint_i` es el giro θi = qi alrededor de z. Si el eje está en
  `joints.json`, es `revolute` con sus límites; si no, es `fixed` en su ángulo
  Home (con `preview:=true`, `revolute` de −π a π). Con un solo SPARK conectado
  el modelo ya está completo.
- `joint_i_dh` es la parte fija de la fila, hasta el marco DH `dh_frame_i`; el
  último marco es `tool0`.
- El bloque `<ros2_control>` lo escribe el macro de `movemaster_hardware` con
  los mismos ejes de `joints.json`. El hardware simulado arranca en Home.
- Un eje de `joints.json` que no está en la tabla detiene el xacro con un error.

Para cambiar la geometría se edita `dh.yaml`; para habilitar un eje, se agrega a
`joints.json` con el nombre de la tabla.

## Vista previa

```bash
ros2 launch movemaster_description display.launch.py            # sliders + RViz
ros2 launch movemaster_description display.launch.py gui:=false # brazo fijo en Home
```

Usa el URDF con `preview:=true`: los cinco ejes se mueven, aunque
`joints.json` todavía no los tenga a todos, y no hay bloque `<ros2_control>`.

| Argumento | Por defecto | Uso |
|---|---|---|
| `gui` | `true` | `false`: `joint_state_publisher` sin ventana, el brazo queda en Home. |
| `rviz` | `true` | `false`: sin RViz. |
| `joint_config` | `joints.json` de `movemaster_hardware` | Límites de los ejes con SPARK. |
| `dh_config` | `config/dh.yaml` | Otra tabla DH. |

Con el `controller_manager` corriendo, RViz muestra el brazo real (o simulado):

```bash
rviz2 -d $(ros2 pkg prefix --share movemaster_description)/rviz/display.rviz
```

## La referencia de cada eje

El encoder del NEO es relativo: al encender, el SPARK cuenta desde la postura en
que está. El bringup corre primero `spark_home --home`, que busca un tope
mecánico de cada eje y le escribe a su SPARK su ángulo DH ahí
(`homing.stop_rad` en `joints.json`); después el brazo va a Home. Para revisar
la tabla contra el brazo, lleva los ejes a la pose cero y compara con un nivel;
el error de cada eje se corrige en su `stop_rad`. Detalle en MANUAL §4.4 y §7.3.

## Archivos

| Archivo | Contenido |
|---|---|
| `config/dh.yaml` | Tabla DH, Home, radios de los eslabones y tamaño de la base. |
| `urdf/movemaster.urdf.xacro` | El robot: argumentos, base, cadena DH y bloque `<ros2_control>`. |
| `urdf/movemaster_arm.xacro` | Macro que convierte cada fila DH en `joint_i`, `<label>_link` y `joint_i_dh`. |
| `launch/display.launch.py` | Vista previa con `robot_state_publisher`, `joint_state_publisher(_gui)` y RViz. |
| `rviz/display.rviz` | Configuración de RViz: modelo, TF de `base_link` y `tool0`. |
| `test/test_description.py` | Cinemática directa del URDF contra el producto DH, pose cero, Home, sentidos positivos, ejes fijos y bloque `<ros2_control>`. |
