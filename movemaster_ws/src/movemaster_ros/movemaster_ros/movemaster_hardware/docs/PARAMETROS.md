# Parámetros de los SPARK MAX: qué se guarda, dónde y quién lo escribe

Cada dato vive en un solo lugar, según lo seguido que cambia.

| Nivel | Datos | Dónde vive | Quién lo escribe | Cuándo cambia |
|---|---|---|---|---|
| Puesta en marcha | CAN ID, tipo de motor, idle mode, límite de corriente | Flash del SPARK | REV Hardware Client (CAN ID) y `spark_commission --apply` (el resto) | Una vez por SPARK, o al reemplazarlo |
| Eje | Conversión, límites, slots (PIDF, rango de salida, MAXMotion), control por defecto | `joints.json` | El driver, en RAM, cada vez que configura | Al calibrar |
| Operación | Modo y slot de cada eje | Memoria del driver | `set_control()`; en la consola, `mode` y `slot` | En tiempo real |
| Programas (pendiente) | Waypoints y preset de cada movimiento | Archivos en la Pi | Teach pendant | Al guardar un programa |

Solo el primer nivel se persiste. Los otros se escriben en RAM o no llegan al
SPARK, así que cambiarlos no desgasta la flash ni tarda lo que tarda guardar.

## Por qué solo esos cuatro datos van a flash

- **REV los separa igual.** Según `spark-frames-2.1.0`, `RESET_SAFE_PARAMETERS`
  devuelve a fábrica casi todos los parámetros, "excepto CAN ID, Motor Type,
  Idle Mode, PWM Input Deadband y Duty Cycle Offset". Son los datos que
  identifican al SPARK y lo protegen; el límite de corriente se suma por la
  misma razón.
- **Protegen antes de que la Pi configure.** Entre que el SPARK enciende y el
  driver lo configura, y con cualquier programa que no los escriba (el backend
  Python, por ejemplo), solo cuenta lo que hay en flash.
- **Persistir guarda todo.** `PERSIST_PARAMETERS` copia a flash todos los
  parámetros de la RAM. Por eso la puesta en marcha restablece primero a
  fábrica: así la flash queda con los valores de fábrica más estos cuatro,
  sin importar lo que hubiera en RAM.
- **El resto se reescribe en cada arranque.** Lo que el driver controla,
  incluido el baseline, lo escribe en RAM cada vez que configura. Eso no
  depende de lo que haya en flash ni de lo que haya guardado otro programa.

REVLib 2025 hace algo parecido con `ResetMode.kResetSafeParameters` y
`PersistMode.kPersistParameters`, pero persiste toda la configuración. Aquí
solo se persiste el baseline.

## Los parámetros de REV que usa MoveMaster

La tabla completa de parámetros del SPARK, con ID, tipo, valor de fábrica y
descripción, está en [`spec/SparkParameters-v0.1.2.md`](../spec/SparkParameters-v0.1.2.md).
Estos son los que toca MoveMaster, con su nombre en esa tabla; `s` es el slot
(0 a 3). Todos se escriben con `PARAMETER_WRITE`, y el driver exige que el ACK
confirme el tipo y el valor exacto.

| ID | Parámetro | Tipo | Fábrica | MoveMaster | Nivel | Lo escribe |
|---|---|---|---|---|---|---|
| 0 | CAN ID | UINT32 | 0 | 1 a 6, uno por eje | Puesta en marcha | REV Hardware Client |
| 2 | Motor Type | UINT32 | BRUSHLESS (1) | BRUSHLESS (1) | Puesta en marcha | `spark_commission` (flash) y el driver (RAM) |
| 6 | Idle Mode | UINT32 | COAST (0) | `spark.idle_mode` | Puesta en marcha | `spark_commission` y el driver |
| 59 | Smart Current Stall Limit | UINT32 | 80 A | `spark.current_limit_a` | Puesta en marcha | `spark_commission` y el driver |
| 60 | Smart Current Free Limit | UINT32 | 20 A | `spark.current_limit_a` | Puesta en marcha | `spark_commission` y el driver |
| 9 | Closed Loop Control Sensor | UINT32 | NONE (0) | MAIN_ENCODER (1) | Eje (fijo) | El driver |
| 112 | Position Conversion Factor | FLOAT | 1.0 | 1.0 | Eje (fijo) | El driver |
| 113 | Velocity Conversion Factor | FLOAT | 1.0 | 1.0 | Eje (fijo) | El driver |
| 149 | Position PID Wrap Enable | BOOL | false | false | Eje (fijo) | El driver |
| 158, 160 | Status 0 Period, Status 2 Period | UINT32 | 10 y 20 ms | `status_period_ms` | Eje | El driver |
| 13+8s a 16+8s | P s, I s, D s, F s | FLOAT | 0 | `slots.<s>.pidf` | Eje (slot) | El driver |
| 19+8s, 20+8s | Output Min s, Output Max s | FLOAT | −1 y 1 | `slots.<s>.output_range` | Eje (slot) | El driver |
| 166+5s, 167+5s, 169+5s | MAXMotion Max Velocity s, Max Accel s, Allowed Closed Loop Error s | FLOAT | 0 | `slots.<s>.maxmotion` | Eje (slot) | El driver |

Los IDs 9, 13–16, 112, 113, 149, 158, 160, 166, 167 y 169 son los que ya usaba
el backend Python validado en el banco. Los demás (2, 6, 19, 20, 59 y 60) se
comprobaron contra la tabla, igual que todos los anteriores. Lo hace la prueba
`config_and_commissioning` en cada compilación: revisa ID, nombre, tipo y los
valores de `MotorType`, `IdleMode` y `Sensor`. Aun así, no se han probado con un
SPARK real. Si un ACK trae otro tipo o valor, el driver se detiene antes de
habilitar y la puesta en marcha no llega a persistir. La primera vez, prueba con
un solo SPARK en el bus y revisa el resultado en REV Hardware Client.

Tres detalles de la tabla:

- **Status 0/2 Period** dice "in μs", pero sus valores de fábrica (10 y 20) son
  los `defaultPeriodMs` de `spark-frames-2.1.0`, y el backend validado escribe
  20 para 20 ms: la unidad es ms.
- **Control Type (ID 5)** guarda el modo activo. No hace falta escribirlo: cada
  trama de setpoint lo fija (ver [Slots y modos](#slots-y-modos-de-control)).
- **Inverted (ID 45)** invierte el motor. MoveMaster no lo usa: el sentido lo da
  `direction`, y la puesta en marcha lo deja en `false`.

Lo que el driver no escribe (I-zone, D Filter, rampas, límites suaves, Inverted,
etc.) queda en el valor de fábrica gracias al restablecimiento de la puesta en
marcha. Según la tabla, los parámetros se guardan aparte del firmware y
sobreviven a una actualización de firmware.

## `gear_ratio` y el factor de conversión del SPARK

Los dos convierten unidades, pero no son intercambiables:

- **El SPARK** mide rotaciones del motor. Su factor de posición (ID 112)
  multiplica lo que reporta en `STATUS_2` y lo que entiende de cada setpoint;
  el de velocidad (ID 113) hace lo mismo con las RPM.
- **`gear_ratio`**, junto con `direction` y `zero_offset_rad`, convierte en la
  Pi entre radianes de la articulación, que usan ROS y MoveIt, y rotaciones del
  motor: `q = zero_offset_rad + direction · rotaciones · 2π / gear_ratio`.

Por qué la conversión se hace solo en la Pi y los factores del SPARK quedan en
1.0:

1. **El factor solo multiplica.** No puede aplicar el offset ni el sentido, así
   que la Pi tendría que convertir de todos modos.
2. **El factor también cambia las unidades del lazo.** Con un factor distinto
   de 1, el error del PID, la cruise velocity, la aceleración y el error
   permitido de MAXMotion pasan a esas unidades nuevas, y las ganancias ya
   ajustadas dejan de valer. Con 1.0 todo queda en rotaciones y RPM del motor,
   como en la documentación de REV y en la hoja de datos del NEO.
3. **Dos conversiones se suman sin avisar.** Si el SPARK tuviera guardado
   2π/100 y la Pi dividiera además entre 100, la posición saldría mal por un
   factor de 100. Por eso el driver escribe 1.0 en cada arranque, y la puesta
   en marcha deja la flash con el valor de fábrica, también 1.0.
4. **La precisión es la misma.** Los setpoints viajan como float32 en ambos
   casos.

`gear_ratio` es entonces la única reducción del sistema. La consecuencia
práctica es que PIDF y MAXMotion se escriben en unidades del motor. Para pasar
a la articulación: `rad/s = RPM · 2π / (60 · gear_ratio)`. Por ejemplo, con
reducción 100:1, 1200 RPM del motor son 1.26 rad/s en la articulación.

## Slots y modos de control

Cada SPARK tiene cuatro slots (0 a 3). Un slot guarda PIDF, el rango de salida
del PID y un perfil MAXMotion (además de I-zone, D Filter e I Max Accum, que
MoveMaster deja en fábrica). No existe un juego de parámetros aparte para
Position: los dos modos usan el PIDF y el rango de salida del slot, y MAXMotion
agrega su perfil encima.

El modo y el slot viajan en cada setpoint: la trama elige el modo
(`POSITION_SETPOINT` o `MAXMOTION_POSITION_SETPOINT`) y su campo `PID_SLOT`
elige el slot. Según `spark-frames-2.1.0`, la trama misma "fija el Control
Type" (ID 5), así que cambiar cualquiera de los dos es inmediato y no requiere
escribir ni persistir ningún parámetro.

| Modo | Qué hace el SPARK | Cuándo conviene |
|---|---|---|
| `maxmotion` | Genera un perfil hacia el setpoint con la cruise velocity y la aceleración del slot. | Movimientos punto a punto y jog del teach pendant: el SPARK limita la velocidad. |
| `position` | El PID persigue cada setpoint, sin perfil. | Trayectorias que ya vienen perfiladas, como las de MoveIt por el `JointTrajectoryController`. |

Un slot sin bloque `maxmotion` solo se puede usar en modo `position`.

### Error de seguimiento en modo Position

En modo `position` el SPARK no limita la velocidad: un salto grande del
setpoint lleva el PID a su salida máxima. Por eso el driver rechaza, sin
enviarlo, cualquier objetivo que quede a más de `max_following_error_rad` de la
posición medida, y enclava un fallo. Así también se detecta un eje que deja de
seguir su trayectoria, por ejemplo por un choque. Elige el valor con el retraso
que muestra el eje a la velocidad máxima de tus trayectorias, más un margen. El
rango de salida del slot (`output_range`) limita además el ciclo de trabajo del
PID.

### Cambiar de modo y de slot

- **En el driver:** `set_control(eje, modo, slot)` aplica desde el siguiente
  `write()`, también con el eje habilitado, y se conserva al reactivar. Rechaza
  un slot no configurado, o MAXMotion en un slot sin perfil, sin enclavar
  ningún fallo.
- **En `spark_console`:** `mode position|maxmotion` y `slot <n>`. Al cambiar de
  modo, la consola mantiene la posición medida.
- **En ROS:** por ahora cada eje usa el `control` de `joints.json` desde que se
  activa. Cambiarlo desde ROS es el siguiente paso.

## `joints.json` v2

Ejemplo de un eje con dos slots: el 0 con perfil MAXMotion y el 2 solo para
Position. El cargador es estricto: una clave desconocida es un error, nunca un
ajuste que se queda en su valor por defecto.

```json
"joint_1": {
  "can_id": 1,
  "gear_ratio": 100.0,
  "direction": 1,
  "zero_offset_rad": 0,
  "min_position_rad": -2.6,
  "max_position_rad": 2.6,
  "max_velocity_rad_s": 1.5,
  "spark": {"motor_type": "brushless", "idle_mode": "brake", "current_limit_a": 40},
  "control": {"mode": "maxmotion", "slot": 0, "max_following_error_rad": 0.1},
  "slots": {
    "0": {"pidf": {"p": 0.5, "i": 0, "d": 0, "f": 0},
          "maxmotion": {"cruise_velocity": 1200, "max_acceleration": 2400, "allowed_profile_error": 0.05}},
    "2": {"pidf": {"p": 1.0, "i": 0.0001, "d": 0, "f": 0}, "output_range": [-0.3, 0.3]}
  }
}
```

| Campo | Regla | Para qué |
|---|---|---|
| `can_id` | Entero 0–63, único | Dirección del SPARK; se asigna con REV Hardware Client. |
| `gear_ratio` | > 0 | Vueltas del motor por vuelta de la articulación. |
| `direction` | 1 o −1 | Sentido de la articulación respecto al motor. |
| `zero_offset_rad` | Finito | Posición articular cuando el encoder marca 0. |
| `min_position_rad`, `max_position_rad` | Mínimo < máximo | Límites; el driver rechaza objetivos fuera de ellos. |
| `max_velocity_rad_s` | > 0 | Límite de velocidad del URDF, con el que planea MoveIt. Ninguna cruise velocity puede superarlo. |
| `spark.motor_type` | Solo `"brushless"` | Los NEO son brushless; el modo brushed puede dañarlos. |
| `spark.idle_mode` | `"coast"` o `"brake"` | Qué hace el SPARK cuando su salida es neutra, como al deshabilitarse. `brake` cortocircuita el motor y frena el brazo, pero no lo sostiene. De fábrica: `coast`. |
| `spark.current_limit_a` | Entero 1–80 | Límite de corriente del motor, igual en parada y a velocidad libre. De fábrica son 80 A en parada y 20 A libre; 40 A es un valor habitual para NEO. |
| `control.mode` | `"position"` o `"maxmotion"` | Modo con el que el eje se activa. |
| `control.slot` | Uno de `slots` | Slot con el que el eje se activa. Con `maxmotion`, debe tener perfil. |
| `control.max_following_error_rad` | > 0 | Distancia máxima entre un objetivo en modo `position` y la posición medida. |
| `slots."0"` a `slots."3"` | Al menos uno | Presets del eje. |
| `slots.<n>.pidf` | `p`, `i`, `d`, `f` ≥ 0 | Ganancias del slot, en unidades del motor. |
| `slots.<n>.output_range` | `[min, max]`, −1 ≤ min < 0 < max ≤ 1; por defecto `[-1, 1]` | Ciclo de trabajo máximo del PID. |
| `slots.<n>.maxmotion` | Opcional: `cruise_velocity` (RPM) > 0, `max_acceleration` (RPM/s) > 0, `allowed_profile_error` (rotaciones) ≥ 0 | Perfil MAXMotion del slot. |

### Pasar del formato anterior

El driver rechaza el formato anterior con un mensaje que indica qué se movió.

| Antes | Ahora |
|---|---|
| `slot` | `control.slot` |
| `pidf` | `slots.<slot>.pidf` |
| `maxmotion` | `slots.<slot>.maxmotion` |
| Límite del URDF calculado desde `maxmotion.cruise_velocity` | `max_velocity_rad_s` |
| — | `spark` y `control` (nuevos, obligatorios) |

## Puesta en marcha de un SPARK

Se hace una vez por SPARK, y de nuevo al reemplazarlo o al cambiar su bloque
`spark`.

1. Asigna el CAN ID con REV Hardware Client, un SPARK a la vez por USB-C.
2. Completa `can_id` y `spark` de ese eje en `joints.json`.
3. Cierra todo lo que envíe heartbeat: el `controller_manager`, `spark_console`
   y el backend Python.
4. Revisa sin escribir nada, desde `movemaster_hardware/` y con el paquete ya
   compilado (paso 2 del manual):
   ```bash
   ./build/spark_commission spec/spark-frames-2.1.0 config/joints.json can0
   ```
   La herramienta solo escucha 0.5 s. Comprueba que cada SPARK responde en su
   CAN ID y que nadie envía el heartbeat de habilitación, y muestra lo que
   quedaría en flash.
5. Aplica, para todos los ejes o solo los que nombres:
   ```bash
   ./build/spark_commission spec/spark-frames-2.1.0 config/joints.json can0 --apply joint_2
   ```
   Para cada SPARK envía `RESET_SAFE_PARAMETERS`, escribe tipo de motor, idle
   mode y límite de corriente, y termina con `PERSIST_PARAMETERS`. Si algo
   falla antes de `PERSIST_PARAMETERS`, ese SPARK no guarda nada; la
   herramienta se detiene e indica qué eje repetir.
6. Comprueba el sentido de cada eje con `driver_monitor`, girándolo a mano. El
   restablecimiento deja `Inverted` en `false`: si lo habías activado en REV
   Hardware Client, el motor ahora gira al revés que antes, y hay que
   corregirlo con `direction`.

La lectura de parámetros por CAN no está disponible en SPARK MAX: el propio
`spark-frames-2.1.0` lo advierte en sus tramas `READ_PARAMETER`. Por eso la
herramienta confirma cada escritura con su ACK y el guardado con el resultado
de `PERSIST_PARAMETERS`, pero no puede releer la flash. Para comprobarla después
de apagar y encender, usa REV Hardware Client.

## Programas y presets

Los programas del teach pendant, con sus waypoints y presets de velocidad,
pertenecen a la Pi y no a los SPARK. Un SPARK solo tiene su tabla de
parámetros. No tiene lugar para waypoints, y seis SPARK no se guardan de forma
atómica. En la Pi se guardan como archivos, versionables con git. Un preset se
aplica de dos formas:

- **Movimientos MAXMotion:** eligiendo el slot que tiene ese perfil.
- **Trayectorias de MoveIt:** con los factores de escala de velocidad y
  aceleración del plan. Cambiar la cruise velocity de un SPARK en caliente haría
  que su eje se retrase respecto a los demás, y el brazo dejaría el camino que
  MoveIt revisó contra colisiones.
