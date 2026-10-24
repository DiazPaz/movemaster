# Firmware del gripper (ESP32 + TMC2209 + servo)

`esp32_tmc2209.ino` convierte una ESP32 en un dispositivo más del bus CAN del brazo, al lado de
los SPARK MAX. Controla dos cosas:

- **La muñeca:** un motor a pasos con driver TMC2209, en grados (de −360° a 360°), con rampa de
  aceleración y frenado.
- **La pinza:** un servo por PWM, en grados (0 a 180).

La Raspberry Pi le manda órdenes por CAN y la ESP32 le responde con su estado cada 20 ms. Las
tramas están descritas en el catálogo
[`gripper-frames-1.0.0`](../../movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/spec/gripper-frames-1.0.0),
que usa el mismo formato que el JSON de REV y que leen las librerías del proyecto (C++ y Python).

---

## Conexiones

| ESP32 | Va a | Notas |
|---|---|---|
| GPIO 27 | STEP del TMC2209 | |
| GPIO 26 | DIR del TMC2209 | |
| GPIO 25 | EN del TMC2209 | LOW = habilitado. Pull-up de 10 kΩ a 3V3 para que arranque apagado. |
| GPIO 13 | Señal del servo | 220–470 Ω en serie. Pull-down de 10 kΩ para que no se sacuda al encender. |
| GPIO 21 | TXD del MCP2551 (pin 1) | Directo. TXD es la **entrada** del transceptor: no se cruza como en un UART. |
| GPIO 22 | RXD del MCP2551 (pin 4) | Con divisor: 1 kΩ en serie desde RXD y 2 kΩ a GND (5 V → 3.3 V). |
| 3V3 | VIO del TMC2209 | Así la lógica del driver es de 3.3 V. |
| GND | GND del TMC2209, VSS del MCP2551 (pin 2) y del servo | Tierra común obligatoria. |

- **TMC2209:** VM a 12 V con un capacitor de 100 µF; MS1 y MS2 a GND (1/8 de micropaso).
- **MCP2551:** VDD a 5 V con un capacitor de 100 nF junto al chip; Rs (pin 8) a GND.
- **Servo:** 7.4 V de su propio regulador, con 470–1000 µF y 100 nF junto al conector.
- **Terminación:** una resistencia de 120 Ω en cada extremo del bus. Con todo apagado, entre
  CANH y CANL se miden ~60 Ω.

---

## Identidad en el bus

El ID de cada trama es de 29 bits con el formato FRC que usan los SPARK MAX. Se puede
decodificar con [`tools/decode_arbitration_id.py`](../decode_arbitration_id.py):

```
 tipo 10 (Miscellaneous) | fabricante 8 (Team Use) | clase | índice | dispositivo 5
 ID = 0x0A080000 | clase << 10 | índice << 6 | 5
```

- **Tipo 10 y fabricante 8** son los que FRC reserva para dispositivos hechos por los equipos.
  Como el fabricante no es REV (5), ningún ID del gripper puede coincidir con uno de un SPARK.
- **Prioridad:** el tipo ocupa los bits más altos del ID y en CAN gana el ID más bajo. Con tipo
  10, las tramas del gripper ceden el bus a las de los SPARK (tipo 2).
- **Dispositivo 5:** se cambia en `CAN_DISPOSITIVO`.

## Tramas

| Clase, índice | ID | Dirección | Trama | Datos |
|---|---|---|---|---|
| 0, 0 | `0x0A080005` | Pi → ESP32 | `WRIST_POSITION_SETPOINT` | float32: objetivo de la muñeca en grados |
| 0, 1 | `0x0A080045` | Pi → ESP32 | `SERVO_SETPOINT` | uint8: ángulo del servo (0 a 180) |
| 0, 2 | `0x0A080085` | Pi → ESP32 | `STOP` | Sin datos: frena la muñeca con rampa |
| 1, 0 | `0x0A080405` | Pi → ESP32 | `SET_MOTION_PROFILE` | float32 °/s y float32 °/s² |
| 1, 1 | `0x0A080445` | Pi → ESP32 | `SET_WRIST_POSITION` | float32: grados actuales (fija la referencia) |
| 2, 0 | `0x0A080805` | ESP32 → Pi | `STATUS_0`, cada 20 ms | Estado (tabla de abajo) |
| 3, 0 | `0x0A080C05` | Pi → ESP32 | `HEARTBEAT`, cada 10–20 ms | Sin datos: habilita al gripper |

La clase agrupa las tramas por función: 0 comandos de movimiento, 1 configuración, 2 estado y
3 habilitación. Todos los datos van en *little-endian*.

`STATUS_0` (8 bytes):

| Bits | Señal | Significado |
|---|---|---|
| 0–31 | `WRIST_POSITION` | float32: posición de la muñeca en grados |
| 32–39 | `SERVO_ANGLE` | Último ángulo mandado al servo |
| 40 | `DRIVER_ENERGIZED` | El TMC2209 está habilitado: la muñeca tiene torque |
| 41 | `WRIST_MOVING` | La muñeca se mueve o no ha llegado a su objetivo |
| 42 | `HEARTBEAT_PRESENT` | Llega `HEARTBEAT` |
| 43 | `NOT_REFERENCED` | La posición no tiene referencia: se activa al arrancar y se borra con `SET_WRIST_POSITION` |
| 44 | `SERVO_ACTIVE` | El servo recibe señal PWM |
| 45–63 | `RESERVED` | En cero |

`NOT_REFERENCED` sirve también para detectar que la ESP32 se reinició, igual que `HAS_RESET`
en los SPARK.

---

## Heartbeat y seguridad

El gripper tiene **su propio `HEARTBEAT`** y no escucha el heartbeat global de los SPARK MAX
(`0x01011840`). Ese heartbeat global habilita a todo dispositivo que lo oiga, lo mande quien lo
mande; si el gripper lo usara, probarlo solo obligaría a habilitar también al brazo. Con un
heartbeat propio, las pruebas del gripper no habilitan ningún SPARK ni bloquean
`spark_commission`. Es la misma idea del `SECONDARY_HEARTBEAT` de REV.

Reglas del firmware:

- **Comandos de movimiento** (clase 0): solo se aceptan con heartbeat. `STOP` se acepta siempre.
- **Configuración** (clase 1): se acepta siempre, para poder configurar el gripper antes de
  habilitarlo.
- **Si pasan 100 ms sin heartbeat:** la muñeca se detiene en seco y **mantiene su torque** (no
  se suelta ni pierde su posición), y el servo **se queda en su último ángulo** (no suelta la
  pieza). Cuando vuelve el heartbeat, nada se mueve hasta recibir un setpoint nuevo.
- **Al encender:** el TMC2209 queda apagado hasta el primer heartbeat y el servo no recibe señal
  hasta su primer setpoint. Nada se mueve solo.

---

## Parámetros del programa

Todos son constantes al inicio de `esp32_tmc2209.ino`:

| Constante | Valor | Qué es |
|---|---|---|
| `REDUCCION` | 1.0 | Vueltas del motor por vuelta de la muñeca. Con varias etapas, se multiplican (`3.0 * 4.0`). |
| `INVERTIR_MUNECA` | false | `true` si la reducción invierte el sentido de giro. |
| `PASOS_MOTOR` | 200 | Pasos por vuelta del motor: 200 si es de 1.8°, 400 si es de 0.9°. |
| `MICROPASOS` | 8 | Debe coincidir con MS1/MS2: 8, 16, 32 o 64. Sin UART el TMC2209 no trabaja en paso completo. |
| `LIMITE_GRADOS` | 360 | Recorrido permitido de la muñeca, en ambos sentidos. |
| `VEL_MAXIMA` | 20000 | Pasos/s máximos. La muñeca llega como máximo a `VEL_MAXIMA / PASOS_POR_GRADO` °/s. |
| `HEARTBEAT_TIMEOUT_MS` | 100 | Tiempo sin heartbeat para detenerse. |
| `CAN_DISPOSITIVO` | 5 | Número de dispositivo en el bus. |
| `SERVO_US_MIN`, `SERVO_US_MAX` | 500, 2500 | Pulso del servo en 0° y en 180°. |

La conversión queda en `PASOS_POR_GRADO = PASOS_MOTOR * MICROPASOS * REDUCCION / 360`. Al
arrancar, el monitor serie la imprime junto con la velocidad máxima de la muñeca.

---

## Compilar y cargar

1. En el Arduino IDE, instala **esp32 de Espressif versión 3.x** desde el gestor de placas y
   elige **ESP32 Dev Module**. El programa no necesita librerías.
2. Ajusta los parámetros de la muñeca y carga el programa.
3. El monitor serie (115200) muestra al arrancar:
   ```
   Gripper: CAN a 1 Mbit/s, dispositivo 5, STATUS_0 en 0x0A080805
   Muñeca: 4.444 pasos por grado, hasta 4500.0 °/s
   ```
   y después, una vez por segundo, la posición, el servo, el heartbeat y el estado del CAN.

---

## Pruebas desde el bus CAN

Se hacen desde la Raspberry Pi, por etapas: si algo falla, se sabe en qué parte está. Los SPARK
pueden quedarse encendidos, porque el heartbeat del gripper no los habilita.

**1. Comunicación** (sin alimentar el motor ni el servo):

```bash
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
candump -td can0,0A080805:1FFFFFFF
#  (0.020)  can0  0A080805   [8]  00 00 00 00 5A 08 00 00
```

Debe llegar un `STATUS_0` cada 20 ms: 0°, servo en 90 (todavía sin señal) y `NOT_REFERENCED`.
Si no llega nada, revisa el monitor serie: `TEC` en 128 o más significa que nadie confirma las
tramas de la ESP32 (cableado de CANH/CANL, velocidad o terminación).

**2. Heartbeat**, en otra terminal:

```bash
while true; do cansend can0 0A080C05#; sleep 0.02; done
```

El byte de banderas pasa a `0D` (energizado, heartbeat, sin referencia). Al detenerlo con
Ctrl+C, en menos de 100 ms pasa a `09`.

**3. Servo** (con sus 7.4 V; el heartbeat sigue corriendo):

```bash
cansend can0 0A080045#5A    # 90°
cansend can0 0A080045#50    # 80°
```

Busca de a poco los ángulos de abierto y cerrado del gripper, sin forzar el servo contra un tope.

**4. Muñeca** (VM a 12 V con la corriente ya ajustada en Vref):

```bash
cansend can0 0A080445#00000000            # la posición actual es 0°
cansend can0 0A080405#0000F04100007042    # perfil lento: 30 °/s y 60 °/s²
cansend can0 0A080005#00002041            # +10°
cansend can0 0A080005#000020C1            # -10°
cansend can0 0A080005#0000B442            # +90°
cansend can0 0A080085#                    # STOP
```

- Si +10° gira al revés, cambia `INVERTIR_MUNECA`.
- Si +90° no gira exactamente 90°, revisa `REDUCCION` y `PASOS_MOTOR`.
- Corta el heartbeat a medio movimiento: la muñeca debe detenerse con torque y no moverse sola
  al volver a iniciarlo.

Los bytes de un float se obtienen con:

```bash
python3 -c "import struct; print(struct.pack('<f', 45).hex())"    # 00003442
```

**Desde Python**, el catálogo evita calcular bytes a mano (desde la raíz del repositorio, con
`python-can`):

```python
import sys
import can
sys.path.insert(0, "libraries")
from sparkmax_json_protocol import SparkFrameDatabase

gripper = SparkFrameDatabase(
    "movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/spec/gripper-frames-1.0.0")
bus = can.Bus(channel="can0", interface="socketcan")

heartbeat = bus.send_periodic(gripper["HEARTBEAT"].packet(5).to_python_can(), 0.02)
bus.send(gripper["SET_WRIST_POSITION"].packet(5, {"POSITION": 0}).to_python_can())
bus.send(gripper["WRIST_POSITION_SETPOINT"].packet(5, {"SETPOINT": 90}).to_python_can())
bus.send(gripper["SERVO_SETPOINT"].packet(5, {"ANGLE": 120}).to_python_can())

status_id = gripper["STATUS_0"].arbitration_id(5)
msg = bus.recv(1.0)
while msg is not None and msg.arbitration_id != status_id:
    msg = bus.recv(1.0)
if msg is not None:
    print(gripper["STATUS_0"].decode_payload(msg.data))
heartbeat.stop()
```

`send_periodic` deja al kernel enviando el heartbeat cada 20 ms.

Cuando el driver de ROS controle el gripper, no mandes el heartbeat de prueba al mismo tiempo: el
paro del driver consiste en dejar de mandarlo, y un segundo emisor lo anularía.
