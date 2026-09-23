# Teach pendant SPARK MAX — MAXMotion Position

Backend Python para pruebas de **un eje** desde terminal y posterior integración
con una interfaz gráfica. Utiliza directamente `SparkMAXMotionProtocol` de tu
`sparkmax_json_protocol.py` y las tramas de `spark-frames-2.1.0`. El heartbeat
conserva el ID, payload y período de tu `example_maxmotion_json.py`, que funciona
en la Raspberry Pi 5: `0x01011840`, ocho bytes `FF`, cada 20 ms.

## Archivos

| Archivo | Uso |
|---|---|
| `teach_pendant_backend.py` | Clase, hilo CAN, comandos de setup y telemetría. |
| `pendant_terminal.py` | Consola interactiva con monitorización en segundo plano. |
| `example_motion.py` | Ejemplo completo de inicialización y movimiento. |
| `test_backend.py` | Pruebas de integración sin hardware, sobre CAN virtual. |
| `sparkmax_json_protocol.py` | Tu librería original, incluida sin modificaciones. |
| `spark-frames-2.1.0` | Tu JSON original, incluido sin modificaciones. |
| `example_maxmotion_json.py` | Tu ejemplo funcional original, como referencia. |
| `requirements.txt` | Dependencia `python-can==4.6.1`. |

## Preparación y ejecución

Usa Python 3.10 o posterior. Para hardware real, se espera Linux con SocketCAN
(`can0` activa), un SPARK con el ID indicado y firmware compatible con el JSON
2.1.0 y el catálogo de parámetros suministrado. Las tramas modernas usadas aquí
se introdujeron en firmware 25; no son las tramas STATUS de firmware 24.
El tipo de motor y su conexión de encoder deben estar correctamente configurados.

Desde la carpeta descomprimida:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
ip -details link show can0
python pendant_terminal.py --channel can0 --id 1 --slot 0 --min-rot -2 --max-rot 2
```

El programa usa el bitrate que ya tenga `can0`; todos los nodos deben coincidir.
Cambiar `--id` selecciona el controlador. Los límites anteriores son ejemplos en
rotaciones del encoder del motor; ajústalos al recorrido permitido en tu banco.
Los límites del host validan objetivos, no garantizan que el mecanismo no rebase
un límite físico por inercia o error de control.

Al iniciar, la terminal configura y confirma el encoder, sus unidades y las
tramas de estado. El eje comienza deshabilitado. Secuencia de ejemplo:

```text
pidf 1.0 0 0 0
profile 500 900 0.1
jobs
status
arm
move 0.25
watch on 2
move 0.50
hold
watch off
disarm
quit
```

Revisa `jobs` hasta que los comandos de setup aparezcan como **OK** antes de
usar `arm`. P=1, I=0, aceleración=500 RPM/s, crucero=900 RPM y error=0.1 rot
son los valores ejecutables de tu ejemplo; aquí D y F se fijan explícitamente en
cero. Revisa estos valores al cambiar de mecanismo. `move 0.25` es una
posición **absoluta**, no un incremento. `arm` toma la PV actual como SP inicial;
no manda al eje a cero. El cero es el del encoder: este paquete no hace homing.

`watch` imprime telemetría desde un hilo independiente y permite seguir
escribiendo. Sus líneas pueden intercalarse con el prompt; `watch off` recupera
la consola limpia. `Ctrl+C`, `quit` y EOF cierran el backend.

Para reproducir también el guardado en flash de tu ejemplo, usa `save` y consulta
`jobs` hasta obtener OK **antes de `arm`**. Es opcional: cambiar parámetros en RAM
no obliga a guardarlos en flash. Un fallo de guardado enclava el fallo y bloquea
el movimiento. El ejemplo adjunto imprimía que no movería si fallaba el guardado,
pero continuaba ejecutando el heartbeat y el setpoint; aquí esa ruta se bloquea.

Para cambiar ganancias o perfil después de mover:

```text
disarm
accel 90
cruise 20
jobs
arm
move 0.75
```

## API y correspondencia con CAN

| Operación | Método / origen | Unidad |
|---|---|---|
| SP absoluto | `send_setpoint(rotations)` → `MAXMOTION_POSITION_SETPOINT` | rotaciones |
| PIDF | `set_pidf(p=..., i=..., d=..., f=...)` | según firmware y ganancia |
| Perfil | `set_motion_profile(maxacceleration=..., cruisevelocity=...)` | RPM/s y RPM |
| Aceleración individual | `set_maxacceleration(value)` | RPM/s |
| Crucero individual | `set_cruisevelocity(value)` | RPM |
| Guardar setup en flash | `persist_parameters()` | confirmación RESULT_CODE=0 |
| PV | `telemetry().pv_rot` ← `STATUS_2.PRIMARY_ENCODER_POSITION` | rotaciones |
| Velocidad | `telemetry().velocity_rpm` ← `STATUS_2.PRIMARY_ENCODER_VELOCITY` | RPM |
| Corriente | `telemetry().current_a` ← `STATUS_0.CURRENT` | A |
| Error | `telemetry().error_rot` = SP − PV | rotaciones |

El JSON coloca velocidad en los primeros cuatro bytes de STATUS_2 y posición
en los siguientes cuatro. La corriente es un campo de 12 bits que cruza bytes;
el codec aplica su escala a amperes. El backend obtiene estos campos por nombre
desde el JSON, sin reinterpretarlos manualmente.

Enviar `MAXMOTION_POSITION_SETPOINT` también selecciona MAXMotion Position en
el SPARK. No hace falta mandar otra trama para cambiar `ControlType`.
`hold()` publica la PV reciente como nuevo objetivo mediante esa misma trama;
el perfil sigue activo, por lo que no representa una parada instantánea.

El error solicitado es respecto al **objetivo final** SP. No es el error
interno contra cada posición intermedia del perfil MAXMotion.

## Ejemplo de uso desde Python

```python
import time
from teach_pendant_backend import TeachPendantBackend

with TeachPendantBackend(
    channel="can0", device_id=1, slot=0, position_limits=(-2.0, 2.0)
) as axis:
    axis.initialize().result(timeout=6)
    axis.set_pidf(p=1.0, i=0.0, d=0.0, f=0.0).result(timeout=3)
    axis.set_motion_profile(
        maxacceleration=500.0,   # RPM/s
        cruisevelocity=900.0,    # RPM
        allowed_profile_error=0.1,
    ).result(timeout=3)
    # Opcional, igual que el ejemplo original:
    # axis.persist_parameters().result(timeout=4)

    deadline = time.monotonic() + 2.0
    while True:
        t = axis.telemetry()
        if t.fault:
            raise RuntimeError(t.fault)
        if t.position_fresh and t.current_fresh:
            break
        if time.monotonic() >= deadline:
            raise TimeoutError("No se recibió telemetría fresca")
        time.sleep(0.02)

    axis.arm()
    axis.send_setpoint(0.25)
    for _ in range(50):
        t = axis.telemetry()
        if t.fault:
            raise RuntimeError(t.fault)
        print(t.sp_rot, t.pv_rot, t.velocity_rpm, t.current_a, t.error_rot)
        time.sleep(0.1)  # CAN sigue activo durante esta espera.
    axis.disarm()
```

También puedes ejecutar el ejemplo que espera a que el movimiento se estabilice:

```bash
python example_motion.py --channel can0 --id 1 --target 0.25
# Con persistencia confirmada antes del movimiento:
python example_motion.py --channel can0 --id 1 --target 0.25 --persist
```

## Integración con una GUI sin bloquearla

Cada método de setup devuelve inmediatamente un `Command`. Usa un temporizador
de tu GUI para consultar `command.done()` y `axis.telemetry()`:

```python
pending = axis.set_pidf(p=0.015)
# En un tick posterior de la interfaz:
if pending.done():
    result = pending.result()  # Ya terminó: devuelve ACKs o lanza el error.
snapshot = axis.telemetry()
```

`result(timeout=...)` es una comodidad para scripts: espera sólo en el hilo que
lo invoca. No lo llames con un comando pendiente dentro del hilo gráfico.
Que esa espera expire no cancela el comando CAN; conserva el ticket y consulta
su resultado. El timeout del protocolo es independiente y sí falla el setup.

Los comandos de movimiento publican el último SP sin esperar una trayectoria.
Si publicas varios SP dentro del mismo período, se transmite el último. Los
comandos de setup se encolan en orden, con una sola respuesta pendiente a la vez.
`telemetry()` devuelve una instantánea inmutable; no ejecuta E/S CAN ni callbacks.

## Hilo, configuración y diagnóstico

El único hilo `spark-can` administra `send()` y `recv()`. Se utiliza
`time.monotonic()`, un período por defecto de 20 ms (50 Hz) para setpoint y
heartbeat, y esperas RX de hasta 5 ms. No se usa el helper síncrono
`write_parameter()` de la librería, porque competiría por las respuestas de CAN.
El worker continúa recibiendo mientras espera una confirmación de parámetros.

La inicialización hace, en este orden:

1. Salir del modo seguidor, para aceptar setpoints dirigidos al controlador.
2. Seleccionar encoder primario (parámetro 9 = 1).
3. Escribir factores de posición y velocidad = 1 (IDs 112 y 113).
4. Desactivar position wrapping (ID 149).
5. Configurar períodos STATUS_0/STATUS_2 (IDs 158 y 160; 20 ms por defecto).
6. Habilitar STATUS_0 y STATUS_2 con máscara `0b101`, preservando otros bits.

Cada paso espera su respuesta. Se verifican ID, tipo, resultado y valor float32
exactamente confirmado. Un rechazo o timeout detiene la secuencia, falla los
comandos encolados y enclava un fallo; los pasos anteriores pueden haber quedado
aplicados. El dispositivo no ofrece una transacción atómica de todos los valores.
La configuración se envía una vez por solicitud, no en cada tick del lazo.

Los parámetros PIDF y MAXMotion se toman del catálogo de la librería, incluido
el desplazamiento de cada slot. Los parámetros auxiliares de inicialización
corresponden al catálogo REV `SparkParameters-v0.1.2.md`. El catálogo de parámetros
y el JSON de tramas son especificaciones distintas: un firmware que cambie IDs
o tipos requiere adaptar el catálogo, aunque reconozca las mismas tramas CAN.
El rechazo se informa y el programa no habilita movimiento automáticamente.

`f` conserva el nombre y el ID que usa tu librería (16 + 8 × slot). Documentación
REV más reciente llama `kV` a ese ID y cambió la API de feedforward; comprueba la
semántica/unidad para tu firmware antes de reutilizar una ganancia F de otra
versión. No se realiza una conversión implícita F↔kV en este backend.

Los cambios de parámetros son volátiles hasta invocar `persist_parameters()` o
`save`. La persistencia usa el magic number del JSON y el mismo criterio de tu
ejemplo: sólo RESULT_CODE=0 confirma éxito; 255 mantiene la espera hasta 2.5 s
sin reiniciar el plazo. Después de la confirmación se esperan 0.25 s sin bloquear
el hilo CAN antes de completar el ticket. Aplica el setup en cada nueva sesión.

La telemetría conserva el último dato conocido e incluye su edad y banderas
`position_fresh` / `current_fresh`. Antes de recibir datos los valores son `None`.
Si PV caduca, `error_rot` pasa a `None`. Si el eje está habilitado y STATUS_0 o
STATUS_2 no se actualiza durante 300 ms, el backend detiene heartbeat/SP y enclava el
fallo. Volver a recibir datos no habilita el eje: hay que resolver la causa,
cerrar la instancia y crear otra. Un fallo CAN o de configuración también queda
visible en `telemetry().fault`.

## Alcance de tiempo real y operación

Es **tiempo real blando**: Python y un sistema operativo convencional no
garantizan deadlines. MAXMotion y el PID los ejecuta el SPARK. El backend ofrece
`skipped_periods` y `max_cycle_gap_s` para observar retrasos del ciclo del host;
estos contadores no prueban tiempos de respuesta del motor.

El heartbeat es **exactamente el de tu ejemplo funcional**, construido con
`CANPacket`: ID extendido `0x01011840` y ocho bytes `FF`. Esa trama no está en el
JSON de SPARK; se conserva explícitamente desde el programa de referencia.
Desarmado no se envía heartbeat. `PRIMARY_HEARTBEAT_LOCK` se muestra como dato
informativo; no bloquea el uso de este heartbeat. No se utiliza el heartbeat
secundario de la especificación.

Esta trama tiene alcance global: `--id` dirige los setpoints y parámetros, pero
no restringe el heartbeat a ese ID. Esta versión es para un banco de un eje con
**un único propietario del heartbeat**. Varias instancias o un segundo emisor de
heartbeat pueden interferir con la deshabilitación. Para un cobot de varios ejes
se necesita un dispatcher común y una política global de habilitación/parada,
además de la coordinación de trayectorias.

`armed` refleja la habilitación solicitada por este backend, no una confirmación
de que el motor está produciendo par. Los setpoints no tienen ACK en esta trama;
la posición y velocidad medidas son la evidencia del movimiento.

`disarm()` y el cierre detienen heartbeat y setpoints, siguiendo la estrategia
`heartbeat.stop()` del ejemplo. **La deshabilitación depende del timeout interno
del SPARK**, no es instantánea y no se inventa una trama de disable. Puede haber
una transmisión ya en curso al pedir `disarm`. Después del último heartbeat se
dejan 0.5 s de silencio antes de aplicar nuevos parámetros; `disable_settle_s`
permite ajustar esta pausa y no constituye una confirmación del estado físico.
El backend no asegura frenado ni sostiene una carga contra gravedad. Este
software de pruebas no sustituye el paro físico, freno ni seguridad de un cobot.

## Verificación incluida

```bash
python -m unittest -v test_backend
```

20 pruebas sobre `python-can` virtual: bytes de referencia para telemetría y
setpoint, selección de slot, habilitación de STATUS_2, unidades, repetición de
SP/heartbeat, API asíncrona con ACK lento, respuestas ajenas, NACK, tipos/valores
incorrectos, timeouts, datos corruptos, watchdog, cierre, fallo de transporte,
heartbeat del ejemplo, persistencia 0/255/error y pausa antes de reconfigurar.
El emulador representa comunicaciones; no valida dinámica, ajuste PID, par,
frenado ni comportamiento del firmware físico. No se hicieron pruebas con un
SPARK MAX real en este entorno.

## Referencias técnicas

- [Especificaciones REV y catálogo de parámetros](https://github.com/REVrobotics/REV-Specs)
- [Catálogo SparkParameters-v0.1.2](https://github.com/REVrobotics/REV-Specs/blob/main/parameters/SparkParameters-v0.1.2.md)
- [MAXMotion Position Control](https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control)
- [Unidades de los lazos de control](https://docs.revrobotics.com/revlib/spark/closed-loop/units)
- [SocketCAN en python-can](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html)

Las definiciones exactas de tramas usadas en este paquete son las del JSON adjunto.
