"""Backend de un eje: MAXMotion Position, CAN y API no bloqueante.

Python >= 3.10, python-can 4.6.1 y los dos archivos de protocolo adjuntos.
Tiempo real blando: el PID y el perfil se ejecutan en el SPARK, no en Python.
Una instancia es el UNICO propietario de CAN/heartbeat en un banco de un eje.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from queue import Queue, Empty, Full
import math
import struct
import threading
import time
from typing import Any

import can
from sparkmax_json_protocol import (
    CANPacket, ParameterDefinition, PARAMETER_TYPE_CODE, SparkMAXMotionProtocol,
)

DEFAULT_SPEC = Path(__file__).with_name("spark-frames-2.1.0")
# Valores exactos del ejemplo validado por el usuario en Raspberry Pi 5.
HEARTBEAT_ID = 0x01011840
HEARTBEAT_DATA = bytes([0xFF] * 8)


class Command:
    """Resultado consultable desde la UI. result() espera SOLO al llamador.

    No ejecuta callbacks de usuario dentro del hilo CAN. Consultar done() en
    el timer de la UI y llamar result() únicamente cuando haya terminado.
    Un timeout de result() no cancela una operación que ya fue encolada.
    """
    def __init__(self) -> None:
        self._event = threading.Event()
        self._value: dict[str, Any] = {}
        self._error: Exception | None = None

    def done(self) -> bool:
        return self._event.is_set()

    def result(self, timeout: float | None = None) -> dict[str, Any]:
        if not self._event.wait(timeout):
            raise TimeoutError("El comando sigue pendiente; no fue cancelado")
        if self._error is not None:
            raise self._error
        return dict(self._value)

    def _finish(self, value=None, error=None) -> None:
        self._value, self._error = dict(value or {}), error
        self._event.set()


@dataclass(frozen=True)
class Telemetry:
    monotonic_s: float
    sp_rot: float | None
    pv_rot: float | None
    velocity_rpm: float | None
    current_a: float | None
    error_rot: float | None
    position_age_s: float | None
    current_age_s: float | None
    position_fresh: bool
    current_fresh: bool
    armed: bool
    initialized: bool
    configuration_pending: int
    fault: str | None
    rx_frames: int
    tx_frames: int
    malformed_frames: int
    skipped_periods: int
    max_cycle_gap_s: float
    mode: str = "MAXMotion Position Control"
    primary_heartbeat_lock: bool = False


@dataclass(frozen=True)
class _Step:
    label: str
    packet: CANPacket
    response: str
    parameter: ParameterDefinition | None = None
    raw_value: int | None = None
    timeout_s: float | None = None


@dataclass
class _Job:
    steps: deque[_Step]
    command: Command
    kind: str
    keys: set[str] = field(default_factory=set)
    values: dict[str, Any] = field(default_factory=dict)


def _float32(value: float, name: str, minimum=None, positive=False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name}: se requiere un número, no bool")
    try:
        value = struct.unpack("<f", struct.pack("<f", float(value)))[0]
    except (ValueError, TypeError, OverflowError, struct.error) as exc:
        raise ValueError(f"{name}: valor no representable como float32") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name}: debe ser finito")
    if positive and value <= 0:
        raise ValueError(f"{name}: debe ser mayor que cero")
    if minimum is not None and value < minimum:
        raise ValueError(f"{name}: debe ser >= {minimum}")
    return value


def _parameter(name: str, number: int, kind: str) -> ParameterDefinition:
    return ParameterDefinition("setup", name, 0, number, kind)


class TeachPendantBackend:
    """Un SPARK, un slot y un único hilo dueño de send()/recv().

    start() -> initialize() -> PIDF/perfil -> telemetría fresca -> arm() -> move.
    initialize/set_pidf/set_motion_profile devuelven Command inmediatamente.
    La configuración se realiza desarmado; la recepción sigue activa.
    Un fallo queda enclavado: cerrar y crear otra instancia para recuperarse.
    """
    def __init__(
        self, spec_path: str | Path = DEFAULT_SPEC, *, device_id: int = 1,
        channel: str = "can0", interface: str = "socketcan", slot: int = 0,
        period_s: float = 0.020, feedback_timeout_s: float = 0.300,
        response_timeout_s: float = 0.500,
        disable_settle_s: float = 0.500,
        position_limits: tuple[float, float] | None = None,
        bus: Any = None, parameter_layout=None,
    ) -> None:
        if type(device_id) is not int or not 0 <= device_id <= 63:
            raise ValueError("device_id debe ser un entero de 0 a 63")
        if type(slot) is not int or not 0 <= slot <= 3:
            raise ValueError("slot debe ser un entero de 0 a 3")
        if not math.isfinite(period_s) or not 0.005 <= period_s <= 0.050:
            raise ValueError("period_s debe estar entre 0.005 y 0.050 s")
        if not math.isfinite(feedback_timeout_s) or feedback_timeout_s < 3 * period_s:
            raise ValueError("feedback_timeout_s debe ser >= 3 * period_s")
        if not math.isfinite(response_timeout_s) or response_timeout_s <= 0:
            raise ValueError("response_timeout_s debe ser positivo")
        if not math.isfinite(disable_settle_s) or disable_settle_s < 0:
            raise ValueError("disable_settle_s debe ser finito y no negativo")
        if position_limits is not None:
            lo, hi = map(float, position_limits)
            if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
                raise ValueError("Límites: mínimo finito < máximo finito")
            position_limits = (lo, hi)
        self.protocol = SparkMAXMotionProtocol(
            spec_path, device_id=device_id, parameter_layout=parameter_layout)
        self.slot, self.channel, self.interface = slot, channel, interface
        self.period_s, self.feedback_timeout_s = period_s, feedback_timeout_s
        self.response_timeout_s = response_timeout_s
        self.disable_settle_s = disable_settle_s
        self.position_limits = position_limits
        self._bus, self._owns_bus = bus, bus is None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False
        self._running = False
        self._jobs: Queue[_Job] = Queue(maxsize=32)
        self._job: _Job | None = None       # solamente el hilo CAN lo modifica
        self._pending: _Step | None = None
        self._deadline = 0.0
        self._job_ready_at = 0.0
        self._last_enable_tx_at: float | None = None
        self._config_count = 0
        self._initialized = self._init_submitted = self._armed = False
        self._pidf_keys: set[str] = set()
        self._profile_keys: set[str] = set()
        self._sp = self._pv = self._velocity = self._current = None
        self._position_at = self._current_at = None
        self._primary_lock = False
        self._fault: str | None = None
        self._rx_count = self._tx_count = self._malformed = self._skipped = 0
        self._max_cycle_gap = 0.0
        self._immediate_tx = False
        self._rx_frames = {}
        for name in ("STATUS_0", "STATUS_2", "PARAMETER_WRITE_RESPONSE",
                     "SET_STATUSES_ENABLED_RESPONSE", "STOP_FOLLOWER_MODE_RESPONSE",
                     "PERSIST_PARAMETERS_RESPONSE"):
            frame = self.protocol.frames[name]
            self._rx_frames[frame.arbitration_id(device_id)] = name
        # Este heartbeat no está definido en el JSON SPARK. Se conserva
        # exactamente el frame del programa que funciona en la Raspberry Pi 5.
        # Es global: no permite habilitar de forma independiente varios ejes.
        self._hb_on = CANPacket(HEARTBEAT_ID, HEARTBEAT_DATA, dlc=8,
                                frame_name="REFERENCE_HEARTBEAT")

    def start(self) -> TeachPendantBackend:
        with self._lock:
            if self._closed or self._thread is not None:
                raise RuntimeError("Instancia ya iniciada/cerrada; crea una nueva")
            if self._bus is None:
                filters = [{"can_id": key, "can_mask": 0x1FFFFFFF, "extended": True}
                           for key in self._rx_frames]
                self._bus = can.Bus(interface=self.interface, channel=self.channel,
                                    receive_own_messages=False, can_filters=filters)
            self._running = True
            self._thread = threading.Thread(target=self._run, name="spark-can", daemon=False)
            self._thread.start()
        return self

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._armed = False
            self._stop.set()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                raise RuntimeError("El transporte CAN no respetó sus timeouts")

    def __enter__(self) -> TeachPendantBackend:
        return self.start()

    def __exit__(self, *_args) -> None:
        self.close()

    def _check_running(self) -> None:
        if not self._running or self._closed:
            raise RuntimeError("Backend no iniciado o cerrado")
        if self._fault:
            raise RuntimeError(self._fault)

    def _write_step(self, parameter: ParameterDefinition, value: Any) -> _Step:
        return _Step(parameter.key, self.protocol.parameter_write_packet(parameter, value),
                     "PARAMETER_WRITE_RESPONSE", parameter,
                     self.protocol.pack_parameter_value(value, parameter.value_type))

    def _submit(self, steps: list[_Step], kind: str, keys=()) -> Command:
        with self._lock:
            self._check_running()
            if self._armed:
                raise RuntimeError("Ejecuta disarm() antes de cambiar el setup")
            if kind != "initialize" and not self._initialized:
                raise RuntimeError("Espera a que initialize() termine correctamente")
            ticket = Command()
            try:
                self._jobs.put_nowait(_Job(deque(steps), ticket, kind, set(keys)))
            except Full as exc:
                raise RuntimeError("Cola de configuración llena") from exc
            self._config_count += 1
            return ticket

    def initialize(self, status_period_ms: int = 20) -> Command:
        """Configura encoder primario, unidades y STATUS_0/2; no mueve el eje.

        Factores de posición/velocidad = 1; position wrapping = false.
        Parámetros auxiliares según la tabla REV compatible con la librería.
        Todos los cambios son RAM; no se envía PERSIST_PARAMETERS.
        """
        if type(status_period_ms) is not int or not 1 <= status_period_ms <= 1000:
            raise ValueError("status_period_ms debe ser entero entre 1 y 1000")
        if status_period_ms / 1000 * 3 > self.feedback_timeout_s:
            raise ValueError("El watchdog requiere al menos 3 períodos STATUS")
        p = self.protocol
        steps = [_Step("stop_follower", p.frames["STOP_FOLLOWER_MODE"].packet(p.device_id),
                       "STOP_FOLLOWER_MODE_RESPONSE")]
        for name, pid, kind, value in (
            ("feedback_sensor", 9, "uint", 1),  # REV kPrimaryEncoder = 1
            ("position_factor", 112, "float", 1.0),
            ("velocity_factor", 113, "float", 1.0),
            ("position_wrapping", 149, "bool", False),
            ("status0_period_ms", 158, "uint", status_period_ms),
            ("status2_period_ms", 160, "uint", status_period_ms),
        ):
            steps.append(self._write_step(_parameter(name, pid, kind), value))
        steps.append(_Step("enable_status_0_2", p.frames["SET_STATUSES_ENABLED"].packet(
            p.device_id, {"MASK": 0b101, "ENABLED_BITFIELD": 0b101}),
            "SET_STATUSES_ENABLED_RESPONSE"))
        with self._lock:
            if self._init_submitted:
                raise RuntimeError("initialize() sólo se permite una vez por instancia")
            command = self._submit(steps, "initialize")
            self._init_submitted = True
            return command

    def set_pidf(self, *, p=None, i=None, d=None, f=None) -> Command:
        """Actualiza las ganancias suministradas del slot fijo.

        f conserva el mapeo de la librería (ID 16 + 8*slot). En la tabla REV
        actual se llama kV: su interpretación/unidad depende del firmware.
        """
        values = {k: _float32(v, k, minimum=0) for k, v in
                  (("p", p), ("i", i), ("d", d), ("f", f)) if v is not None}
        if not values:
            raise ValueError("Proporciona al menos una ganancia PIDF")
        group = self.protocol["pidf"][self.slot]
        steps = [self._write_step(group[k], v) for k, v in values.items()]
        return self._submit(steps, "pidf", values)

    def set_motion_profile(self, *, maxacceleration=None, cruisevelocity=None,
                           allowed_profile_error=None) -> Command:
        """maxacceleration: RPM/s; cruisevelocity: RPM; error: rotaciones."""
        values = {}
        if maxacceleration is not None:
            values["max_acceleration"] = _float32(maxacceleration, "maxacceleration", positive=True)
        if cruisevelocity is not None:
            values["cruise_velocity"] = _float32(cruisevelocity, "cruisevelocity", positive=True)
        if allowed_profile_error is not None:
            values["allowed_profile_error"] = _float32(allowed_profile_error, "allowed_profile_error", minimum=0)
        if not values:
            raise ValueError("Proporciona al menos un parámetro de MAXMotion")
        group = self.protocol["maxmotion"][self.slot]
        steps = [self._write_step(group[k], v) for k, v in values.items()]
        return self._submit(steps, "profile", values)

    def set_maxacceleration(self, rpm_per_second: float) -> Command:
        return self.set_motion_profile(maxacceleration=rpm_per_second)

    def set_cruisevelocity(self, rpm: float) -> Command:
        return self.set_motion_profile(cruisevelocity=rpm)

    def persist_parameters(self) -> Command:
        """Guarda en flash, opcionalmente, como en el ejemplo de Raspberry Pi.

        Sólo RESULT_CODE=0 confirma. Un 255 mantiene la espera hasta 2.5 s.
        Tras confirmarse, espera 0.25 s antes de completar el Command.
        """
        frame = self.protocol.frames["PERSIST_PARAMETERS"]
        magic = int(frame.signals["MAGIC_NUMBER"]["decodedMin"])
        packet = frame.packet(self.protocol.device_id, {"MAGIC_NUMBER": magic})
        return self._submit([_Step("persist_parameters", packet,
                                    "PERSIST_PARAMETERS_RESPONSE", timeout_s=2.5)], "persist")

    def _fresh(self, timestamp, now) -> bool:
        return timestamp is not None and now - timestamp <= self.feedback_timeout_s

    def _require_feedback(self) -> None:
        now = time.monotonic()
        if not (self._fresh(self._position_at, now) and self._fresh(self._current_at, now)):
            raise RuntimeError("Se requieren STATUS_0 y STATUS_2 recientes")
        # PRIMARY_HEARTBEAT_LOCK es informativo con el heartbeat del ejemplo;
        # sólo impediría utilizar SECONDARY_HEARTBEAT, que aquí no se transmite.

    def _check_target(self, rotations: float) -> float:
        raw = float(rotations)
        value = _float32(rotations, "setpoint")
        if self.position_limits is not None:
            lo, hi = self.position_limits
            if not (lo <= raw <= hi and lo <= value <= hi):
                raise ValueError(f"SP fuera de los límites [{lo}, {hi}] rot")
        return value

    def arm(self) -> float:
        """Habilita manteniendo la PV medida; requiere setup confirmado."""
        with self._lock:
            self._check_running()
            if self._armed:
                raise RuntimeError("El eje ya está habilitado")
            if not self._initialized or self._config_count:
                raise RuntimeError("Inicialización/configuración pendiente")
            if not {"p", "i", "d", "f"} <= self._pidf_keys:
                raise RuntimeError("Configura P, I, D y F antes de arm()")
            if not {"max_acceleration", "cruise_velocity"} <= self._profile_keys:
                raise RuntimeError("Configura maxacceleration y cruisevelocity antes de arm()")
            self._require_feedback()
            self._sp = self._check_target(self._pv)
            self._armed = True
            self._immediate_tx = True
            return self._sp

    def send_setpoint(self, rotations: float) -> float:
        """Publica el último SP absoluto; el hilo envía/repite MAXMOTION_POSITION_SETPOINT.

        No espera al movimiento. Varias publicaciones entre ciclos se agrupan:
        se transmite sólo la más reciente, evitando una cola de objetivos viejos.
        Devuelve el SP cuantizado a float32 que irá por CAN.
        """
        value = self._check_target(rotations)
        with self._lock:
            self._check_running()
            if not self._armed:
                raise RuntimeError("Ejecuta arm() antes de enviar movimiento")
            self._require_feedback()
            self._sp = value
        return value

    def hold(self) -> float:
        """Nuevo objetivo = PV reciente; sigue usando el perfil MAXMotion."""
        with self._lock:
            self._require_feedback()
            return self.send_setpoint(self._pv)

    def disarm(self) -> None:
        """Detiene heartbeat/SP; la deshabilitación depende del watchdog del SPARK.

        Conserva la estrategia de stop() del ejemplo, sin inventar una trama
        de disable. No es una parada inmediata ni un freno de seguridad.
        """
        with self._lock:
            self._armed = False
            self._immediate_tx = True

    def telemetry(self) -> Telemetry:
        with self._lock:
            now = time.monotonic()
            fresh2, fresh0 = self._fresh(self._position_at, now), self._fresh(self._current_at, now)
            return Telemetry(
                now, self._sp, self._pv, self._velocity, self._current,
                self._sp - self._pv if self._sp is not None and fresh2 else None,
                now - self._position_at if self._position_at is not None else None,
                now - self._current_at if self._current_at is not None else None,
                fresh2, fresh0, self._armed, self._initialized, self._config_count,
                self._fault, self._rx_count, self._tx_count, self._malformed,
                self._skipped, self._max_cycle_gap,
                primary_heartbeat_lock=self._primary_lock,
            )

    def _send(self, packet: CANPacket) -> None:
        self._bus.send(packet.to_python_can(), timeout=0.005)
        with self._lock:
            self._tx_count += 1

    def _trip(self, reason: str) -> None:
        with self._lock:
            self._fault = self._fault or reason
            self._armed = False
            self._immediate_tx = True
        self._fail_jobs(RuntimeError(self._fault))

    def _fail_jobs(self, error: Exception) -> None:
        with self._lock:
            if self._job is not None:
                self._job.command._finish(error=error)
                self._job = None
            self._pending = None
            while True:
                try:
                    self._jobs.get_nowait().command._finish(error=error)
                except Empty:
                    break
            self._config_count = 0

    def _configuration_tick(self, now: float) -> None:
        if self._fault:
            return
        if self._pending is not None:
            if now >= self._deadline:
                self._trip(f"Timeout CAN esperando {self._pending.label}; setup puede estar parcial")
            return
        if self._job is None:
            try:
                self._job = self._jobs.get_nowait()
            except Empty:
                return
        if not self._job.steps:
            if now >= self._job_ready_at:
                self._complete_job()
            return
        self._pending = self._job.steps.popleft()
        self._send(self._pending.packet)
        self._deadline = time.monotonic() + (self._pending.timeout_s or self.response_timeout_s)

    def _complete_job(self) -> None:
        job = self._job
        if job is None:
            return
        with self._lock:
            if job.kind == "initialize":
                self._initialized = True
                self._position_at = self._current_at = None
                self._pv = self._velocity = self._current = None
            elif job.kind == "pidf":
                self._pidf_keys.update(job.keys)
            elif job.kind == "profile":
                self._profile_keys.update(job.keys)
            self._config_count -= 1
        self._job = None
        job.command._finish(job.values)

    def _ack(self, name: str, data: bytes, decoded: dict) -> None:
        step, job = self._pending, self._job
        if step is None or job is None or step.response != name:
            return
        if step.parameter is not None:
            response = self.protocol.decode_parameter_write_response(data)
            if response["parameter_id"] != step.parameter.parameter_id:
                return
            expected_type = PARAMETER_TYPE_CODE[step.parameter.value_type]
            if not response["success"]:
                self._trip(f"{step.label}: PARAMETER_WRITE rechazado, código {response['result_code']}")
                return
            if (response["parameter_type_code"] != expected_type or
                    int(decoded["VALUE"]) != step.raw_value):
                self._trip(f"{step.label}: tipo/valor confirmado no coincide con la escritura")
                return
        elif name == "SET_STATUSES_ENABLED_RESPONSE":
            if int(decoded["SPECIFIED_MASK"]) != 0b101:
                return
            if int(decoded["RESULT_CODE"]) != 0 or int(decoded["ENABLED_BITFIELD"]) & 0b101 != 0b101:
                self._trip("No se pudieron habilitar STATUS_0 y STATUS_2")
                return
            response = decoded
        elif name == "PERSIST_PARAMETERS_RESPONSE":
            result = int(decoded["RESULT_CODE"])
            if result == 255:
                # No es éxito: conservar la misma solicitud y su deadline.
                return
            if result != 0:
                self._trip(f"Persistencia rechazada: RESULT_CODE={result}")
                return
            response = decoded
        else:
            response = {"success": True}
        job.values[step.label] = response
        self._pending = None
        if not job.steps:
            if job.kind == "persist":
                self._job_ready_at = time.monotonic() + 0.25
            else:
                self._complete_job()

    def _receive(self, msg) -> None:
        if msg.is_error_frame:
            self._trip("Trama de error CAN recibida")
            return
        if not msg.is_extended_id or msg.is_remote_frame:
            return
        name = self._rx_frames.get(msg.arbitration_id)
        if name is None:
            return
        try:
            data = bytes(msg.data)
            decoded = self.protocol.frames[name].decode_payload(data)
            if any(isinstance(v, float) and not math.isfinite(v) for v in decoded.values()):
                raise ValueError("Telemetría no finita")
        except (ValueError, struct.error):
            with self._lock:
                self._malformed += 1
            return
        with self._lock:
            now = time.monotonic()
            self._rx_count += 1
            if name == "STATUS_2":
                self._pv = float(decoded["PRIMARY_ENCODER_POSITION"])
                self._velocity = float(decoded["PRIMARY_ENCODER_VELOCITY"])
                self._position_at = now
            elif name == "STATUS_0":
                self._current = float(decoded["CURRENT"])
                self._current_at = now
                self._primary_lock = bool(decoded["PRIMARY_HEARTBEAT_LOCK"])
        self._ack(name, data, decoded)

    def _run(self) -> None:
        next_cycle = time.monotonic()
        last_cycle = None
        was_armed = False
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                with self._lock:
                    if self._armed and not (self._fresh(self._position_at, now) and
                                           self._fresh(self._current_at, now)):
                        self._trip("Watchdog: STATUS_0 o STATUS_2 sin actualizar")
                    armed, sp = self._armed, self._sp
                    immediate = self._immediate_tx
                    self._immediate_tx = False
                if now >= next_cycle or immediate or armed != was_armed:
                    if last_cycle is not None:
                        with self._lock:
                            self._max_cycle_gap = max(self._max_cycle_gap, now - last_cycle)
                    last_cycle = now
                    if armed:
                        # Siempre cargar el SP antes de habilitar, para no revivir
                        # un objetivo de una sesión anterior del controlador.
                        self._send(self.protocol.maxmotion_setpoint_packet(sp, slot=self.slot))
                        self._send(self._hb_on)
                        self._last_enable_tx_at = time.monotonic()
                    was_armed = armed
                    if now >= next_cycle:
                        periods = int((now - next_cycle) / self.period_s) + 1
                        next_cycle += periods * self.period_s
                        with self._lock:
                            self._skipped += periods - 1
                # Después de detener heartbeat, respetar una ventana de silencio
                # antes de escribir parámetros (no confirma el estado físico).
                now = time.monotonic()
                quiet = (self._last_enable_tx_at is None or
                         now - self._last_enable_tx_at >= self.disable_settle_s)
                if not was_armed and quiet:
                    self._configuration_tick(now)
                # Un solo consumidor; un frame por iteración mantiene prioridad
                # de heartbeat/watchdog incluso si el bus tiene mucho tráfico.
                timeout = min(0.005, max(0.0, next_cycle - time.monotonic()))
                msg = self._bus.recv(timeout=timeout)
                if msg is not None:
                    self._receive(msg)
        except Exception as exc:
            self._trip(f"Fallo del hilo CAN: {type(exc).__name__}: {exc}")
        finally:
            with self._lock:
                self._armed = False
                self._running = False
            self._fail_jobs(RuntimeError(self._fault or "Backend cerrado"))
            # Igual que heartbeat.stop() en el ejemplo: dejar de transmitir.
            # El watchdog del firmware determina cuándo se deshabilita el motor.
            if self._owns_bus and self._bus is not None:
                self._bus.shutdown()
