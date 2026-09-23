"""teach_pendant_backend.py

Backend de control en tiempo real para un teach pendant de un cobot,
construido sobre `sparkmax_json_protocol.py` (REV SPARK MAX, JSON de frames
CAN oficial de REV Robotics).

Alcance de este módulo (según lo solicitado):
    - Solo MAXMotion Position Control (no hay velocity/duty cycle control aquí).
    - Un hilo en tiempo real que:
        1. drena y decodifica los frames STATUS_0 / STATUS_2 entrantes,
        2. reenvía continuamente el último Setpoint (SP) comandado vía
           MAXMOTION_POSITION_SETPOINT (igual que un lazo de control real:
           el setpoint se debe refrescar periódicamente, no enviarse una sola vez),
        3. despacha las respuestas PARAMETER_WRITE_RESPONSE hacia quien esté
           esperando esa escritura de parámetro (PIDF / perfil MAXMotion).
    - Getters de telemetría: posición (PV), velocidad, corriente y error de
      seguimiento (SP - PV), thread-safe.

Heartbeat:
    `sparkmax_json_protocol.py` indica explícitamente que no es dueño del
    heartbeat: "Keep using the heartbeat mechanism already validated in your
    application." Este módulo respeta eso pero ya no lo deja como tarea
    pendiente: trae integrado, por defecto, el mecanismo ya validado
    (ID 0x01011840, payload 0xFF x8, cada ~20 ms) para que el bucle en
    tiempo real lo dispare por su cuenta, en el mismo hilo. Si prefieres tu
    propia implementación (u otra ya probada), pasa `heartbeat_fn` y esa
    toma prioridad; con `enable_heartbeat=False` se desactiva por completo
    (por ejemplo si otro proceso ya lo está enviando).

Persistencia de parámetros:
    `persist_parameters()` replica el flujo ya validado: envía
    PERSIST_PARAMETERS con el MAGIC_NUMBER que trae el propio JSON, y espera
    PERSIST_PARAMETERS_RESPONSE. RESULT_CODE == 0 es éxito; 0xFF no está
    documentado como éxito así que se sigue esperando otra respuesta hasta
    agotar el timeout; cualquier otro código se trata como error inmediato.

Nota de concurrencia:
    El propio protocolo advierte que `write_parameter()` no debe usarse desde
    múltiples consumidores llamando `bus.recv()` a la vez. Por eso este
    backend NO usa `protocol.write_parameter()` ni `protocol.configure_slot()`
    directamente: arma cada frame con los builders de bajo nivel
    (`parameter_write_packet`, frame `.packet()`), lo envía, y deja que el
    único lector (el hilo en tiempo real) despache la respuesta
    correspondiente (PARAMETER_WRITE_RESPONSE o PERSIST_PARAMETERS_RESPONSE)
    hacia quien esté esperando.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Dict, Optional

try:
    import can
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "python-can no está instalado. Instálalo con `pip install python-can` "
        "antes de usar teach_pendant_backend."
    ) from exc

from sparkmax_json_protocol import ParameterDefinition, SparkMAXMotionProtocol

logger = logging.getLogger(__name__)


class SparkMaxTeachPendant:
    """Backend de teach pendant para un eje SPARK MAX en MAXMotion Position Control.

    Ejemplo de uso programático
    ----------------------------
        pendant = SparkMaxTeachPendant(
            "spark-frames-2_1.0",
            device_id=1,
            can_channel="can0",
        )
        pendant.start()

        pendant.set_motion_profile(max_acceleration=8, cruise_velocity=5)
        pendant.set_pidf(p=0.05, i=0.0, d=0.0, f=0.0)
        pendant.persist_parameters()  # opcional: guarda en la memoria del SPARK

        pendant.move_to(2.5)          # rotaciones
        time.sleep(2.0)
        print(pendant.snapshot())

        pendant.stop()

    O como context manager:
        with SparkMaxTeachPendant("spark-frames-2_1.0", device_id=1) as pendant:
            pendant.move_to(1.0)
            time.sleep(1.0)
    """

    # Heartbeat validado: broadcast fijo, no depende del device_id.
    HEARTBEAT_ARBITRATION_ID = 0x01011840
    HEARTBEAT_DATA = bytes([0xFF] * 8)

    def __init__(
        self,
        json_path: str,
        *,
        device_id: int = 1,
        can_channel: str = "can0",
        can_interface: str = "socketcan",
        can_kwargs: Optional[Dict[str, Any]] = None,
        loop_hz: float = 50.0,
        default_slot: int = 0,
        enable_heartbeat: bool = True,
        heartbeat_fn: Optional[Callable[[], None]] = None,
        heartbeat_hz: float = 50.0,
        param_write_timeout: float = 1.0,
        persist_timeout: float = 2.5,
    ) -> None:
        if not 0 <= device_id <= 63:
            raise ValueError("device_id debe estar entre 0 y 63")
        if loop_hz <= 0:
            raise ValueError("loop_hz debe ser positivo")

        self.device_id = int(device_id)
        self.default_slot = int(default_slot)
        self.param_write_timeout = float(param_write_timeout)
        self.persist_timeout = float(persist_timeout)

        self.protocol = SparkMAXMotionProtocol(json_path, device_id=self.device_id)

        self.bus = can.Bus(
            channel=can_channel, interface=can_interface, **(can_kwargs or {})
        )

        # Frames/arbitration IDs propios de este device_id, precalculados una vez.
        self._status0_frame = self.protocol.frames["STATUS_0"]
        self._status2_frame = self.protocol.frames["STATUS_2"]
        self._status0_id = self._status0_frame.arbitration_id(self.device_id)
        self._status2_id = self._status2_frame.arbitration_id(self.device_id)
        self._param_resp_id = self.protocol.frames["PARAMETER_WRITE_RESPONSE"].arbitration_id(
            self.device_id
        )
        self._persist_frame = self.protocol.frames["PERSIST_PARAMETERS"]
        self._persist_resp_frame = self.protocol.frames["PERSIST_PARAMETERS_RESPONSE"]
        self._persist_resp_id = self._persist_resp_frame.arbitration_id(self.device_id)
        self._persist_magic_number = int(
            self._persist_frame["signals"]["MAGIC_NUMBER"]["decodedMin"]
        )

        self._loop_period = 1.0 / float(loop_hz)
        if heartbeat_fn is not None:
            self._heartbeat_fn: Optional[Callable[[], None]] = heartbeat_fn
        elif enable_heartbeat:
            self._heartbeat_fn = self._send_default_heartbeat
        else:
            self._heartbeat_fn = None
        self._heartbeat_period = 1.0 / float(heartbeat_hz) if self._heartbeat_fn else None

        # --- Telemetría (PV, velocidad, corriente) ---------------------------
        self._telemetry_lock = threading.Lock()
        self._position = 0.0
        self._velocity = 0.0
        self._current = 0.0
        self._telemetry_ts: Optional[float] = None

        # --- Setpoint activo (comando de movimiento) --------------------------
        self._setpoint_lock = threading.Lock()
        self._setpoint = 0.0
        self._active_slot = self.default_slot
        self._feedforward = 0.0
        self._feedforward_units = 0
        self._has_target = False

        # --- Despacho de escritura de parámetros (PIDF / MAXMotion) -----------
        self._write_lock = threading.Lock()  # serializa transacciones completas
        self._param_state_lock = threading.Lock()
        self._param_event = threading.Event()
        self._pending_param_id: Optional[int] = None
        self._pending_response: Optional[Dict[str, Any]] = None

        # --- Despacho de PERSIST_PARAMETERS_RESPONSE ---------------------------
        self._persist_event = threading.Event()
        self._awaiting_persist = False
        self._persist_results: list[int] = []

        # --- Control del hilo en tiempo real ------------------------------------
        self._stop_evt = threading.Event()
        self._stop_evt.set()
        self._thread: Optional[threading.Thread] = None

    def _send_default_heartbeat(self) -> None:
        """Heartbeat ya validado: ID 0x01011840, payload 0xFF x8, ~cada 20 ms."""
        msg = can.Message(
            arbitration_id=self.HEARTBEAT_ARBITRATION_ID,
            data=self.HEARTBEAT_DATA,
            is_extended_id=True,
        )
        self.bus.send(msg)

    # ------------------------------------------------------------------------
    # Ciclo de vida del hilo en tiempo real
    # ------------------------------------------------------------------------

    def start(self) -> None:
        """Arranca el bucle en tiempo real (RX/TX) en un hilo daemon."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._rt_loop, name="SparkMaxTeachPendantRT", daemon=True
        )
        self._thread.start()

    def stop(self, *, shutdown_bus: bool = True) -> None:
        """Detiene el bucle en tiempo real y opcionalmente cierra el bus CAN."""
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if shutdown_bus:
            try:
                self.bus.shutdown()
            except Exception:
                logger.exception("Error cerrando el bus CAN")

    def __enter__(self) -> "SparkMaxTeachPendant":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------------
    # Bucle en tiempo real
    # ------------------------------------------------------------------------

    def _rt_loop(self) -> None:
        next_heartbeat = time.monotonic()
        while not self._stop_evt.is_set():
            loop_start = time.monotonic()

            # 1) Drenar todos los frames entrantes disponibles sin bloquear.
            while True:
                try:
                    msg = self.bus.recv(timeout=0.0)
                except Exception:
                    logger.exception("Error leyendo el bus CAN")
                    break
                if msg is None:
                    break
                try:
                    self._handle_rx(msg)
                except Exception:
                    logger.exception("Error procesando frame CAN recibido")

            # 2) Heartbeat (mecanismo ya validado por la aplicación, si se provee).
            if self._heartbeat_fn is not None:
                now = time.monotonic()
                if now >= next_heartbeat:
                    try:
                        self._heartbeat_fn()
                    except Exception:
                        logger.exception("heartbeat_fn lanzó una excepción")
                    next_heartbeat = now + self._heartbeat_period

            # 3) Reenviar el Setpoint activo (control de posición en lazo cerrado).
            has_target, sp, slot, ff, ff_units = self._read_active_target()
            if has_target:
                try:
                    self.protocol.send_setpoint(
                        self.bus,
                        sp,
                        slot=slot,
                        arbitrary_feedforward=ff,
                        arbitrary_feedforward_units=ff_units,
                    )
                except Exception:
                    logger.exception("Error enviando MAXMOTION_POSITION_SETPOINT")

            # 4) Mantener la tasa del bucle.
            elapsed = time.monotonic() - loop_start
            remaining = self._loop_period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _handle_rx(self, msg: Any) -> None:
        if not getattr(msg, "is_extended_id", True):
            return

        arb_id = msg.arbitration_id
        data = bytes(msg.data)

        if arb_id == self._status2_id:
            decoded = self._status2_frame.decode_payload(data)
            with self._telemetry_lock:
                self._position = decoded["PRIMARY_ENCODER_POSITION"]
                self._velocity = decoded["PRIMARY_ENCODER_VELOCITY"]
                self._telemetry_ts = time.monotonic()

        elif arb_id == self._status0_id:
            decoded = self._status0_frame.decode_payload(data)
            with self._telemetry_lock:
                self._current = decoded["CURRENT"]

        elif arb_id == self._param_resp_id:
            decoded = self.protocol.decode_parameter_write_response(data)
            with self._param_state_lock:
                if (
                    self._pending_param_id is not None
                    and decoded.get("parameter_id") == self._pending_param_id
                ):
                    self._pending_response = decoded
                    self._param_event.set()

        elif arb_id == self._persist_resp_id:
            decoded = self._persist_resp_frame.decode_payload(data)
            with self._param_state_lock:
                if self._awaiting_persist:
                    self._persist_results.append(int(decoded["RESULT_CODE"]))
                    self._persist_event.set()

    def _read_active_target(self):
        with self._setpoint_lock:
            return (
                self._has_target,
                self._setpoint,
                self._active_slot,
                self._feedforward,
                self._feedforward_units,
            )

    # ------------------------------------------------------------------------
    # Control de movimiento (MAXMotion Position Control)
    # ------------------------------------------------------------------------

    def move_to(
        self,
        position_rotations: float,
        *,
        slot: Optional[int] = None,
        arbitrary_feedforward: float = 0.0,
        arbitrary_feedforward_units: int = 0,
    ) -> None:
        """Comanda un nuevo Setpoint (SP) de posición, en rotaciones.

        El bucle en tiempo real reenvía este Setpoint en cada ciclo (como un
        control de posición real) hasta el próximo `move_to()`, `hold()` o
        `release()`.
        """
        with self._setpoint_lock:
            self._setpoint = float(position_rotations)
            self._active_slot = self.default_slot if slot is None else int(slot)
            self._feedforward = float(arbitrary_feedforward)
            self._feedforward_units = int(arbitrary_feedforward_units)
            self._has_target = True

    def hold(self) -> None:
        """Congela el eje: usa la posición medida actual (PV) como nuevo SP."""
        self.move_to(self.get_position(), slot=self._active_slot)

    def release(self) -> None:
        """Deja de reenviar un Setpoint (deja de comandar posición)."""
        with self._setpoint_lock:
            self._has_target = False

    # ------------------------------------------------------------------------
    # Configuración (PIDF y perfil de movimiento MAXMotion)
    # ------------------------------------------------------------------------

    def set_pidf(
        self,
        *,
        p: Optional[float] = None,
        i: Optional[float] = None,
        d: Optional[float] = None,
        f: Optional[float] = None,
        slot: Optional[int] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Dict[str, Any]]:
        """Actualiza únicamente las ganancias PIDF indicadas, vía PARAMETER_WRITE."""
        slot = self.default_slot if slot is None else slot
        group = self.protocol["pidf"][slot]
        supplied = {k: v for k, v in (("p", p), ("i", i), ("d", d), ("f", f)) if v is not None}
        return self._write_parameters(group, supplied, timeout)

    def set_motion_profile(
        self,
        *,
        max_acceleration: Optional[float] = None,
        cruise_velocity: Optional[float] = None,
        allowed_profile_error: Optional[float] = None,
        slot: Optional[int] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, Dict[str, Any]]:
        """Actualiza aceleración máxima, velocidad de crucero y/o el error de
        perfil/lazo cerrado permitido (rotaciones) de MAXMotion."""
        slot = self.default_slot if slot is None else slot
        group = self.protocol["maxmotion"][slot]
        supplied = {
            k: v
            for k, v in (
                ("max_acceleration", max_acceleration),
                ("cruise_velocity", cruise_velocity),
                ("allowed_profile_error", allowed_profile_error),
            )
            if v is not None
        }
        return self._write_parameters(group, supplied, timeout)

    def configure_and_persist(
        self,
        *,
        pidf: Optional[Dict[str, float]] = None,
        maxmotion: Optional[Dict[str, float]] = None,
        slot: Optional[int] = None,
        timeout: Optional[float] = None,
        persist: bool = True,
        persist_timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Escribe PIDF y/o parámetros MAXMotion, valida que cada uno haya sido
        aceptado con el valor correcto (`success` y `value_matches`) y, solo si
        todos lo fueron, llama `persist_parameters()`. Replica el flujo ya
        validado: configurar -> verificar -> persistir -> (luego) heartbeat y
        setpoint, con la diferencia de que aquí el heartbeat ya corre solo
        dentro del bucle en tiempo real desde `start()`.
        """
        slot = self.default_slot if slot is None else slot
        results: Dict[str, Dict[str, Any]] = {}
        if pidf:
            results.update(self.set_pidf(slot=slot, timeout=timeout, **pidf))
        if maxmotion:
            results.update(self.set_motion_profile(slot=slot, timeout=timeout, **maxmotion))

        all_ok = bool(results) and all(
            r.get("success") and r.get("value_matches") for r in results.values()
        )

        persisted = False
        if persist and all_ok:
            persisted = self.persist_parameters(
                timeout=self.persist_timeout if persist_timeout is None else persist_timeout
            )

        return {"results": results, "all_ok": all_ok, "persisted": persisted}

    def _write_parameters(
        self, group, supplied: Dict[str, float], timeout: Optional[float]
    ) -> Dict[str, Dict[str, Any]]:
        results: Dict[str, Dict[str, Any]] = {}
        for key, value in supplied.items():
            parameter = group[key]
            results[key] = self._write_parameter_sync(parameter, value, timeout)
        return results

    def _write_parameter_sync(
        self,
        parameter: ParameterDefinition,
        value: float,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Envía PARAMETER_WRITE y espera su PARAMETER_WRITE_RESPONSE.

        A diferencia de `SparkMAXMotionProtocol.write_parameter()`, esto NO
        hace su propio `bus.recv()`: se apoya en el único lector central (el
        hilo en tiempo real) para no competir por frames del bus.
        """
        if not self.is_running:
            raise RuntimeError(
                "El bucle en tiempo real no está corriendo; llama start() primero."
            )
        timeout = self.param_write_timeout if timeout is None else timeout

        with self._write_lock:
            self._param_event.clear()
            with self._param_state_lock:
                self._pending_param_id = parameter.parameter_id
                self._pending_response = None

            packet = self.protocol.parameter_write_packet(parameter, value)
            self.bus.send(packet.to_python_can())

            got_response = self._param_event.wait(timeout)

            with self._param_state_lock:
                response = self._pending_response
                self._pending_param_id = None

        if not got_response or response is None:
            raise TimeoutError(
                f"Timeout esperando PARAMETER_WRITE_RESPONSE para "
                f"parameter_id={parameter.parameter_id} ({parameter.description})"
            )

        response = dict(response)
        response["requested_value"] = value
        response["parameter"] = parameter
        if response.get("success"):
            current = response.get("current_value")
            if parameter.value_type == "float":
                response["value_matches"] = math.isclose(
                    float(current), float(value), rel_tol=1e-5, abs_tol=1e-7
                )
            else:
                response["value_matches"] = current == value
        else:
            response["value_matches"] = None
        return response

    # ------------------------------------------------------------------------
    # Persistencia de parámetros
    # ------------------------------------------------------------------------

    def persist_parameters(self, *, timeout: Optional[float] = None) -> bool:
        """Envía PERSIST_PARAMETERS (con el MAGIC_NUMBER del propio JSON) y
        espera confirmación.

        RESULT_CODE == 0 -> éxito confirmado.
        RESULT_CODE == 0xFF -> no documentado como éxito; se sigue esperando
        otra respuesta hasta agotar el timeout.
        Cualquier otro código -> error, retorna False de inmediato.
        """
        if not self.is_running:
            raise RuntimeError(
                "El bucle en tiempo real no está corriendo; llama start() primero."
            )
        timeout = self.persist_timeout if timeout is None else timeout

        packet = self._persist_frame.packet(
            self.device_id, {"MAGIC_NUMBER": self._persist_magic_number}
        )

        with self._write_lock:
            with self._param_state_lock:
                self._awaiting_persist = True
                self._persist_results = []
            self._persist_event.clear()

            self.bus.send(packet.to_python_can())

            deadline = time.monotonic() + timeout
            success = False
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                got = self._persist_event.wait(remaining)
                if not got:
                    break
                self._persist_event.clear()
                with self._param_state_lock:
                    codes = list(self._persist_results)
                    self._persist_results = []

                stop = False
                for code in codes:
                    if code == 0:
                        success = True
                        stop = True
                        break
                    if code == 0xFF:
                        logger.warning(
                            "PERSIST_PARAMETERS_RESPONSE = 0xFF (no documentado "
                            "como éxito); esperando otra respuesta."
                        )
                        continue
                    logger.error("PERSIST_PARAMETERS_RESPONSE = %s (error)", code)
                    stop = True
                    break
                if stop:
                    break

            with self._param_state_lock:
                self._awaiting_persist = False

        return success

    # ------------------------------------------------------------------------
    # Telemetría
    # ------------------------------------------------------------------------

    def get_position(self) -> float:
        """Posición actual (PV), leída de PRIMARY_ENCODER_POSITION en STATUS_2."""
        with self._telemetry_lock:
            return self._position

    def get_velocity(self) -> float:
        """Velocidad actual, leída de PRIMARY_ENCODER_VELOCITY en STATUS_2."""
        with self._telemetry_lock:
            return self._velocity

    def get_current(self) -> float:
        """Consumo de corriente (A), leído de CURRENT en STATUS_0."""
        with self._telemetry_lock:
            return self._current

    def get_setpoint(self) -> Optional[float]:
        """Último Setpoint (SP) comandado, o None si no hay uno activo."""
        with self._setpoint_lock:
            return self._setpoint if self._has_target else None

    def get_tracking_error(self) -> Optional[float]:
        """Error de seguimiento = SP - PV. None si no hay Setpoint activo."""
        sp = self.get_setpoint()
        if sp is None:
            return None
        return sp - self.get_position()

    def snapshot(self) -> Dict[str, Any]:
        """Foto instantánea de toda la telemetría, thread-safe."""
        with self._telemetry_lock:
            position = self._position
            velocity = self._velocity
            current = self._current
            ts = self._telemetry_ts
        sp = self.get_setpoint()
        return {
            "position": position,
            "velocity": velocity,
            "current": current,
            "setpoint": sp,
            "tracking_error": (sp - position) if sp is not None else None,
            "telemetry_age_s": (time.monotonic() - ts) if ts is not None else None,
        }


def _parse_kv(tokens) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for tok in tokens:
        if "=" in tok:
            key, value = tok.split("=", 1)
            result[key] = value
    return result


# -----------------------------------------------------------------------------
# CLI de pruebas desde terminal
# -----------------------------------------------------------------------------

def _run_cli() -> None:
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    parser = argparse.ArgumentParser(
        description="Prueba de terminal para el backend de teach pendant (SPARK MAX, MAXMotion Position Control)."
    )
    parser.add_argument("--json", default="spark-frames-2_1.0", help="Ruta al JSON de frames CAN de REV")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--slot", type=int, default=0)
    parser.add_argument("--loop-hz", type=float, default=50.0)
    parser.add_argument(
        "--no-heartbeat",
        action="store_true",
        help="No enviar heartbeat (por ejemplo si otro proceso ya lo está enviando)",
    )
    parser.add_argument("--heartbeat-hz", type=float, default=50.0)
    args = parser.parse_args()

    pendant = SparkMaxTeachPendant(
        args.json,
        device_id=args.device_id,
        can_channel=args.channel,
        can_interface=args.interface,
        default_slot=args.slot,
        loop_hz=args.loop_hz,
        enable_heartbeat=not args.no_heartbeat,
        heartbeat_hz=args.heartbeat_hz,
    )

    help_text = """
Comandos disponibles:
  mv <rotaciones>                  Envía un nuevo Setpoint de posición (rotaciones)
  hold                             Mantiene la posición actual como Setpoint
  release                          Deja de comandar posición (libera el setpoint)
  pid p=<v> i=<v> d=<v> f=<v>      Actualiza ganancias PIDF (solo las indicadas)
  profile acc=<v> cruise=<v> err=<v>  Perfil MAXMotion (aceleración / crucero / error permitido)
  persist                          Envía PERSIST_PARAMETERS y espera confirmación
  status                           Muestra telemetría actual
  watch <segundos>                 Imprime telemetría en vivo durante N segundos
  help                             Muestra esta ayuda
  exit                             Detiene el bucle y sale
"""
    print(f"Conectado a device_id={args.device_id} en {args.channel} ({args.interface}).")
    print("Iniciando bucle en tiempo real... (Ctrl+C para salir)")
    pendant.start()
    print(help_text)

    try:
        while True:
            try:
                raw = input("teach-pendant> ").strip()
            except EOFError:
                break
            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()
            try:
                if cmd in ("exit", "quit"):
                    break
                elif cmd == "help":
                    print(help_text)
                elif cmd == "mv":
                    pos = float(parts[1])
                    pendant.move_to(pos)
                    print(f"Setpoint enviado: {pos:.4f} rot (slot {pendant._active_slot})")
                elif cmd == "hold":
                    pendant.hold()
                    print(f"Manteniendo posición actual como setpoint: {pendant.get_setpoint():.4f} rot")
                elif cmd == "release":
                    pendant.release()
                    print("Setpoint liberado; ya no se reenvía posición.")
                elif cmd == "pid":
                    kv = _parse_kv(parts[1:])
                    kwargs = {k: float(v) for k, v in kv.items() if k in ("p", "i", "d", "f")}
                    result = pendant.set_pidf(**kwargs)
                    print("PIDF actualizado:", result)
                elif cmd == "profile":
                    kv = _parse_kv(parts[1:])
                    kwargs = {}
                    if "acc" in kv:
                        kwargs["max_acceleration"] = float(kv["acc"])
                    if "cruise" in kv:
                        kwargs["cruise_velocity"] = float(kv["cruise"])
                    if "err" in kv:
                        kwargs["allowed_profile_error"] = float(kv["err"])
                    result = pendant.set_motion_profile(**kwargs)
                    print("Perfil MAXMotion actualizado:", result)
                elif cmd == "persist":
                    ok = pendant.persist_parameters()
                    print("Persistencia confirmada." if ok else "No se confirmó la persistencia.")
                elif cmd == "status":
                    print(pendant.snapshot())
                elif cmd == "watch":
                    secs = float(parts[1]) if len(parts) > 1 else 3.0
                    end = time.monotonic() + secs
                    while time.monotonic() < end:
                        print(pendant.snapshot())
                        time.sleep(0.1)
                else:
                    print(f"Comando no reconocido: {cmd!r}. Escribe 'help'.")
            except (IndexError, ValueError) as exc:
                print(f"Error en el comando: {exc}")
            except TimeoutError as exc:
                print(f"Timeout: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        print("Deteniendo bucle en tiempo real...")
        pendant.stop()


if __name__ == "__main__":
    _run_cli()
