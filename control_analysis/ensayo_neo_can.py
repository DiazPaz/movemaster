#!/usr/bin/env python3
"""Identificación en lazo abierto: NEO + SPARK MAX + Raspberry Pi (SocketCAN).

Ejemplo:
    python3 ensayo_neo_can.py --channel can0 --device-id 1 --step-seconds 4

Secuencia predeterminada: 0, 5, 10, ..., 100 % (4 s cada uno), 0 % (4 s).
Necesita python-can; la interfaz CAN debe estar configurada previamente.
Basado en test(1).py y spark-frames-2.1.0 suministrados por el usuario.
Los nuevos STATUS_0/2 requieren firmware compatible (JSON: desde 25.0.0).

CONFIGURACIÓN PREVIA EN REV HARDWARE CLIENT:
  - Brushless, encoder primario del NEO.
  - Factores de conversión posición/velocidad = 1 para rotaciones/RPM.
  - Rampa de lazo abierto = 0 si se desea identificar sin rampa añadida.
  - Mantener y anotar límites de corriente, inversión, Brake/Coast y filtros.
Este programa conserva esos ajustes y los períodos CAN existentes; habilitar
STATUS_2 NO configura su período. El JSON indica 20 ms como predeterminado
para STATUS_2 y 10 ms para STATUS_0. resumen.csv mide lo realmente recibido.

CSV separados: comandos.csv, status_0.csv, status_2.csv, resumen.csv.
No se interpolan muestras ni se fusionan tramas de instantes distintos.
t_rx_s: timestamp de recepción SocketCAN relativo al inicio (reloj Unix).
t_host_s: instante en que Python lee la trama (reloj monotónico).
Comandos: tiempos antes/después de bus.send; no son confirmación de actuación.
STATUS_0 proporciona el duty realmente reportado por el SPARK.
Los tiempos de recepción no son timestamps internos de muestreo del encoder.

Heartbeat universal conservado de test.py: habilita todos los dispositivos
que lo respetan. Ejecutar como único emisor de órdenes del banco de ensayo.
Ctrl+C/SIGTERM/error: intenta enviar cero y deja de emitir heartbeat.
La parada depende de la comunicación/controlador; no sustituye un paro físico.
"""

import argparse
from contextlib import ExitStack
import csv
from datetime import datetime
import math
from pathlib import Path
import signal
import statistics
import struct
import time

import can


# arbId BASE del JSON; los seis bits inferiores contienen el Device ID.
DUTY_BASE = 0x02050080
ENABLE_BASE = 0x02050400
ENABLE_REPLY_BASE = 0x02050440
STATUS_0_BASE = 0x0205B800
STATUS_2_BASE = 0x0205B880
HEARTBEAT_ID = 0x01011840             # Del test.py original, no del JSON.
HEARTBEAT_DATA = bytes([0xFF] * 8)
TX_PERIOD_S = 0.020                 # Heartbeat/refresco duty, NO muestreo.
TELEMETRY_TIMEOUT_S = 0.500
MAX_STEP_LATENESS_S = 0.100


def message(arb_id, data):
    return can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)


def send_duty(bus, device_id, duty):
    if not math.isfinite(duty) or not 0.0 <= duty <= 1.0:
        raise ValueError("Duty fuera de [0, 1]")
    # float32 setpoint, int16 FF=0, uint16 configuración=0 (little-endian).
    bus.send(message(DUTY_BASE | device_id, struct.pack("<fhH", duty, 0, 0)),
             timeout=0.02)


def heartbeat(bus):
    bus.send(message(HEARTBEAT_ID, HEARTBEAT_DATA), timeout=0.02)


def enable_statuses(bus, device_id):
    """Mantiene cero mientras espera ACK; modifica únicamente bits 0 y 2."""
    mask = (1 << 0) | (1 << 2)
    send_duty(bus, device_id, 0.0)  # Cero ANTES de habilitar el motor.
    heartbeat(bus)
    bus.send(message(ENABLE_BASE | device_id, struct.pack("<HH", mask, mask)),
             timeout=0.02)
    deadline = time.monotonic() + 1.0
    next_tx = time.monotonic() + TX_PERIOD_S
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_tx:
            send_duty(bus, device_id, 0.0)
            heartbeat(bus)
            next_tx = now + TX_PERIOD_S
        msg = bus.recv(timeout=max(0.0, min(next_tx, deadline) - time.monotonic()))
        if msg is None:
            continue
        if msg.is_error_frame:
            raise RuntimeError("Error CAN durante la configuración")
        if (not msg.is_extended_id or msg.is_remote_frame
                or msg.arbitration_id != (ENABLE_REPLY_BASE | device_id)):
            continue
        if len(msg.data) != 5:
            raise RuntimeError("Longitud inválida en respuesta de habilitación")
        result, specified, enabled = struct.unpack("<BHH", msg.data)
        if result != 0 or specified != mask or (enabled & mask) != mask:
            raise RuntimeError(f"Habilitación rechazada: {result=}, {enabled=:#06x}")
        return
    raise RuntimeError("Sin confirmación de habilitación de STATUS_0 y STATUS_2")


def decode_status_0(data):
    if len(data) != 8:
        raise ValueError("STATUS_0 debe contener 8 bytes")
    raw = int.from_bytes(data, "little")
    output = struct.unpack_from("<h", data)[0] * 0.00003082369457075716
    return [output,
            ((raw >> 16) & 0xFFF) * 0.0073260073260073,
            ((raw >> 28) & 0xFFF) * 0.0366300366300366,
            (raw >> 40) & 0xFF,
            *[(raw >> bit) & 1 for bit in range(48, 54)]]


def decode_status_2(data):
    if len(data) != 8:
        raise ValueError("STATUS_2 debe contener 8 bytes")
    values = struct.unpack("<ff", data)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Encoder contiene NaN/inf")
    return values


def build_schedule(step_seconds, final_seconds, max_duty):
    steps = [(i * step_seconds, pct / 100.0)
             for i, pct in enumerate(range(0, max_duty + 1, 5))]
    end_steps = len(steps) * step_seconds
    steps.append((end_steps, 0.0))
    return steps, end_steps + final_seconds


def stop_motor(bus, device_id):
    """No hay hilo de heartbeat que pueda seguir habilitando tras el error."""
    for _ in range(3):
        try:
            send_duty(bus, device_id, 0.0)
        except Exception as exc:
            print(f"No se pudo enviar cero: {exc}")
        time.sleep(0.02)


def write_summary(path, args, state, reason):
    rows = [("result", reason), ("channel", args.channel),
            ("device_id", args.device_id), ("step_seconds", args.step_seconds),
            ("final_seconds", args.final_seconds), ("max_duty_percent", args.max_duty),
            ("frames_spec", "2.1.0"), ("heartbeat_and_refresh_s", TX_PERIOD_S),
            ("status_periods", "existing configuration; not modified"),
            ("encoder_units", "RPM and rotations ONLY if conversion factors are 1"),
            ("test_note", args.note), ("t0_unix_s", state.get("wall0", "")),
            ("monotonic_duration_s", state.get("duration", ""))]
    for name in ("status_0", "status_2"):
        times = state[name]
        rows.append((name + "_count", len(times)))
        if len(times) < 2:
            continue
        intervals = [b - a for a, b in zip(times, times[1:])]
        rows.append((name + "_nonpositive_intervals", sum(d <= 0 for d in intervals)))
        if any(d <= 0 for d in intervals):
            rows.append((name + "_timing_valid", False))
            continue  # Posible salto del reloj Unix: no calcular una tasa ficticia.
        mean = statistics.mean(intervals)
        for key, value in [("dt_mean_ms", mean * 1000),
                           ("dt_median_ms", statistics.median(intervals) * 1000),
                           ("dt_min_ms", min(intervals) * 1000),
                           ("dt_max_ms", max(intervals) * 1000),
                           ("rate_hz", 1 / mean)]:
            rows.append((name + "_" + key, value))
        print(f"{name}: {len(times)} tramas, media {mean * 1000:.3f} ms, "
              f"{1 / mean:.2f} Hz; máximo {max(intervals) * 1000:.3f} ms")
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["parameter", "value"])
        writer.writerows(rows)


def run(args):
    folder = Path(args.output) / datetime.now().strftime("ensayo_%Y%m%d_%H%M%S_%f")
    folder.mkdir(parents=True, exist_ok=False)
    print(f"CSV: {folder.resolve()}")
    schedule, duration = build_schedule(args.step_seconds, args.final_seconds, args.max_duty)
    state = {"status_0": [], "status_2": []}
    reason, exit_code = "completed", 0
    bus = None
    start = None
    try:
        with ExitStack() as files:
            common = ["t_rx_s", "t_host_s", "rx_unix_s", "host_minus_rx_ms", "dt_rx_ms"]
            headers = {
                "status_0": common + ["applied_duty", "bus_voltage_V", "current_A",
                    "temperature_C", "hard_forward", "hard_reverse", "soft_forward",
                    "soft_reverse", "inverted", "primary_heartbeat_lock", "raw_hex"],
                "status_2": common + ["velocity_rpm", "position_rot", "raw_hex"],
                "comandos": ["t_plan_s", "t_tx_begin_s", "t_tx_end_s", "tx_unix_s",
                             "step_index", "duty_command", "event", "lateness_ms"],
            }
            writers, handles = {}, []
            for name, header in headers.items():
                file = files.enter_context((folder / (name + ".csv")).open(
                    "w", newline="", encoding="utf-8"))
                handles.append(file)
                writers[name] = csv.writer(file)
                writers[name].writerow(header)
            ids = [STATUS_0_BASE | args.device_id, STATUS_2_BASE | args.device_id,
                   ENABLE_REPLY_BASE | args.device_id]
            bus = can.Bus(interface="socketcan", channel=args.channel,
                          receive_own_messages=False,
                          can_filters=[{"can_id": i, "can_mask": 0x1FFFFFFF,
                                        "extended": True} for i in ids])
            try:
                enable_statuses(bus, args.device_id)
                start = time.monotonic()
                wall0 = time.time()
                state["wall0"] = wall0
                # time.time y monotonic se leen consecutivamente; pequeña incertidumbre.
                next_tx, next_flush = start, start + 1.0
                index, duty = -1, 0.0
                last_seen = {"status_0": None, "status_2": None}
                while True:
                    now = time.monotonic()
                    elapsed = now - start
                    if elapsed >= duration:
                        break
                    # Sin telemetría fresca se termina la prueba, conservando lo capturado.
                    for name, seen in last_seen.items():
                        if now - (start if seen is None else seen) > TELEMETRY_TIMEOUT_S:
                            raise RuntimeError(f"Sin {name} durante > {TELEMETRY_TIMEOUT_S}s")
                    change = index + 1 < len(schedule) and elapsed >= schedule[index + 1][0]
                    if change:
                        index += 1
                        planned, duty = schedule[index]
                        if elapsed - planned > MAX_STEP_LATENESS_S:
                            raise RuntimeError("Escalón retrasado >100 ms; ensayo interrumpido")
                        if duty > 0 and any(v is None for v in last_seen.values()):
                            raise RuntimeError("No se recibió ambos STATUS antes del primer escalón")
                    if change or now >= next_tx:
                        tx_begin = time.monotonic() - start
                        tx_wall = time.time()
                        send_duty(bus, args.device_id, duty)
                        tx_end = time.monotonic() - start
                        heartbeat(bus)
                        writers["comandos"].writerow([
                            planned if change else "", tx_begin, tx_end, tx_wall,
                            index, duty, "step" if change else "refresh",
                            (tx_begin - planned) * 1000 if change else ""])
                        next_tx = time.monotonic() + TX_PERIOD_S
                        if change:
                            print(f"t={tx_begin:7.3f} s | duty={100 * duty:5.1f}%")
                    if now >= next_flush:
                        for file in handles:
                            file.flush()
                        next_flush = now + 1.0
                    # Espera hasta la próxima trama o tarea. No hay sleep de muestreo.
                    next_step = start + schedule[index + 1][0] if index + 1 < len(schedule) else start + duration
                    deadline = min(next_tx, next_step, start + duration)
                    msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
                    if msg is None:
                        continue
                    host_mono, host_wall = time.monotonic(), time.time()
                    if msg.is_error_frame:
                        raise RuntimeError("Error de bus CAN")
                    if not msg.is_extended_id or msg.is_remote_frame or not msg.is_rx:
                        continue
                    if msg.arbitration_id == (STATUS_0_BASE | args.device_id):
                        name, decoded = "status_0", decode_status_0(msg.data)
                    elif msg.arbitration_id == (STATUS_2_BASE | args.device_id):
                        name, decoded = "status_2", decode_status_2(msg.data)
                    else:
                        continue
                    # No usar los STATUS encolados durante el handshake inicial.
                    if msg.timestamp < wall0:
                        continue
                    previous = state[name][-1] if state[name] else None
                    dt_ms = (msg.timestamp - previous) * 1000 if previous is not None else ""
                    writers[name].writerow([msg.timestamp - wall0, host_mono - start,
                        msg.timestamp, (host_wall - msg.timestamp) * 1000, dt_ms,
                        *decoded, bytes(msg.data).hex()])
                    state[name].append(msg.timestamp)
                    last_seen[name] = host_mono
                    if host_wall - msg.timestamp > TELEMETRY_TIMEOUT_S:
                        raise RuntimeError("Lectura CAN atrasada >500 ms; revisar carga de la Pi")
            finally:
                # Parar ANTES de cerrar archivos o calcular estadísticas.
                if start is not None:
                    state["duration"] = time.monotonic() - start
                stop_motor(bus, args.device_id)
                bus.shutdown()
    except KeyboardInterrupt:
        reason, exit_code = "interrupted_by_user", 130
        print("Prueba interrumpida; se conservan los CSV parciales.")
    except Exception as exc:
        reason, exit_code = f"error: {exc}", 1
        print(reason)
    write_summary(folder / "resumen.csv", args, state, reason)
    print(f"Resultado: {reason}\nArchivos: {folder.resolve()}")
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--step-seconds", type=float, default=6.0)
    parser.add_argument("--final-seconds", type=float, default=6.0)
    parser.add_argument("--max-duty", type=int, choices=range(5, 101, 5), default=100,
                        help="Máximo en porcentaje; predeterminado 100")
    parser.add_argument("--output", default="resultados_neo")
    parser.add_argument("--note", default="",
                        help="Descripción del montaje y ajustes: carga, reducción, filtros, etc.")
    args = parser.parse_args()
    if not 0 <= args.device_id <= 63:
        parser.error("device-id debe estar entre 0 y 63")
    if not math.isfinite(args.step_seconds) or args.step_seconds < 1:
        parser.error("step-seconds debe ser >= 1 s")
    if not math.isfinite(args.final_seconds) or args.final_seconds < 1:
        parser.error("final-seconds debe ser >= 1 s")
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
