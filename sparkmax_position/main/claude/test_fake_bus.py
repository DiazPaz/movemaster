import struct
import sys
import time
import threading
from collections import deque
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, "/mnt/user-data/uploads")
sys.path.insert(0, "/home/claude")

import can  # noqa: E402


class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=True):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


class FakeBus:
    """Simula un bus CAN: guarda lo enviado y permite inyectar frames de 'respuesta'."""

    def __init__(self, channel=None, bustype=None, **kwargs):
        self.sent = []
        self.rx_queue = deque()
        self.lock = threading.Lock()

    def send(self, msg):
        with self.lock:
            self.sent.append(msg)

    def recv(self, timeout=0.0):
        with self.lock:
            if self.rx_queue:
                return self.rx_queue.popleft()
        return None

    def inject(self, msg):
        with self.lock:
            self.rx_queue.append(msg)

    def shutdown(self):
        pass


fake_bus_singleton = {}


def fake_bus_factory(channel=None, bustype=None, **kwargs):
    bus = FakeBus(channel, bustype, **kwargs)
    fake_bus_singleton["bus"] = bus
    return bus


with mock.patch.object(can, "Bus", side_effect=fake_bus_factory):
    from teach_pendant_backend import SparkMaxTeachPendant

    pendant = SparkMaxTeachPendant(
        "/mnt/user-data/uploads/spark-frames-2_1.0",
        device_id=1,
        can_channel="vcan0",
        can_interface="socketcan",
        heartbeat_hz=50.0,
    )

bus = fake_bus_singleton["bus"]
pendant.start()
time.sleep(0.3)

# --- 0) Heartbeat por defecto: ID 0x01011840, payload 0xFF x8, ~50 Hz ---
hb_frames = [m for m in bus.sent if m.arbitration_id == pendant.HEARTBEAT_ARBITRATION_ID]
print(f"Frames de heartbeat en ~0.3s: {len(hb_frames)}")
assert len(hb_frames) >= 8, "el heartbeat por defecto no se está enviando periódicamente"
assert bytes(hb_frames[-1].data) == pendant.HEARTBEAT_DATA
print("OK: heartbeat por defecto (0x01011840, 0xFF x8) funciona.\n")

# --- 1) Simular un frame STATUS_2 entrante (posición=2.5 rot, velocidad=1.2) ---
status2_frame = pendant.protocol.frames["STATUS_2"]
payload = status2_frame.encode_payload({
    "PRIMARY_ENCODER_VELOCITY": 1.2,
    "PRIMARY_ENCODER_POSITION": 2.5,
})
bus.inject(FakeMessage(pendant._status2_id, payload))

# --- 2) Simular un frame STATUS_0 (CURRENT) ---
status0_frame = pendant.protocol.frames["STATUS_0"]
payload0 = status0_frame.encode_payload({"CURRENT": 3.0})
bus.inject(FakeMessage(pendant._status0_id, payload0))

time.sleep(0.15)  # dar tiempo al hilo para procesar

snap = pendant.snapshot()
print("Snapshot tras inyectar STATUS_0/STATUS_2:", snap)
assert abs(snap["position"] - 2.5) < 1e-3, "posicion no coincide"
assert abs(snap["velocity"] - 1.2) < 1e-3, "velocidad no coincide"
assert abs(snap["current"] - 3.0) < 0.05, "corriente no coincide (con cuantizacion esperada)"
print("OK: telemetría STATUS_0/STATUS_2 decodificada correctamente.\n")

# --- 3) move_to(): debe empezar a reenviar MAXMOTION_POSITION_SETPOINT cada ciclo ---
pendant.move_to(4.0)
time.sleep(0.25)  # a 50Hz deberian salir ~10+ frames
setpoint_frame = pendant.protocol.frames["MAXMOTION_POSITION_SETPOINT"]
setpoint_id = setpoint_frame.arbitration_id(1)
sent_setpoints = [m for m in bus.sent if m.arbitration_id == setpoint_id]
print(f"Frames de setpoint enviados en ~0.25s: {len(sent_setpoints)}")
assert len(sent_setpoints) >= 5, "no se está reenviando el setpoint periódicamente"
last = setpoint_frame.decode_payload(sent_setpoints[-1].data)
print("Último setpoint decodificado:", last)
assert abs(last["SETPOINT"] - 4.0) < 1e-3
print("OK: reenvío periódico del Setpoint funciona.\n")

err = pendant.get_tracking_error()
print(f"Tracking error (SP - PV) = {err} (esperado ~1.5)")
assert abs(err - 1.5) < 1e-3
print("OK: error de seguimiento correcto.\n")

# --- 4) set_pidf(): debe enviar PARAMETER_WRITE y, cuando el hilo ve la
#        PARAMETER_WRITE_RESPONSE simulada, desbloquear la llamada. ---
param_write_frame = pendant.protocol.frames["PARAMETER_WRITE"]
param_resp_frame = pendant.protocol.frames["PARAMETER_WRITE_RESPONSE"]


persist_frame = pendant.protocol.frames["PERSIST_PARAMETERS"]
persist_resp_frame = pendant.protocol.frames["PERSIST_PARAMETERS_RESPONSE"]


def fake_spark_responder():
    """Hilo que simula al SPARK MAX respondiendo PARAMETER_WRITE y PERSIST_PARAMETERS."""
    seen_writes = set()
    seen_persists = set()
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        with bus.lock:
            new_writes = [
                m for m in bus.sent
                if m.arbitration_id == param_write_frame.arbitration_id(1) and id(m) not in seen_writes
            ]
            new_persists = [
                m for m in bus.sent
                if m.arbitration_id == persist_frame.arbitration_id(1) and id(m) not in seen_persists
            ]
        for m in new_writes:
            seen_writes.add(id(m))
            decoded = param_write_frame.decode_payload(m.data)
            param_id = int(decoded["PARAMETER_ID"])
            raw_value = int(decoded["VALUE"])
            resp_payload = param_resp_frame.encode_payload({
                "PARAMETER_ID": param_id,
                "PARAMETER_TYPE": 3,  # float
                "VALUE": raw_value,
                "RESULT_CODE": 0,
            })
            bus.inject(FakeMessage(param_resp_frame.arbitration_id(1), resp_payload))
        for m in new_persists:
            seen_persists.add(id(m))
            # Simula el caso real observado: primero un 0xFF "no documentado",
            # luego el 0 definitivo.
            bus.inject(FakeMessage(
                persist_resp_frame.arbitration_id(1),
                persist_resp_frame.encode_payload({"RESULT_CODE": 0xFF}),
            ))
            time.sleep(0.01)
            bus.inject(FakeMessage(
                persist_resp_frame.arbitration_id(1),
                persist_resp_frame.encode_payload({"RESULT_CODE": 0}),
            ))
        time.sleep(0.01)


responder = threading.Thread(target=fake_spark_responder, daemon=True)
responder.start()

result = pendant.set_pidf(p=0.05, i=0.001, timeout=1.0)
print("Resultado set_pidf:", result)
assert result["p"]["success"] is True
assert abs(result["p"]["current_value"] - 0.05) < 1e-4
assert result["i"]["success"] is True
assert abs(result["i"]["current_value"] - 0.001) < 1e-4
print("OK: escritura de PIDF (PARAMETER_WRITE / PARAMETER_WRITE_RESPONSE) funciona end-to-end.\n")

result2 = pendant.set_motion_profile(max_acceleration=8, cruise_velocity=5, allowed_profile_error=0.1)
print("Resultado set_motion_profile:", result2)
assert result2["max_acceleration"]["success"] is True
assert result2["max_acceleration"]["value_matches"] is True
assert result2["cruise_velocity"]["success"] is True
assert result2["allowed_profile_error"]["success"] is True
assert abs(result2["allowed_profile_error"]["current_value"] - 0.1) < 1e-4
print("OK: escritura de perfil MAXMotion (incl. allowed_profile_error) funciona end-to-end.\n")

# --- 5) persist_parameters(): debe manejar 0xFF ("no documentado") y luego
#        devolver True al ver RESULT_CODE == 0. ---
persisted = pendant.persist_parameters(timeout=2.0)
print("Resultado persist_parameters():", persisted)
assert persisted is True
print("OK: persist_parameters() maneja 0xFF y confirma con RESULT_CODE=0.\n")

# --- 6) configure_and_persist(): escribe, valida y persiste en un solo paso ---
result3 = pendant.configure_and_persist(
    pidf={"p": 0.02},
    maxmotion={"cruise_velocity": 900, "max_acceleration": 500},
)
print("Resultado configure_and_persist():", result3)
assert result3["all_ok"] is True
assert result3["persisted"] is True
print("OK: configure_and_persist() (configurar -> verificar -> persistir) funciona.\n")

pendant.release()
time.sleep(0.05)
assert pendant.get_setpoint() is None
print("OK: release() detiene el comando de posición.\n")

pendant.stop()
print("Todas las pruebas pasaron correctamente.")
