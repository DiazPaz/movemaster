"""Pruebas de integración en python-can virtual. No acceden a can0 ni a motores.

El peer usa bytes/IDs de referencia independientes del codec del backend para
detectar errores de orden, escala, direccionamiento y slot. No simula la física.
"""
import math
from pathlib import Path
import struct
import threading
import time
import unittest
import uuid

import can
from teach_pendant_backend import TeachPendantBackend

SPEC = Path(__file__).with_name("spark-frames-2.1.0")


def eventually(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.002)
    raise AssertionError("La condición no se cumplió antes del timeout")


class Peer:
    def __init__(self, channel, device=1):
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.device = device
        self.stop = threading.Event()
        self.error = None
        self.records = []
        self.lock = threading.Lock()
        self.status2 = True
        self.status0 = True
        self.locked = False
        self.ack_delay = 0.0
        self.no_ack = False
        self.rejected_id = None
        self.wrong_value = False
        self.wrong_type = False
        self.extra_unrelated_ack = False
        self.persist_responses = (0,)
        self.pending = []
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def send(self, base, payload, *, device=None):
        self.bus.send(can.Message(arbitration_id=base | (self.device if device is None else device),
                                  is_extended_id=True, data=payload), timeout=0.01)

    def logged(self, base=None):
        with self.lock:
            return [r for r in self.records if base is None or r[1] & ~63 == base]

    def run(self):
        due = time.monotonic()
        try:
            while not self.stop.is_set():
                rx = self.bus.recv(0.002)
                now = time.monotonic()
                if rx is not None:
                    with self.lock:
                        self.records.append((now, rx.arbitration_id, bytes(rx.data)))
                    base = rx.arbitration_id & ~63
                    if base == 0x02050400:  # SET_STATUSES_ENABLED
                        self.pending.append((now + self.ack_delay, 0x02050440,
                                             struct.pack("<BHH", 0, 5, 7)))
                    elif base == 0x02057C80:  # STOP_FOLLOWER_MODE
                        self.pending.append((now + self.ack_delay, 0x02057CC0, b""))
                    elif base == 0x0205FFC0:  # PERSIST_PARAMETERS
                        for index, code in enumerate(self.persist_responses):
                            self.pending.append((now + self.ack_delay + 0.03 * index,
                                                 0x02050500, bytes([code])))
                    elif base == 0x02053800:
                        pid = rx.data[0]
                        raw = struct.unpack("<I", rx.data[1:])[0]
                        kind = 2 if pid in (9, 158, 160) else (4 if pid == 149 else 3)
                        result = 4 if pid == self.rejected_id else 0
                        ack = struct.pack("<BBIB", pid, 2 if self.wrong_type else kind,
                                          raw ^ 1 if self.wrong_value else raw, result)
                        if self.extra_unrelated_ack:
                            self.send(0x02053840, struct.pack("<BBIB", 250, kind, raw, 0))
                            self.send(0x02053840, ack, device=(self.device + 1) % 64)
                        self.pending.append((now + self.ack_delay, 0x02053840, ack))
                ready = [x for x in self.pending if x[0] <= now]
                self.pending = [x for x in self.pending if x[0] > now]
                for _, base, payload in ready:
                    if not self.no_ack:
                        self.send(base, payload)
                if now >= due:
                    # 273 * 150 / 4095 = 10 A, campo CURRENT bits 28..39.
                    if self.status0:
                        raw = (273 << 28) | (int(self.locked) << 53)
                        self.send(0x0205B800, raw.to_bytes(8, "little"))
                    if self.status2:
                        self.send(0x0205B880, struct.pack("<ff", 30.0, 1.25))
                    due = now + 0.010
        except Exception as exc:
            self.error = exc

    def close(self):
        self.stop.set()
        self.thread.join(1)
        self.bus.shutdown()
        if self.error:
            raise self.error


class BackendTests(unittest.TestCase):
    def setUp(self):
        channel = str(uuid.uuid4())
        self.peer = Peer(channel)
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.axis = TeachPendantBackend(SPEC, bus=self.bus, slot=2,
                                       feedback_timeout_s=0.15, response_timeout_s=0.25,
                                       position_limits=(-2, 2)).start()

    def tearDown(self):
        self.axis.close()
        self.peer.close()
        self.bus.shutdown()

    def initialized(self):
        self.axis.initialize().result(2)
        eventually(lambda: self.axis.telemetry().position_fresh and
                   self.axis.telemetry().current_fresh)

    def configured(self):
        self.initialized()
        self.axis.set_pidf(p=0.01, i=0, d=0, f=0).result(2)
        self.axis.set_motion_profile(maxacceleration=60, cruisevelocity=15,
                                    allowed_profile_error=0.1).result(2)

    def test_enable_status_and_units_ack(self):
        self.initialized()
        writes = {payload[0]: payload[1:] for _, _, payload in self.peer.logged(0x02053800)}
        self.assertEqual(writes[9], struct.pack("<I", 1))
        self.assertEqual(writes[112], struct.pack("<f", 1))
        self.assertEqual(writes[113], struct.pack("<f", 1))
        self.assertEqual(writes[149], bytes(4))
        self.assertEqual(writes[158], struct.pack("<I", 20))
        self.assertEqual(writes[160], struct.pack("<I", 20))
        self.assertEqual(self.peer.logged(0x02050400)[0][2], b"\x05\x00\x05\x00")
        self.assertFalse(self.axis.telemetry().armed)
        self.assertFalse(self.peer.logged(0x02050200))
        self.assertFalse(self.peer.logged(0x01011840))

    def test_golden_telemetry_and_tracking_error(self):
        self.configured()
        t = self.axis.telemetry()
        self.assertEqual(t.pv_rot, 1.25)
        self.assertEqual(t.velocity_rpm, 30.0)
        self.assertAlmostEqual(t.current_a, 10.0, places=6)
        self.assertIsNone(t.sp_rot)
        self.assertIsNone(t.error_rot)
        self.assertEqual(self.axis.arm(), 1.25)
        self.axis.send_setpoint(-0.5)
        self.assertEqual(self.axis.telemetry().error_rot, -1.75)

    def test_only_maxmotion_slot_ids_and_periodic_tx(self):
        self.configured()
        pid_values = {data[0]: struct.unpack("<I", data[1:])[0]
                      for _, _, data in self.peer.logged(0x02053800)}
        self.assertTrue({29, 30, 31, 32, 176, 177, 179} <= set(pid_values))
        self.assertEqual(pid_values[176], struct.unpack("<I", struct.pack("<f", 15))[0])
        self.axis.arm()
        self.axis.send_setpoint(0.5)
        eventually(lambda: len(self.peer.logged(0x02050200)) >= 5)
        targets = self.peer.logged(0x02050200)
        self.assertEqual(targets[-1][2], struct.pack("<fhH", 0.5, 0, 2))
        gaps = [b[0] - a[0] for a, b in zip(targets, targets[1:])]
        self.assertLess(max(gaps), 0.15)  # tolerancia de prueba, no garantía RT
        forbidden = {0x02050080, 0x02050100, 0x02050180, 0x020501C0, 0x02050240}
        self.assertFalse(any(arb & ~63 in forbidden for _, arb, _ in self.peer.logged()))
        enable = self.peer.logged(0x01011840)
        self.assertEqual(enable[-1][1], 0x01011840)
        self.assertEqual(enable[-1][2], b"\xff" * 8)
        self.assertLessEqual(targets[0][0], enable[0][0])

    def test_slow_configuration_does_not_block_rx(self):
        self.initialized()
        self.peer.ack_delay = 0.1
        self.peer.extra_unrelated_ack = True
        start = time.monotonic()
        before = self.axis.telemetry().rx_frames
        ticket = self.axis.set_pidf(p=0.025)
        self.assertLess(time.monotonic() - start, 0.05)
        self.assertFalse(ticket.done())
        eventually(lambda: self.axis.telemetry().rx_frames > before + 6)
        self.assertFalse(ticket.done())  # otros IDs/otros parámetros no completan el ticket
        self.assertIn("p", ticket.result(1))

    def test_nack_fails_remaining_parameters_and_latches(self):
        self.initialized()
        self.peer.rejected_id = 29
        ticket = self.axis.set_pidf(p=0.1, i=0.2, d=0.3, f=0.4)
        with self.assertRaisesRegex(RuntimeError, "rechazado"):
            ticket.result(1)
        self.assertFalse(any(data[0] == 30 for _, _, data in self.peer.logged(0x02053800)))
        with self.assertRaises(RuntimeError):
            self.axis.arm()
        self.assertIsNotNone(self.axis.telemetry().fault)

    def test_wrong_echo_value_is_failure(self):
        self.initialized()
        self.peer.wrong_value = True
        with self.assertRaisesRegex(RuntimeError, "no coincide"):
            self.axis.set_pidf(p=0.1).result(1)

    def test_wrong_echo_type_is_failure(self):
        self.initialized()
        self.peer.wrong_type = True
        with self.assertRaisesRegex(RuntimeError, "no coincide"):
            self.axis.set_pidf(p=0.1).result(1)

    def test_parameter_timeout_fails_queued_commands(self):
        self.initialized()
        self.peer.no_ack = True
        first = self.axis.set_pidf(p=0.1)
        second = self.axis.set_cruisevelocity(10)
        with self.assertRaisesRegex(RuntimeError, "Timeout CAN"):
            first.result(1)
        with self.assertRaisesRegex(RuntimeError, "Timeout CAN"):
            second.result(1)
        self.assertTrue(self.axis.telemetry().position_fresh)
        self.assertEqual(self.axis.telemetry().configuration_pending, 0)

    def test_watchdog_stale_encoder_even_with_current_alive(self):
        self.configured()
        self.axis.arm()
        eventually(lambda: self.peer.logged(0x01011840))
        self.peer.status2 = False
        eventually(lambda: self.axis.telemetry().fault is not None)
        t = self.axis.telemetry()
        self.assertFalse(t.armed)
        self.assertFalse(t.position_fresh)
        self.assertTrue(t.current_fresh)
        self.assertIsNone(t.error_rot)
        count = len(self.peer.logged(0x01011840))
        time.sleep(0.06)
        self.assertEqual(len(self.peer.logged(0x01011840)), count)
        self.peer.status2 = True
        eventually(lambda: self.axis.telemetry().position_fresh)
        with self.assertRaises(RuntimeError):
            self.axis.arm()  # recuperar telemetría no rearranca el motor

    def test_reject_bad_telemetry_and_ignore_other_devices(self):
        self.initialized()
        self.peer.send(0x0205B880, struct.pack("<ff", 1, 999), device=2)
        self.peer.send(0x0205B880, b"bad")
        self.peer.send(0x0205B880, struct.pack("<ff", math.nan, 1))
        eventually(lambda: self.axis.telemetry().malformed_frames == 2)
        self.assertEqual(self.axis.telemetry().pv_rot, 1.25)
        self.assertIsNone(self.axis.telemetry().fault)

    def test_primary_lock_is_compatible_with_reference_heartbeat(self):
        self.configured()
        self.peer.locked = True
        eventually(lambda: self.axis.telemetry().primary_heartbeat_lock)
        self.axis.arm()
        eventually(lambda: self.peer.logged(0x01011840))
        self.assertIsNone(self.axis.telemetry().fault)

    def test_validation_is_atomic_and_configuration_requires_disarm(self):
        with self.assertRaises(RuntimeError):
            self.axis.arm()
        self.initialized()
        before = len(self.peer.logged(0x02053800))
        with self.assertRaises(ValueError):
            self.axis.set_pidf(p=0.1, i=math.nan)
        self.assertEqual(len(self.peer.logged(0x02053800)), before)
        for invalid in (0, -1, math.nan, math.inf, 1e100):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.axis.set_maxacceleration(invalid)
        self.axis.set_pidf(p=0.01, i=0, d=0, f=0).result(1)
        self.axis.set_motion_profile(maxacceleration=60, cruisevelocity=15).result(1)
        self.axis.arm()
        with self.assertRaises(ValueError):
            self.axis.send_setpoint(2.1)
        with self.assertRaises(ValueError):
            self.axis.send_setpoint(math.inf)
        with self.assertRaises(RuntimeError):
            self.axis.set_pidf(p=0.1)
        self.axis.disarm()
        with self.assertRaises(RuntimeError):
            self.axis.send_setpoint(0)

    def test_close_completes_pending_command_and_stops_worker(self):
        self.initialized()
        self.peer.no_ack = True
        ticket = self.axis.set_pidf(p=0.1)
        self.axis.close()
        self.assertTrue(ticket.done())
        with self.assertRaisesRegex(RuntimeError, "cerrado"):
            ticket.result()
        self.assertFalse(self.peer.logged(0x01011840))
        self.assertFalse(self.axis._thread.is_alive())

    def test_transport_failure_propagates_and_thread_exits(self):
        self.configured()
        original = self.bus.send
        def failed(*_args, **_kwargs):
            raise can.CanError("bus-off simulado")
        self.bus.send = failed
        try:
            self.axis.arm()  # El heartbeat se transmite sólo habilitado.
            eventually(lambda: self.axis.telemetry().fault is not None)
            self.assertIn("bus-off", self.axis.telemetry().fault)
            eventually(lambda: not self.axis._thread.is_alive())
            with self.assertRaises(RuntimeError):
                self.axis.arm()
        finally:
            self.bus.send = original

    def test_persist_uses_magic_and_waits_after_confirmed_success(self):
        self.configured()
        start = time.monotonic()
        ticket = self.axis.persist_parameters()
        result = ticket.result(1)
        self.assertGreaterEqual(time.monotonic() - start, 0.25)
        self.assertEqual(result["persist_parameters"]["RESULT_CODE"], 0)
        self.assertEqual(self.peer.logged(0x0205FFC0)[-1][2], struct.pack("<H", 15011))
        self.assertFalse(self.peer.logged(0x01011840))

    def test_persist_255_waits_for_zero(self):
        self.configured()
        self.peer.persist_responses = (255, 0)
        result = self.axis.persist_parameters().result(1)
        self.assertEqual(result["persist_parameters"]["RESULT_CODE"], 0)
        self.assertIsNone(self.axis.telemetry().fault)

    def test_persist_255_alone_times_out_and_prevents_motion(self):
        self.configured()
        self.peer.persist_responses = (255,)
        with self.assertRaisesRegex(RuntimeError, "Timeout CAN"):
            self.axis.persist_parameters().result(3.5)
        with self.assertRaises(RuntimeError):
            self.axis.arm()
        self.assertFalse(self.peer.logged(0x01011840))

    def test_persist_error_prevents_motion(self):
        self.configured()
        self.peer.persist_responses = (1,)
        with self.assertRaisesRegex(RuntimeError, "Persistencia rechazada"):
            self.axis.persist_parameters().result(1)
        with self.assertRaises(RuntimeError):
            self.axis.arm()

    def test_disarm_stops_heartbeat_and_waits_before_setup(self):
        self.configured()
        self.axis.arm()
        eventually(lambda: len(self.peer.logged(0x01011840)) >= 2)
        self.axis.disarm()
        request = self.axis.set_pidf(p=0.025)
        request.result(1.5)
        last_hb = self.peer.logged(0x01011840)[-1][0]
        last_write = self.peer.logged(0x02053800)[-1][0]
        self.assertGreaterEqual(last_write - last_hb, 0.49)
        self.assertFalse(self.axis.telemetry().armed)


class HeartbeatEncodingTests(unittest.TestCase):
    def test_exact_heartbeat_from_working_raspberry_pi_example(self):
        for device in (0, 1, 31, 53, 63):
            with self.subTest(device=device):
                axis = TeachPendantBackend(SPEC, device_id=device)
                self.assertEqual(axis._hb_on.data, b"\xff" * 8)
                self.assertEqual(axis._hb_on.arbitration_id, 0x01011840)


if __name__ == "__main__":
    unittest.main(verbosity=2)
