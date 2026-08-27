import can
import struct
import threading
import time


class SparkMax:
    HEARTBEAT_ID = 0x01011840
    DUTY_BASE_ID = 0x02050080

    def __init__(self, can_id=1, channel="can0", period=0.02):
        if not 0 <= can_id <= 63:
            raise ValueError("CAN ID must be between 0 and 63")

        self.can_id = can_id
        self.channel = channel
        self.period = period

        self.bus = can.Bus(
            interface="socketcan",
            channel=channel
        )

        self.duty_id = self.DUTY_BASE_ID | can_id

        self._duty = 0.0
        self._running = False
        self._lock = threading.Lock()
        self._thread = None

    def _send_heartbeat(self):
        msg = can.Message(
            arbitration_id=self.HEARTBEAT_ID,
            data=b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",
            is_extended_id=True
        )

        self.bus.send(msg)

    def _send_duty_cycle(self, duty):
        payload = struct.pack("<f", duty) + b"\x00\x00\x00\x00"

        msg = can.Message(
            arbitration_id=self.duty_id,
            data=payload,
            is_extended_id=True
        )

        self.bus.send(msg)

    def _control_loop(self):
        next_time = time.monotonic()

        while self._running:
            with self._lock:
                duty = self._duty

            try:
                self._send_heartbeat()
                self._send_duty_cycle(duty)

            except can.CanError as e:
                print(f"CAN transmission error: {e}")

            next_time += self.period

            delay = next_time - time.monotonic()

            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.monotonic()

    def start(self):
        if self._running:
            return

        self._running = True

        self._thread = threading.Thread(
            target=self._control_loop,
            daemon=True
        )

        self._thread.start()

    def set(self, duty):
        duty = float(duty)

        if duty > 1.0:
            duty = 1.0
        elif duty < -1.0:
            duty = -1.0

        with self._lock:
            self._duty = duty

    def stop(self):
        self.set(0.0)

    def close(self):
        # Command zero before stopping communication.
        self.stop()
        time.sleep(0.1)

        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)

        self.bus.shutdown()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
