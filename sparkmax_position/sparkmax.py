import can
import struct
import threading
import time

DEVICE_TYPE_MOTOR_CONTROLLER = 2   # confirmado (tabla FRC CAN Device Types)
MANUFACTURER_REV = 5               # confirmado (tabla FRC CAN Manufacturer Codes)
API_INDEX_SET_SETPOINT = 2         # confirmado empíricamente (usado para duty cycle)

# --------------------------------------------------------------
# API Class por modo de control.
#
# IMPORTANTE: el modo de control NO se selecciona con un byte dentro
# del payload -- se selecciona con este número de Clase dentro del
# propio ID de la trama. "duty_cycle" = 0 quedó confirmado en la
# práctica (era justo el bug: todo se mandaba a la Clase 0 sin
# importar qué control_type se pusiera en el payload).
#
# Los demás valores son HIPÓTESIS tomadas del enum ControlType de
# REVLib, pero esa numeración ha cambiado entre versiones de REVLib
# (ver explicación en el chat). VERIFICA con candump mientras corres
# "Run Motor" en modo Position/MAXMotion Position desde el Hardware
# Client antes de confiar en "max_motion_position" para mover algo.
# --------------------------------------------------------------
API_CLASS_BY_CONTROL_TYPE = {
    "duty_cycle": 0,             # confirmado
    "velocity": 1,               # sin verificar
    "voltage": 2,                # sin verificar
    "position": 3,                # sin verificar
    "max_motion_position": 4,     # HIPÓTESIS -- verificar con candump (alternativa: 5)
    "current": 5,                 # sin verificar (podría chocar con max_motion_position en firmware nuevo)
}


def build_arbitration_id(device_type: int, manufacturer: int,
                          api_class: int, api_index: int,
                          device_number: int) -> int:
    """
    Arma un ID extendido de 29 bits con el esquema de direccionamiento
    de FRC: [Device Type:5][Manufacturer:8][API Class:6][API Index:4][Device Number:6]
    """
    api_id = ((api_class & 0x3F) << 4) | (api_index & 0x0F)
    return (
        ((device_type & 0x1F) << 24)
        | ((manufacturer & 0xFF) << 16)
        | ((api_id & 0x3FF) << 6)
        | (device_number & 0x3F)
    )


class SparkMax:
    HEARTBEAT_ID = 0x01011840

    # ID Base para Telemetría Status 2 (Device Type=2, Mfg=5, API Class=46, API Index=2)
    STATUS_2_BASE_ID = 0x0205B880

    def __init__(self, can_id=1, channel="can0", period=0.02,
                 control_type="max_motion_position"):
        if not 0 <= can_id <= 63:
            raise ValueError("CAN ID must be between 0 and 63")
        if control_type not in API_CLASS_BY_CONTROL_TYPE:
            raise ValueError(f"control_type desconocido: {control_type!r}")

        self.can_id = can_id
        self.channel = channel
        self.period = period
        self.control_type = control_type

        self.bus = can.Bus(
            interface="socketcan",
            channel=channel
        )

        self.status_2_id = self.STATUS_2_BASE_ID | can_id

        # Variables de estado
        self._target_setpoint = 0.0
        self._current_position = 0.0
        self._current_velocity = 0.0

        self._running = False
        self._lock = threading.Lock()

        # Hilos
        self._control_thread = None
        self._read_thread = None

    def _reference_id(self, control_type: str) -> int:
        api_class = API_CLASS_BY_CONTROL_TYPE[control_type]
        return build_arbitration_id(
            DEVICE_TYPE_MOTOR_CONTROLLER, MANUFACTURER_REV,
            api_class, API_INDEX_SET_SETPOINT, self.can_id,
        )

    def _send_heartbeat(self):
        msg = can.Message(
            arbitration_id=self.HEARTBEAT_ID,
            data=b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",
            is_extended_id=True
        )
        self.bus.send(msg)

    def _send_reference(self, setpoint, control_type=None, pid_slot=0):
        """
        setpoint: valor en las unidades del modo (duty cycle: -1..1;
                  position/max_motion_position: rotaciones).
        control_type: si no se especifica, usa self.control_type.
        """
        control_type = control_type or self.control_type
        # bytes: float32 setpoint + pidSlot + relleno
        payload = struct.pack("<f B B h", setpoint, pid_slot, 0, 0)

        msg = can.Message(
            arbitration_id=self._reference_id(control_type),
            data=payload,
            is_extended_id=True
        )
        self.bus.send(msg)

    # --- HILO DE ESCRITURA (Control y Heartbeat) ---
    def _control_loop(self):
        next_time = time.monotonic()

        while self._running:
            with self._lock:
                target = self._target_setpoint

            try:
                self._send_heartbeat()
                self._send_reference(target)  # usa self.control_type
            except can.CanError as e:
                print(f"Error de transmisión CAN: {e}")

            next_time += self.period
            delay = next_time - time.monotonic()

            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.monotonic()

    # --- HILO DE LECTURA (Telemetría) ---
    def _read_loop(self):
        while self._running:
            try:
                # El timeout evita que el hilo se quede bloqueado eternamente al cerrar el programa
                msg = self.bus.recv(timeout=0.1)

                if msg is not None and msg.is_extended_id:
                    if msg.arbitration_id == self.status_2_id:
                        # bytes 0-3 = velocidad, bytes 4-7 = posición
                        # (orden confirmado en spark_monitor.py)
                        vel, pos = struct.unpack("<f f", msg.data)

                        with self._lock:
                            self._current_position = pos
                            self._current_velocity = vel

            except can.CanError:
                pass  # Ignorar errores de lectura momentáneos para no saturar la consola

    def start(self):
        if self._running:
            return

        self._running = True

        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def set_position(self, target):
        """Asigna la posición objetivo (en rotaciones), usando self.control_type."""
        with self._lock:
            self._target_setpoint = float(target)

    def get_position(self):
        with self._lock:
            return self._current_position

    def get_velocity(self):
        with self._lock:
            return self._current_velocity

    def stop(self):
        self.set_position(0.0)

    def close(self):
        self._running = False

        if self._control_thread is not None:
            self._control_thread.join(timeout=1.0)
        if self._read_thread is not None:
            self._read_thread.join(timeout=1.0)

        self.bus.shutdown()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()