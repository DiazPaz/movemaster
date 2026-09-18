import can
import struct
import threading
import time
import argparse


# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================

CAN_CHANNEL = "can0"
DEVICE_ID = 1

# Heartbeat universal
HEARTBEAT_ID = 0x01011840
HEARTBEAT_DATA = bytes([0xFF] * 8)

HEARTBEAT_PERIOD = 0.020  # 20 ms


# ============================================================
# IDS DE CONTROL
# ============================================================

# Position Control
POSITION_BASE_ID = 0x02050100

# MAXMotion Position Control
MAXMOTION_POSITION_BASE_ID = 0x02050200

POSITION_ID = POSITION_BASE_ID | DEVICE_ID
MAXMOTION_POSITION_ID = MAXMOTION_POSITION_BASE_ID | DEVICE_ID


# ============================================================
# IDS DE TELEMETRÍA
# ============================================================

# Status 0:
# Applied Output, Voltage, Current, Temperature, Limits...
STATUS_0_BASE_ID = 0x0205B800

# Status 2:
# Primary Encoder Velocity + Position
STATUS_2_BASE_ID = 0x0205B880

#
STATUS_0_ID = STATUS_0_BASE_ID | DEVICE_ID
STATUS_2_ID = STATUS_2_BASE_ID | DEVICE_ID



# ============================================================
# SET STATUSES ENABLED
# ============================================================

# API Class = 1
# API Index = 0
SET_STATUSES_ENABLED_BASE_ID = 0x02050400

# API Class = 1
# API Index = 1
SET_STATUSES_ENABLED_RESPONSE_BASE_ID = 0x02050440


SET_STATUSES_ENABLED_ID = (
    SET_STATUSES_ENABLED_BASE_ID | DEVICE_ID
)

SET_STATUSES_ENABLED_RESPONSE_ID = (
    SET_STATUSES_ENABLED_RESPONSE_BASE_ID | DEVICE_ID
)


# ============================================================
# HEARTBEAT
# ============================================================

class HeartbeatThread(threading.Thread):

    def __init__(self, bus, period=HEARTBEAT_PERIOD):
        super().__init__(daemon=True)

        self.bus = bus
        self.period = period
        self.running = True

    def run(self):

        next_time = time.monotonic()

        while self.running:

            msg = can.Message(
                arbitration_id=HEARTBEAT_ID,
                data=HEARTBEAT_DATA,
                is_extended_id=True
            )

            try:
                self.bus.send(msg)

            except can.CanError as e:
                print(f"[ERROR] Heartbeat CAN: {e}")

            next_time += self.period

            delay = next_time - time.monotonic()

            if delay > 0:
                time.sleep(delay)

            else:
                next_time = time.monotonic()

    def stop(self):
        self.running = False


# ============================================================
# HABILITAR STATUS FRAMES
# ============================================================

def enable_status_frames(bus, status_numbers, timeout=1.0):

    """
    Habilita los Status Frames indicados y espera la respuesta
    del SPARK MAX.

    Ejemplo:
        enable_status_frames(bus, [2, 7])

    Status 2:
        velocidad + posición

    Status 7:
        setpoint + AtSetpoint + PID slot
    """

    mask = 0

    for status in status_numbers:

        if not 0 <= status <= 15:
            raise ValueError(
                f"Status inválido: {status}"
            )

        mask |= (1 << status)

    # Queremos que todos los bits seleccionados en MASK
    # queden habilitados.
    enabled_bitfield = mask

    # uint16 mask
    # uint16 enabled bitfield
    payload = struct.pack(
        "<HH",
        mask,
        enabled_bitfield
    )

    msg = can.Message(
        arbitration_id=SET_STATUSES_ENABLED_ID,
        data=payload,
        is_extended_id=True
    )

    print("\n========== ENABLE STATUS FRAMES ==========")
    print(f"Statuses solicitados: {status_numbers}")
    print(
        f"CAN ID TX:            "
        f"0x{SET_STATUSES_ENABLED_ID:08X}"
    )
    print(f"Mask:                 0x{mask:04X}")
    print(
        f"Enabled solicitado:   "
        f"0x{enabled_bitfield:04X}"
    )
    print(
        f"Payload:              "
        f"{payload.hex(' ').upper()}"
    )

    bus.send(msg)

    # --------------------------------------------------------
    # Esperar respuesta del SPARK MAX
    # --------------------------------------------------------

    start = time.monotonic()

    while time.monotonic() - start < timeout:

        remaining = timeout - (time.monotonic() - start)

        msg = bus.recv(
            timeout=max(0.0, min(0.05, remaining))
        )

        if msg is None:
            continue

        if not msg.is_extended_id:
            continue

        if (
            msg.arbitration_id
            != SET_STATUSES_ENABLED_RESPONSE_ID
        ):
            continue

        if len(msg.data) != 5:

            print(
                "[ERROR] Respuesta Set Statuses con "
                f"longitud inesperada: {len(msg.data)}"
            )

            print(
                "Payload RX:",
                msg.data.hex(" ").upper()
            )

            return False

        # ----------------------------------------------------
        # Payload oficial:
        #
        # byte 0     : Result Code
        # bytes 1-2  : Specified Mask
        # bytes 3-4  : Enabled Bitfield completo
        # ----------------------------------------------------

        result_code = msg.data[0]

        specified_mask = int.from_bytes(
            msg.data[1:3],
            byteorder="little",
            signed=False
        )

        enabled_response = int.from_bytes(
            msg.data[3:5],
            byteorder="little",
            signed=False
        )

        print("\n---------- RESPONSE ----------")
        print(
            f"CAN ID RX:       "
            f"0x{msg.arbitration_id:08X}"
        )
        print(
            f"Payload RX:      "
            f"{msg.data.hex(' ').upper()}"
        )
        print(f"Result Code:     {result_code}")
        print(
            f"Specified Mask:  "
            f"0x{specified_mask:04X}"
        )
        print(
            f"Enabled Bitfield:"
            f" 0x{enabled_response:04X}"
        )

        if result_code == 0:
            print("Resultado:        ÉXITO")
        else:
            print("Resultado:        ERROR")

        # Mostrar individualmente los statuses
        print("\nStatus actualmente habilitados:")

        for status in range(16):

            if enabled_response & (1 << status):
                print(f"  Status {status}: ENABLED")

        print("==========================================\n")

        # Verificar específicamente los solicitados
        requested_are_enabled = (
            enabled_response & mask
        ) == mask

        if result_code != 0:
            return False

        if not requested_are_enabled:

            print(
                "[ERROR] El controlador respondió éxito, "
                "pero no aparecen todos los Status "
                "solicitados como habilitados."
            )

            return False

        return True

    print(
        "[ERROR] Timeout esperando respuesta a "
        "Set Statuses Enabled."
    )

    print(
        "Se esperaba respuesta en:"
        f" 0x{SET_STATUSES_ENABLED_RESPONSE_ID:08X}"
    )

    return False


# ============================================================
# PAYLOAD DE SETPOINT
# ============================================================

def build_setpoint_payload(
    setpoint,
    pid_slot=0,
    arbitrary_ff=0
):

    """
    Construye el payload para Position / MAXMotion Position.

    Bytes:
        0-3 : float32 setpoint
        4-5 : int16 arbitrary feedforward
        6-7 : configuración/reservado

    PID slot:
        bits 48-49

    Arbitrary FF units:
        bit 50
        0 = Voltage
    """

    if not 0 <= pid_slot <= 3:
        raise ValueError(
            "PID slot debe estar entre 0 y 3"
        )

    # Bits 48-49 = PID slot
    # Bit 50 = 0 => Voltage
    config = pid_slot & 0x03

    return struct.pack(
        "<fhH",
        float(setpoint),
        int(arbitrary_ff),
        config
    )


# ============================================================
# ENVIAR SETPOINT
# ============================================================

def send_setpoint(
    bus,
    mode,
    setpoint,
    pid_slot=0
):

    if mode == "position":

        arbitration_id = POSITION_ID

    elif mode == "maxmotion":

        arbitration_id = MAXMOTION_POSITION_ID

    else:

        raise ValueError(
            f"Modo inválido: {mode}"
        )

    payload = build_setpoint_payload(
        setpoint,
        pid_slot
    )

    msg = can.Message(
        arbitration_id=arbitration_id,
        data=payload,
        is_extended_id=True
    )

    bus.send(msg)

    print("\n========== TX SETPOINT ==========")
    print(f"Modo:       {mode}")
    print(
        f"CAN ID:     "
        f"0x{arbitration_id:08X}"
    )
    print(
        f"Setpoint:   "
        f"{setpoint:.4f} rot"
    )
    print(f"PID Slot:   {pid_slot}")
    print(
        f"Payload:    "
        f"{payload.hex(' ').upper()}"
    )
    print("=================================\n")


# ============================================================
# DECODER STATUS 0
# ============================================================

def decode_status_0(data):

    if len(data) != 8:
        return None

    raw = int.from_bytes(
        data,
        byteorder="little",
        signed=False
    )

    # --------------------------------------------------------
    # Applied Output
    # bits 0-15, signed
    # --------------------------------------------------------

    applied_raw = raw & 0xFFFF

    if applied_raw & 0x8000:
        applied_raw -= 0x10000

    applied_output = (
        applied_raw
        * 0.00003082369457075716
    )

    # --------------------------------------------------------
    # Bus Voltage
    # bits 16-27
    # --------------------------------------------------------

    voltage_raw = (raw >> 16) & 0xFFF

    voltage = (
        voltage_raw
        * 0.0073260073260073
    )

    # --------------------------------------------------------
    # Motor Current
    # bits 28-39
    # --------------------------------------------------------

    current_raw = (raw >> 28) & 0xFFF

    current = (
        current_raw
        * 0.0366300366300366
    )

    # --------------------------------------------------------
    # Temperature
    # bits 40-47
    # --------------------------------------------------------

    temperature = (
        (raw >> 40) & 0xFF
    )

    # --------------------------------------------------------
    # Limits
    # --------------------------------------------------------

    hard_fwd = bool(
        (raw >> 48) & 1
    )

    hard_rev = bool(
        (raw >> 49) & 1
    )

    soft_fwd = bool(
        (raw >> 50) & 1
    )

    soft_rev = bool(
        (raw >> 51) & 1
    )

    return {
        "applied_output": applied_output,
        "voltage": voltage,
        "current": current,
        "temperature": temperature,
        "hard_fwd": hard_fwd,
        "hard_rev": hard_rev,
        "soft_fwd": soft_fwd,
        "soft_rev": soft_rev
    }


# ============================================================
# DECODER STATUS 2
# ============================================================

def decode_status_2(data):

    """
    Status 2:

    bytes 0-3:
        Primary Encoder Velocity
        float32 little-endian
        RPM por defecto

    bytes 4-7:
        Primary Encoder Position
        float32 little-endian
        rotaciones por defecto
    """

    if len(data) != 8:
        return None

    velocity, position = struct.unpack(
        "<ff",
        data
    )

    return {
        "velocity": velocity,
        "position": position
    }





# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Prueba Position / MAXMotion Position "
            "para REV SPARK MAX"
        )
    )

    parser.add_argument(
        "--mode",
        choices=[
            "position",
            "maxmotion"
        ],
        required=True,
        help=(
            "Modo de control: "
            "position o maxmotion"
        )
    )

    parser.add_argument(
        "--setpoint",
        type=float,
        required=True,
        help=(
            "Setpoint de posición "
            "en rotaciones"
        )
    )

    parser.add_argument(
        "--time",
        type=float,
        default=20.0,
        help=(
            "Duración de la prueba "
            "en segundos"
        )
    )

    parser.add_argument(
        "--slot",
        type=int,
        default=0,
        help="PID slot (0-3)"
    )

    args = parser.parse_args()

    # ========================================================
    # ABRIR CAN
    # ========================================================

    bus = can.Bus(
        interface="socketcan",
        channel=CAN_CHANNEL
    )

    heartbeat = HeartbeatThread(bus)

    try:

        # ====================================================
        # 1. HEARTBEAT
        # ====================================================

        print("Iniciando heartbeat...")

        heartbeat.start()

        # Dar tiempo al SPARK para quedar habilitado
        time.sleep(0.25)

        # ====================================================
        # 2. HABILITAR STATUS 2
        # ====================================================

        status_ok = enable_status_frames(
            bus,
            [2],
            timeout=1.0
        )

        if not status_ok:

            print(
                "\nNo se pudo confirmar la "
                "habilitación de Status 2."
            )

            print(
                "No se enviará el setpoint."
            )

            return

        # Esperar algunos frames periódicos
        time.sleep(0.10)

        # ====================================================
        # 3. ENVIAR SETPOINT
        # ====================================================

        send_setpoint(
            bus,
            args.mode,
            args.setpoint,
            args.slot
        )

        # ====================================================
        # 4. VARIABLES DE TELEMETRÍA
        # ====================================================

        status0 = None
        status2 = None
        

        status0_count = 0
        status2_count = 0
        

        start = time.monotonic()

        next_print = start

        # ====================================================
        # 5. HEADER
        # ====================================================

        print(
            " Tiempo | Posición | Velocidad | "
            "Output | Corriente | "
            " Límites"
        )

        print("-" * 120)

        # ====================================================
        # 6. LOOP DE LECTURA
        # ====================================================

        while (
            time.monotonic() - start
            < args.time
        ):

            msg = bus.recv(
                timeout=0.01
            )

            if (
                msg is not None
                and msg.is_extended_id
            ):

                # --------------------------------------------
                # STATUS 0
                # --------------------------------------------

                if (
                    msg.arbitration_id
                    == STATUS_0_ID
                ):

                    decoded = decode_status_0(
                        msg.data
                    )

                    if decoded is not None:
                        status0 = decoded
                        status0_count += 1

                # --------------------------------------------
                # STATUS 2
                # --------------------------------------------

                elif (
                    msg.arbitration_id
                    == STATUS_2_ID
                ):

                    decoded = decode_status_2(
                        msg.data
                    )

                    if decoded is not None:
                        status2 = decoded
                        status2_count += 1


            # =================================================
            # IMPRIMIR CADA 250 ms
            # =================================================

            now = time.monotonic()

            if now >= next_print:

                elapsed = now - start

                # --------------------------------------------
                # Status 2
                # --------------------------------------------

                if status2 is not None:

                    position = (
                        f"{status2['position']:8.4f}"
                    )

                    velocity = (
                        f"{status2['velocity']:9.3f}"
                    )

                else:

                    position = "   ---  "
                    velocity = "    ---  "

                # --------------------------------------------
                # Status 0
                # --------------------------------------------

                if status0 is not None:

                    output = (
                        f"{status0['applied_output']:7.3f}"
                    )

                    current = (
                        f"{status0['current']:8.2f}"
                    )

                    limits = (
                        f"HF={int(status0['hard_fwd'])} "
                        f"HR={int(status0['hard_rev'])} "
                        f"SF={int(status0['soft_fwd'])} "
                        f"SR={int(status0['soft_rev'])}"
                    )

                else:

                    output = "  ---  "
                    current = "   ---  "
                    limits = "---"

                # --------------------------------------------
                # Status 8
                # --------------------------------------------

                
                # --------------------------------------------
                # PRINT
                # --------------------------------------------

                print(
                    f"{elapsed:6.2f} | "
                    f"{position} | "
                    f"{velocity} | "
                    f"{output} | "
                    f"{current} | "
                    f"{limits}"
                )

                next_print += 0.25

        # ====================================================
        # 7. RESUMEN
        # ====================================================

        print("\n========== RESUMEN ==========")

        print(
            f"Status 0 recibidos: {status0_count}"
        )

        print(
            f"Status 2 recibidos: {status2_count}"
        )


        if status2 is not None:

            print(
                "\nÚltima posición:"
                f" {status2['position']:.6f} rot"
            )

            print(
                "Última velocidad:"
                f" {status2['velocity']:.6f} RPM"
            )

        
        print("=============================\n")

    except KeyboardInterrupt:

        print(
            "\nPrueba detenida por el usuario."
        )

    except can.CanError as e:

        print(
            f"\n[ERROR CAN] {e}"
        )

    finally:

        # ====================================================
        # DETENER HEARTBEAT Y CERRAR BUS
        # ====================================================

        heartbeat.stop()

        heartbeat.join(
            timeout=1.0
        )

        bus.shutdown()

        print("Heartbeat detenido.")
        print(
            "Comunicación CAN cerrada."
        )


if __name__ == "__main__":
    main()