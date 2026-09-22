"""
Configuración y control MAXMotion Position.

Configura slot 0:
    kP_0 = 1.0
    kI_0 = 0.001

    MAXMotion Cruise Velocity_0 = 900 RPM
    MAXMotion Max Accel_0 = 500 RPM/s
    MAXMotion Allowed Closed Loop Error_0 = 0.5 rot

Después:
    1. Verifica PARAMETER_WRITE_RESPONSE
    2. Persiste parámetros
    3. Verifica PERSIST_PARAMETERS_RESPONSE
    4. Inicia heartbeat
    5. Envía MAXMotion Position Setpoint
"""

import argparse
import threading
import time

import can

from sparkmax_json_protocol import SparkMAXMotionProtocol


# ============================================================
# CONFIGURACIÓN
# ============================================================

JSON_PATH = "spark-frames-2.1.0"

DEVICE_ID = 1
PID_SLOT = 0

KP = 1.0
KI = 0.0

MAX_ACCEL = 500.0
CRUISE_VELOCITY = 900.0

ALLOWED_CLOSED_LOOP_ERROR = 0.1


# ============================================================
# HEARTBEAT
# ============================================================

HEARTBEAT_ID = 0x01011840
HEARTBEAT_DATA = bytes([0xFF] * 8)

HEARTBEAT_PERIOD = 0.020


class HeartbeatThread(threading.Thread):

    def __init__(self, bus):

        super().__init__(daemon=True)

        self.bus = bus
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

                print(
                    f"[ERROR] Heartbeat: {e}"
                )

            next_time += HEARTBEAT_PERIOD

            delay = (
                next_time
                - time.monotonic()
            )

            if delay > 0:

                time.sleep(delay)

            else:

                next_time = time.monotonic()

    def stop(self):

        self.running = False


# ============================================================
# IMPRIMIR PARAMETER WRITE
# ============================================================

def print_parameter_result(
    name,
    result
):

    print(
        f"\n--- {name} ---"
    )

    print(
        f"Parameter ID : "
        f"{result['parameter_id']}"
    )

    print(
        f"Requested    : "
        f"{result['requested_value']}"
    )

    print(
        f"SPARK value  : "
        f"{result['current_value']}"
    )

    print(
        f"Result code  : "
        f"{result['result_code']}"
    )

    print(
        f"Success      : "
        f"{result['success']}"
    )

    print(
        f"Value match  : "
        f"{result['value_matches']}"
    )


# ============================================================
# LIMPIAR RX
# ============================================================

def drain_rx(bus, duration=0.05):

    """
    Vacía mensajes viejos del buffer RX antes de una
    operación que espera una respuesta concreta.
    """

    deadline = (
        time.monotonic()
        + duration
    )

    while time.monotonic() < deadline:

        msg = bus.recv(
            timeout=0.005
        )

        if msg is None:
            break


# ============================================================
# PERSIST PARAMETERS
# ============================================================

def persist_parameters(
    spark,
    bus,
    timeout=2.5
):

    """
    Envía PERSIST_PARAMETERS usando exclusivamente
    la información del JSON.

    RESULT_CODE = 0:
        éxito confirmado.

    RESULT_CODE = 255:
        no está documentado como éxito.
        Seguimos esperando otra respuesta hasta timeout.
    """

    persist_frame = spark[
        "frames"
    ]["PERSIST_PARAMETERS"]

    response_frame = spark[
        "frames"
    ]["PERSIST_PARAMETERS_RESPONSE"]


    # --------------------------------------------------------
    # MAGIC NUMBER DESDE EL JSON
    # --------------------------------------------------------

    magic_spec = (
        persist_frame[
            "signals"
        ]["MAGIC_NUMBER"]
    )

    magic_number = int(
        magic_spec["decodedMin"]
    )


    # --------------------------------------------------------
    # CONSTRUIR FRAME
    # --------------------------------------------------------

    packet = persist_frame.packet(
        spark.device_id,
        {
            "MAGIC_NUMBER":
                magic_number
        }
    )


    response_id = (
        response_frame
        .arbitration_id(
            spark.device_id
        )
    )


    # --------------------------------------------------------
    # LIMPIAR RESPUESTAS VIEJAS
    # --------------------------------------------------------

    drain_rx(
        bus,
        duration=0.05
    )


    print(
        "\n========== PERSIST PARAMETERS =========="
    )

    print(
        f"Frame:       "
        f"{persist_frame['name']}"
    )

    print(
        f"CAN ID TX:   "
        f"0x{packet.arbitration_id:08X}"
    )

    print(
        f"Magic:       "
        f"{magic_number}"
    )

    print(
        f"Payload TX:  "
        f"{packet.data.hex(' ').upper()}"
    )


    # --------------------------------------------------------
    # TRANSMITIR
    # --------------------------------------------------------

    bus.send(
        packet.to_python_can()
    )


    # --------------------------------------------------------
    # ESPERAR RESPUESTA
    # --------------------------------------------------------

    deadline = (
        time.monotonic()
        + timeout
    )

    responses_seen = []

    while time.monotonic() < deadline:

        remaining = (
            deadline
            - time.monotonic()
        )

        msg = bus.recv(
            timeout=min(
                0.05,
                max(
                    0.0,
                    remaining
                )
            )
        )

        if msg is None:
            continue

        if not msg.is_extended_id:
            continue

        if (
            msg.arbitration_id
            != response_id
        ):
            continue


        decoded = (
            response_frame
            .decode_payload(
                bytes(msg.data)
            )
        )

        result_code = int(
            decoded["RESULT_CODE"]
        )

        responses_seen.append(
            result_code
        )


        print(
            f"\nCAN ID RX:   "
            f"0x{msg.arbitration_id:08X}"
        )

        print(
            f"Payload RX:  "
            f"{bytes(msg.data).hex(' ').upper()}"
        )

        print(
            f"Result Code: "
            f"{result_code}"
        )


        # ====================================================
        # ÉXITO DOCUMENTADO
        # ====================================================

        if result_code == 0:

            print(
                "Resultado:    ÉXITO"
            )

            print(
                "Persistencia confirmada "
                "por el SPARK MAX."
            )

            print(
                "========================================\n"
            )

            return True


        # ====================================================
        # 0xFF: NO DOCUMENTADO COMO ÉXITO
        # ====================================================

        if result_code == 0xFF:

            print(
                "[WARNING] RESULT_CODE = 255 (0xFF)."
            )

            print(
                "No está documentado como éxito; "
                "se seguirá esperando una respuesta 0."
            )

            continue


        # ====================================================
        # OTRO ERROR
        # ====================================================

        print(
            "Resultado:    ERROR"
        )

        print(
            "========================================\n"
        )

        return False


    # --------------------------------------------------------
    # TIMEOUT
    # --------------------------------------------------------

    print(
        "\n[ERROR] No se recibió "
        "RESULT_CODE = 0."
    )

    print(
        f"Respuestas observadas: "
        f"{responses_seen}"
    )

    print(
        "La persistencia NO puede "
        "considerarse confirmada."
    )

    print(
        "========================================\n"
    )

    return False


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--setpoint",
        type=float,
        required=True,
        help=(
            "Setpoint MAXMotion Position "
            "en rotaciones"
        )
    )

    args = parser.parse_args()


    # ========================================================
    # CARGAR PROTOCOLO
    # ========================================================

    spark = SparkMAXMotionProtocol(
        JSON_PATH,
        device_id=DEVICE_ID
    )


    # ========================================================
    # ABRIR CAN
    # ========================================================

    bus = can.Bus(
        interface="socketcan",
        channel="can0"
    )

    heartbeat = None


    try:

        # ====================================================
        # 1. CONFIGURACIÓN
        #
        # TODAVÍA NO INICIAMOS EL HEARTBEAT.
        # El motor permanece deshabilitado.
        # ====================================================

        print(
            "\n========================================"
        )

        print(
            " CONFIGURANDO SLOT 0"
        )

        print(
            "========================================"
        )


        results = spark.configure_slot(

            bus,

            slot=PID_SLOT,

            pidf={
                "p": KP,
                "i": KI,
            },

            maxmotion={

                "max_acceleration":
                    MAX_ACCEL,

                "cruise_velocity":
                    CRUISE_VELOCITY,

                "allowed_profile_error":
                    ALLOWED_CLOSED_LOOP_ERROR,
            },

            timeout=1.0
        )


        # ====================================================
        # 2. MOSTRAR RESPUESTAS
        # ====================================================

        for name, result in results.items():

            print_parameter_result(
                name,
                result
            )


        # ====================================================
        # 3. VALIDAR WRITES
        # ====================================================

        all_ok = all(

            result["success"]
            and
            result["value_matches"]

            for result
            in results.values()
        )


        if not all_ok:

            print(
                "\n[ERROR] Uno o más "
                "parámetros fallaron."
            )

            return


        print(
            "\nTodos los parámetros fueron "
            "aceptados correctamente."
        )


        # ====================================================
        # 4. PEQUEÑA PAUSA ANTES DE PERSISTIR
        # ====================================================

        time.sleep(0.1)


        # ====================================================
        # 5. PERSIST PARAMETERS
        #
        # MOTOR TODAVÍA DESHABILITADO.
        # ====================================================

        persist_ok = persist_parameters(
            spark,
            bus,
            timeout=2.5
        )


        if not persist_ok:

            print(
                "[ERROR] El SPARK no confirmó "
                "la persistencia."
            )

            print(
                "No se enviará el setpoint."
            )

            return


        # ====================================================
        # 6. ESPERAR A QUE EL SPARK TERMINE LA OPERACIÓN
        # ====================================================

        time.sleep(0.25)


        # ====================================================
        # 7. INICIAR HEARTBEAT
        # ====================================================

        print(
            "\nIniciando heartbeat..."
        )

        heartbeat = HeartbeatThread(
            bus
        )

        heartbeat.start()

        time.sleep(0.25)


        # ====================================================
        # 8. MAXMOTION POSITION SETPOINT
        # ====================================================

        print(
            "\n========== MAXMOTION POSITION =========="
        )


        setpoint_packet = (
            spark.send_setpoint(

                bus,

                args.setpoint,

                slot=PID_SLOT,

                arbitrary_feedforward=0.0,

                arbitrary_feedforward_units=0
            )
        )


        print(
            f"Frame:       "
            f"{setpoint_packet.frame_name}"
        )

        print(
            f"CAN ID:      "
            f"0x{setpoint_packet.arbitration_id:08X}"
        )

        print(
            f"Setpoint:    "
            f"{args.setpoint} rot"
        )

        print(
            f"PID Slot:    "
            f"{PID_SLOT}"
        )

        print(
            f"Payload:     "
            f"{setpoint_packet.data.hex(' ').upper()}"
        )

        print(
            "MAXMotion Position Control activado."
        )

        print(
            "========================================\n"
        )


        # ====================================================
        # 9. MANTENER CONTROL ACTIVO
        # ====================================================

        print(
            "Control activo."
        )

        print(
            "Ctrl+C para terminar."
        )


        while True:

            time.sleep(1.0)


    except KeyboardInterrupt:

        print(
            "\nPrograma detenido."
        )


    finally:

        if heartbeat is not None:

            heartbeat.stop()

            heartbeat.join(
                timeout=1.0
            )

        bus.shutdown()

        print(
            "Bus CAN cerrado."
        )


if __name__ == "__main__":

    main()