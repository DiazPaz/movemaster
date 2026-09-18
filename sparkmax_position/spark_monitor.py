import can
import struct
import math
import time


# ============================================================
# Configuration
# ============================================================

CAN_CHANNEL = "can0"
SPARK_MAX_ID = 1

STATUS_0_ID = 0x0205B800 | SPARK_MAX_ID
STATUS_1_ID = 0x0205B840 | SPARK_MAX_ID
STATUS_2_ID = 0x0205B880 | SPARK_MAX_ID


# ============================================================
# CAN setup
# ============================================================

bus = can.Bus(
    interface="socketcan",
    channel=CAN_CHANNEL
)

print("SPARK MAX monitor")
print(f"CAN ID: {SPARK_MAX_ID}")
print()
print(f"Status 0: 0x{STATUS_0_ID:08X}")
print(f"Status 1: 0x{STATUS_1_ID:08X}")
print(f"Status 2: 0x{STATUS_2_ID:08X}")
print()
print("Press Ctrl+C to exit.\n")


# ============================================================
# Status 0
# Output, bus voltage, motor current, temperature
# ============================================================

def decode_status_0(data):
    if len(data) != 8:
        raise ValueError("Status 0 must contain 8 bytes")

    raw = int.from_bytes(data, byteorder="little")

    # Applied output: bits 0-15
    output_raw = raw & 0xFFFF

    if output_raw & 0x8000:
        output_raw -= 0x10000

    output = output_raw * 0.00003082369457075716

    # Bus voltage: bits 16-27
    voltage_raw = (raw >> 16) & 0xFFF
    voltage = voltage_raw * 0.0073260073260073

    # Motor current: bits 28-39
    current_raw = (raw >> 28) & 0xFFF
    current = current_raw * 0.0366300366300366

    # Temperature: bits 40-47
    temperature = (raw >> 40) & 0xFF

    return {
        "output": output,
        "output_raw": output_raw,
        "voltage": voltage,
        "voltage_raw": voltage_raw,
        "current": current,
        "current_raw": current_raw,
        "temperature": temperature,
    }


# ============================================================
# Status 2
#
# According to the spark_mmrt source:
#
# bytes 0-3 = primary encoder velocity (float32)
# bytes 4-7 = primary encoder position (float32)
#
# Velocity units: RPM
# Position units: rotations
# ============================================================

def decode_status_2(data):
    if len(data) != 8:
        raise ValueError("Status 2 must contain 8 bytes")

    velocity_rpm = struct.unpack(
        "<f",
        bytes(data[0:4])
    )[0]

    position_rotations = struct.unpack(
        "<f",
        bytes(data[4:8])
    )[0]

    if not math.isfinite(velocity_rpm):
        raise ValueError("Invalid encoder velocity")

    if not math.isfinite(position_rotations):
        raise ValueError("Invalid encoder position")

    return {
        "velocity_rpm": velocity_rpm,
        "position_rotations": position_rotations,
    }


# ============================================================
# Latest telemetry
# ============================================================

output = None
voltage = None
current = None
current_raw = None
temperature = None

velocity_rpm = None
position_rotations = None

peak_current = 0.0
peak_current_raw = 0

status_0_seen = False
status_1_seen = False
status_2_seen = False

last_print = 0.0


# ============================================================
# Helpers
# ============================================================

def format_value(value, spec):
    if value is None:
        return "N/A"

    return format(value, spec)


# ============================================================
# Main loop
# ============================================================

try:
    while True:

        msg = bus.recv(timeout=1.0)

        if msg is None:
            continue

        if not msg.is_extended_id:
            continue


        # ----------------------------------------------------
        # Status 0
        # ----------------------------------------------------

        if msg.arbitration_id == STATUS_0_ID:

            try:
                status = decode_status_0(msg.data)

            except ValueError as e:
                print(f"Status 0 decode error: {e}")
                continue

            output = status["output"]
            voltage = status["voltage"]

            current = status["current"]
            current_raw = status["current_raw"]

            temperature = status["temperature"]

            status_0_seen = True

            if current > peak_current:
                peak_current = current
                peak_current_raw = current_raw


        # ----------------------------------------------------
        # Status 1
        #
        # We currently only record that it exists.
        # Status 1 contains faults/warnings, NOT RPM.
        # ----------------------------------------------------

        elif msg.arbitration_id == STATUS_1_ID:

            status_1_seen = True


        # ----------------------------------------------------
        # Status 2
        #
        # Encoder velocity + encoder position
        # ----------------------------------------------------

        elif msg.arbitration_id == STATUS_2_ID:

            try:
                status = decode_status_2(msg.data)

            except ValueError as e:
                print(f"Status 2 decode error: {e}")
                continue

            velocity_rpm = status["velocity_rpm"]
            position_rotations = status["position_rotations"]

            status_2_seen = True


        else:
            continue


        # ----------------------------------------------------
        # Print approximately 10 times per second
        # ----------------------------------------------------

        now = time.monotonic()

        if now - last_print < 0.1:
            continue

        last_print = now


        output_text = format_value(output, "+.4f")
        voltage_text = format_value(voltage, ".2f")
        current_text = format_value(current, ".3f")

        rpm_text = format_value(velocity_rpm, ".1f")
        position_text = format_value(position_rotations, ".4f")

        raw_current_text = (
            "N/A"
            if current_raw is None
            else str(current_raw)
        )

        temp_text = (
            "N/A"
            if temperature is None
            else str(temperature)
        )


        print(
            f"Output: {output_text} | "
            f"Voltage: {voltage_text} V | "
            f"Current: {current_text} A "
            f"(raw={raw_current_text}) | "
            f"RPM: {rpm_text} | "
            f"Position: {position_text} rot | "
            f"Temp: {temp_text} C"
        )


except KeyboardInterrupt:

    print("\nStopped.\n")

    print(
        f"Peak current observed: "
        f"{peak_current:.3f} A "
        f"(raw={peak_current_raw})"
    )

    print()

    print("Frames detected:")
    print(f"Status 0: {'YES' if status_0_seen else 'NO'}")
    print(f"Status 1: {'YES' if status_1_seen else 'NO'}")
    print(f"Status 2: {'YES' if status_2_seen else 'NO'}")


finally:
    bus.shutdown()