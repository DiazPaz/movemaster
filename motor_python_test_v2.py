import can
import math
import struct
import threading
import time

from sparkmax import SparkMax


# ============================================================
# CONFIGURATION
# ============================================================

CAN_CHANNEL = "can0"
SPARK_MAX_ID = 1

# 108 motor revolutions = 1 gearbox output revolution
GEAR_RATIO = 108.0

# REV Hardware Client:
# Primary Encoder Position Conversion Factor = 360
#
# Therefore Status 2 position is motor-shaft DEGREES.
POSITION_CONVERSION_FACTOR = 360.0

# Allowed OUTPUT-SHAFT setpoints, relative to manually captured zero.
MIN_ANGLE_DEG = -360.0
MAX_ANGLE_DEG = +360.0

# Fixed open-loop motor output used while moving.
# Start low. Increase only after testing the real mechanism.
OPEN_LOOP_DUTY = 0.1

# Stop when the output shaft is this close to the target.
# This is only used to decide when to turn the motor off.
STOP_TOLERANCE_DEG = 0.25

# If commanding a positive angle makes the measured angle go negative,
# change this from +1.0 to -1.0.
MOTOR_SIGN = +1.0

# Status 2 telemetry safety timeout.
TELEMETRY_TIMEOUT_S = 0.25

# Maximum time allowed for one move.
MOVE_TIMEOUT_S = 60.0


# ============================================================
# STATUS 2
# ============================================================

STATUS_2_ID = 0x0205B880 | SPARK_MAX_ID


def decode_status_2(data):
    """
    spark_mmrt firmware-25 layout:

      bytes 0..3 = primary encoder velocity, float32
      bytes 4..7 = primary encoder position, float32

    With Position Conversion Factor = 360, position is motor-shaft degrees.
    """
    if len(data) != 8:
        raise ValueError("Status 2 must contain exactly 8 bytes")

    velocity = struct.unpack("<f", bytes(data[0:4]))[0]
    position = struct.unpack("<f", bytes(data[4:8]))[0]

    if not math.isfinite(velocity) or not math.isfinite(position):
        raise ValueError("Invalid Status 2 value")

    return velocity, position


# ============================================================
# SHARED TELEMETRY
# ============================================================

state_lock = threading.Lock()
shutdown_event = threading.Event()

motor_position_deg = None
motor_velocity = None
last_status2_time = None

# This is NOT written into the SPARK MAX.
# It is simply the encoder reading captured when you press ENTER.
zero_motor_deg = None


# ============================================================
# HARDWARE
# ============================================================

motor = SparkMax(
    can_id=SPARK_MAX_ID,
    channel=CAN_CHANNEL
)

telemetry_bus = can.Bus(
    interface="socketcan",
    channel=CAN_CHANNEL
)


# ============================================================
# TELEMETRY THREAD
# ============================================================

def telemetry_loop():
    global motor_position_deg
    global motor_velocity
    global last_status2_time

    while not shutdown_event.is_set():
        msg = telemetry_bus.recv(timeout=0.1)

        if msg is None:
            continue

        if not msg.is_extended_id:
            continue

        if msg.arbitration_id != STATUS_2_ID:
            continue

        try:
            velocity, position = decode_status_2(msg.data)
        except ValueError:
            continue

        with state_lock:
            motor_velocity = velocity
            motor_position_deg = position
            last_status2_time = time.monotonic()


# ============================================================
# POSITION / ANGLE HELPERS
# ============================================================

def wait_for_status2(timeout=3.0):
    start = time.monotonic()

    while time.monotonic() - start < timeout:
        with state_lock:
            if motor_position_deg is not None:
                return True

        time.sleep(0.02)

    return False


def capture_zero():
    """
    Define the mechanism's CURRENT physical location as output angle 0 deg.
    The accumulated SPARK encoder itself is NOT reset.
    """
    global zero_motor_deg

    with state_lock:
        if motor_position_deg is None:
            raise RuntimeError("No Status 2 encoder position available")

        zero_motor_deg = motor_position_deg
        captured = zero_motor_deg

    print()
    print("ZERO CAPTURED")
    print(f"Raw motor encoder: {captured:.3f} motor-deg")
    print("Current gearbox output position is now defined as 0.000 deg.")
    print()


def get_joint_angle_deg():
    """
    Continuous/unwrapped output-shaft angle relative to manual zero.

    No modulo 360 is used.
    """
    with state_lock:
        position = motor_position_deg
        zero = zero_motor_deg

    if position is None or zero is None:
        return None

    return (position - zero) / GEAR_RATIO


def joint_target_to_motor_deg(joint_angle_deg):
    """
    Convert an output-shaft target to the corresponding absolute
    accumulated motor encoder target.
    """
    with state_lock:
        zero = zero_motor_deg

    if zero is None:
        raise RuntimeError("Zero has not been captured")

    return zero + (joint_angle_deg * GEAR_RATIO)


# ============================================================
# OPEN-LOOP MOVE
# ============================================================

def move_to_angle(target_joint_deg):
    """
    Fixed-duty open-loop actuation.

    The encoder is ONLY watched to know when to turn the motor OFF.
    There is no proportional/PID correction and no active holding
    after the move finishes.

    Because the motor is commanded at a fixed duty, some overshoot
    is expected.
    """

    target_joint_deg = float(target_joint_deg)

    if not MIN_ANGLE_DEG <= target_joint_deg <= MAX_ANGLE_DEG:
        raise ValueError(
            f"Target must be between {MIN_ANGLE_DEG:.0f} and "
            f"{MAX_ANGLE_DEG:.0f} degrees"
        )

    current_joint = get_joint_angle_deg()

    if current_joint is None:
        raise RuntimeError("Encoder position is not available")

    target_motor = joint_target_to_motor_deg(target_joint_deg)

    print()
    print(f"Current output angle: {current_joint:+.2f} deg")
    print(f"Target output angle:  {target_joint_deg:+.2f} deg")
    print(f"Motor encoder target: {target_motor:+.2f} motor-deg")

    initial_error = target_joint_deg - current_joint

    if abs(initial_error) <= STOP_TOLERANCE_DEG:
        motor.set(0.0)
        print("Already within stop tolerance.")
        return

    # Direction is chosen once at the beginning of the move.
    direction = +1.0 if initial_error > 0 else -1.0
    duty = MOTOR_SIGN * direction * OPEN_LOOP_DUTY

    print(f"Commanding fixed duty: {duty:+.3f}")
    print("Press Ctrl+C at any time to stop.")

    motor.set(duty)

    start = time.monotonic()
    last_print = 0.0

    try:
        while True:
            now = time.monotonic()

            with state_lock:
                position = motor_position_deg
                last_rx = last_status2_time
                zero = zero_motor_deg

            # Fail safe if encoder telemetry disappears.
            if (
                position is None
                or last_rx is None
                or (now - last_rx) > TELEMETRY_TIMEOUT_S
            ):
                motor.set(0.0)
                raise RuntimeError(
                    "Status 2 telemetry lost/stale. Motor stopped."
                )

            current_joint = (position - zero) / GEAR_RATIO
            error = target_joint_deg - current_joint

            if now - last_print >= 0.25:
                print(
                    f"Angle: {current_joint:+8.2f} deg | "
                    f"Target: {target_joint_deg:+8.2f} deg | "
                    f"Error: {error:+7.2f} deg"
                )
                last_print = now

            # Stop if we are close enough.
            reached_tolerance = abs(error) <= STOP_TOLERANCE_DEG

            # Also stop immediately if fixed-duty motion crossed the target.
            crossed_target = (
                (direction > 0 and current_joint >= target_joint_deg)
                or
                (direction < 0 and current_joint <= target_joint_deg)
            )

            if reached_tolerance or crossed_target:
                motor.set(0.0)

                final_angle = get_joint_angle_deg()

                print()
                print("Motor output set to 0%.")
                print(f"Final measured output angle: {final_angle:+.2f} deg")
                print(
                    "No active position hold is running. "
                    "Brake mode, if enabled in the SPARK MAX, is passive only."
                )
                print()
                return

            if now - start >= MOVE_TIMEOUT_S:
                motor.set(0.0)
                raise RuntimeError(
                    f"Move timed out after {MOVE_TIMEOUT_S:.1f} seconds. "
                    "Motor stopped."
                )

            time.sleep(0.01)

    except:
        motor.set(0.0)
        raise


# ============================================================
# MAIN
# ============================================================

telemetry_thread = threading.Thread(
    target=telemetry_loop,
    daemon=True
)

try:
    print("Starting SPARK MAX...")
    motor.start()

    # Absolutely no motor drive during manual zero positioning.
    motor.set(0.0)

    telemetry_thread.start()

    print("Waiting for Status 2 encoder telemetry...")

    if not wait_for_status2():
        raise RuntimeError(
            "No Status 2 telemetry received. "
            "Make sure Status 2 is enabled."
        )

    print()
    print("MANUAL ZERO PROCEDURE")
    print("---------------------")
    print("Motor command is currently 0%.")
    print(
        "Manually move the robotic-arm base to the physical position "
        "that you want to call 0 degrees."
    )
    input("When it is in the correct position, press ENTER to capture zero... ")

    capture_zero()

    print("OPEN-LOOP ANGLE CONTROL")
    print("-----------------------")
    print("Allowed absolute setpoints: -360 to +360 output degrees.")
    print("Encoder position is continuous; it is NOT wrapped at 360 degrees.")
    print("After a move completes, motor output becomes 0%.")
    print()
    print("Commands:")
    print("  5       move to +5 deg")
    print("  -30     move to -30 deg")
    print("  360     move to +360 deg")
    print("  -360    move to -360 deg")
    print("  z       redefine CURRENT physical location as 0 deg")
    print("  p       print current angle")
    print("  q       quit")
    print()

    while True:
        command = input("Target [-360..360], z, p, or q: ").strip().lower()

        if command in ("q", "quit", "exit"):
            break

        if command in ("z", "zero"):
            motor.set(0.0)
            capture_zero()
            continue

        if command in ("p", "position"):
            angle = get_joint_angle_deg()

            if angle is None:
                print("Angle unavailable.")
            else:
                print(f"Current output angle: {angle:+.3f} deg")

            continue

        try:
            target = float(command)
            move_to_angle(target)

        except ValueError as e:
            print(f"Invalid command: {e}")

        except RuntimeError as e:
            print(f"ERROR: {e}")


except KeyboardInterrupt:
    print("\nCtrl+C received.")

except Exception as e:
    print(f"\nERROR: {e}")

finally:
    print("\nStopping motor...")
    shutdown_event.set()

    motor.set(0.0)
    time.sleep(0.1)

    telemetry_bus.shutdown()
    motor.close()

    print("Done.")
