"""Example usage of sparkmax_json_protocol.py."""

import can
from sparkmax_json_protocol import SparkMAXMotionProtocol

JSON_PATH = "spark-frames-2.1.0"
DEVICE_ID = 1

spark = SparkMAXMotionProtocol(JSON_PATH, device_id=DEVICE_ID)

# -----------------------------------------------------------------------------
# Dictionary-style introspection
# -----------------------------------------------------------------------------
kp0 = spark["pidf"][0]["p"]
ki0 = spark["pidf"][0]["i"]
kd0 = spark["pidf"][0]["d"]
kf0 = spark["pidf"][0]["f"]

cruise0 = spark["maxmotion"][0]["cruise_velocity"]
accel0 = spark["maxmotion"][0]["maxaccel"]  # alias works
error0 = spark["maxmotion"][0]["allowedprofileerror"]  # alias works

print("P0:", kp0)
print("Cruise0:", cruise0)

# The setpoint frame definition comes directly from the JSON.
setpoint_spec = spark["setpoint"]
print(f"MAXMotion SP base ID: 0x{setpoint_spec.base_arb_id:08X}")
print(f"MAXMotion SP device ID: 0x{setpoint_spec.arbitration_id(DEVICE_ID):08X}")

# -----------------------------------------------------------------------------
# Build packets WITHOUT sending (useful for tests)
# -----------------------------------------------------------------------------
print(spark.parameter_write_packet(kp0, 0.05))
print(spark.parameter_write_packet(accel0, 8.0))
print(spark.maxmotion_setpoint_packet(0.5, slot=0))

# -----------------------------------------------------------------------------
# Send on SocketCAN
# Heartbeat must already be running elsewhere.
# -----------------------------------------------------------------------------
# bus = can.Bus(interface="socketcan", channel="can0")
#
# result = spark.write_parameter(bus, kp0, 0.05)
# print(result)
#
# results = spark.configure_slot(
#     bus,
#     slot=0,
#     pidf={"p": 0.05, "i": 0.0, "d": 0.0, "f": 0.0},
#     maxmotion={
#         "cruise_velocity": 30.0,
#         "max_acceleration": 10.0,
#         "allowed_profile_error": 1.0,
#     },
# )
# print(results)
#
# spark.send_setpoint(bus, 0.5, slot=0)
# bus.shutdown()
