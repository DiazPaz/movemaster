#include "movemaster_hardware/sparkmax_json_protocol.hpp"
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 2) { std::cerr << "Usage: protocol_demo /path/to/spark-frames-2.1.0\n"; return 2; }
  try {
    movemaster::SparkMAXMotionProtocol spark(argv[1], 1);
    std::cout << "No CAN socket is opened. These are encoded examples only.\n"
        << "Frames: " << spark.frames.size() << ", version: " << spark.frames.frames_version() << '\n'
        << spark.maxmotion_setpoint_packet(0.5, 0).repr() << '\n'
        << spark.parameter_write_packet(spark["pidf"][0]["p"], 0.05).repr() << '\n'
        << spark.parameter_read_packet(spark["pidf"][0]["p"]).repr() << '\n';
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
