#include "movemaster_hardware/movemaster_driver.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace { volatile std::sig_atomic_t running = 1; void stop(int) { running = 0; } }
int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: driver_monitor SPEC.json JOINT_CONFIG.json can0\n"
        << "Writes setup to RAM and reads telemetry. Does not activate or send heartbeat.\n";
    return 2;
  }
  try {
    auto config = movemaster::load_driver_config(argv[2], argv[1], argv[3]);
    movemaster::MoveMasterDriver driver(config);
    driver.configure();
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::cout << "Setup acknowledged. Monitoring without enable heartbeat. Ctrl+C to exit.\n";
    while (running) {
      driver.read();
      for (std::size_t i = 0; i < driver.states().size(); ++i) {
        const auto &s = driver.states()[i];
        std::cout << config.joints[i].name << ": " << s.position << " rad, "
            << s.velocity << " rad/s, " << s.current << " A\n";
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
