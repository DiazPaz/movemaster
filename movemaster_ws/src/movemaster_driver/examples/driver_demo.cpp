// movemaster_driver_demo: prueba de banco SIN ROS.
//
//   movemaster_driver_demo <spark-frames.json> [can] [eje] [objetivo_rad] [vel_rad_s]
//
// Sin objetivo sólo configura los 3 SPARK MAX e imprime la telemetría.
// Con objetivo, arma y lleva el eje indicado (0..2) a objetivo_rad con una rampa
// lineal en el host a vel_rad_s (0.2 rad/s por defecto). Position Control no
// tiene perfil propio en el SPARK: un escalón grande sería un golpe al reductor.
// Ctrl+C desarma (deja de enviar heartbeat).

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "movemaster_driver/movemaster_driver.hpp"

namespace
{
std::atomic<bool> g_stop{false};
void onSignal(int) {g_stop = true;}

void print(const movemaster_driver::MoveMasterDriver & driver)
{
  for (std::size_t i = 0; i < driver.jointCount(); ++i) {
    const auto s = driver.state(i);
    std::printf("  %-8s q=%9.4f rad  dq=%8.4f rad/s  motor=%9.3f rot  I=%6.2f A  ref=%9.4f%s\n",
      driver.jointConfig(i).name.c_str(), s.position_rad, s.velocity_rad_s,
      s.motor_position_rot, s.current_a, s.reference_rad, s.position_fresh ? "" : "  (viejo)");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  if (argc < 2) {
    std::fprintf(stderr,
      "Uso: %s <spark-frames.json> [can0] [eje] [objetivo_rad] [vel_rad_s]\n", argv[0]);
    return 2;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto config = movemaster_driver::makeDefaultConfig(argv[1]);
  if (argc > 2) {config.can_interface = argv[2];}
  const bool move = argc > 4;
  const std::size_t joint = argc > 3 ? static_cast<std::size_t>(std::atoi(argv[3])) : 0;
  const double target = move ? std::atof(argv[4]) : 0.0;
  const double speed = argc > 5 ? std::fabs(std::atof(argv[5])) : 0.2;

  try {
    movemaster_driver::MoveMasterDriver driver(config);
    driver.open();
    std::printf("Configurando %zu SPARK MAX en %s...\n", driver.jointCount(),
      config.can_interface.c_str());
    driver.configure();
    if (!driver.waitForFeedback(2s)) {
      std::fprintf(stderr, "Sin telemetría STATUS_0/STATUS_2: %s\n", driver.fault().c_str());
      return 1;
    }
    print(driver);
    if (!move) {return 0;}
    if (joint >= driver.jointCount()) {
      std::fprintf(stderr, "Eje fuera de rango\n");
      return 2;
    }

    driver.arm();
    double reference = driver.state(joint).reference_rad;
    const auto period = 20ms;
    auto next = std::chrono::steady_clock::now();
    int tick = 0;
    while (!g_stop) {
      const double step = speed * std::chrono::duration<double>(period).count();
      const double error = target - reference;
      reference += std::fabs(error) <= step ? error : std::copysign(step, error);
      driver.setReference(joint, reference);
      if (!driver.sendReferences()) {
        std::fprintf(stderr, "Fallo: %s\n", driver.fault().c_str());
        return 1;
      }
      if (++tick % 25 == 0) {print(driver);}
      const auto s = driver.state(joint);
      if (reference == target && std::fabs(s.position_rad - target) < 0.01) {
        std::printf("Objetivo alcanzado.\n");
        print(driver);
        break;
      }
      next += period;
      std::this_thread::sleep_until(next);
    }
    driver.disarm();
    driver.close();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "Error: %s\n", error.what());
    return 1;
  }
  return 0;
}
