// Consola de un SPARK MAX. SP absoluto en rotaciones del encoder del motor.
// Reutiliza el driver: modos Position y MAXMotion, slots, feedback, limites y heartbeat.
#include "movemaster_hardware/movemaster_driver.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <sstream>
#include <system_error>
#include <thread>

namespace spark_console {
constexpr double kTau = 6.283185307179586476925286766559;
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }

// Leer la terminal no debe detener el envio periodico del heartbeat.
class NonblockingInput {
 public:
  explicit NonblockingInput(int fd) : fd_(fd), original_flags_(::fcntl(fd, F_GETFL)) {
    if (original_flags_ < 0 || ::fcntl(fd_, F_SETFL, original_flags_ | O_NONBLOCK) < 0)
      throw std::system_error(errno, std::generic_category(), "No se pudo preparar la terminal");
  }
  ~NonblockingInput() { ::fcntl(fd_, F_SETFL, original_flags_); }
  NonblockingInput(const NonblockingInput &) = delete;
  NonblockingInput &operator=(const NonblockingInput &) = delete;
  bool receive() {
    char data[128];
    const auto n = ::read(fd_, data, sizeof(data));
    if (n == 0) return false;  // EOF / terminal cerrada: salir y deshabilitar.
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      throw std::system_error(errno, std::generic_category(), "Error leyendo terminal");
    if (n > 0) buffer.append(data, static_cast<std::size_t>(n));
    if (buffer.size() > 512) throw std::runtime_error("Entrada demasiado larga");
    return true;
  }
  std::string buffer;
 private:
  int fd_, original_flags_;
};

inline const char *mode_name(movemaster::ControlMode mode) {
  return mode == movemaster::ControlMode::kPosition ? "position" : "maxmotion";
}

// El parametro Driver permite probar este mismo bucle sin conectar un motor.
template<class Driver>
void run(Driver &driver, const movemaster::DriverConfig &config,
    int input_fd = STDIN_FILENO, std::ostream &out = std::cout) {
  using movemaster::ControlMode;
  struct DisableOnExit { Driver &driver; ~DisableOnExit() { driver.deactivate(); } } guard{driver};
  if (config.joints.size() != 1) throw std::invalid_argument("Esta consola requiere exactamente un joint en el JSON");
  const auto &joint = config.joints.front();
  NonblockingInput input(input_fd);
  std::vector<double> target(1, 0.0);
  const auto tick = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config.period_s));
  const char *help = "Comandos: on | sp <rotaciones> | mode position|maxmotion | slot <0-3> | pv | off | q\n";
  // El driver valida el slot y el modo; aqui solo se informa por que no cambio.
  const auto change_control = [&](ControlMode mode, int slot) {
    try {
      driver.set_control(0, mode, slot);
      return true;
    } catch (const std::invalid_argument &e) {
      out << "Sin cambios: " << e.what() << '\n';
      return false;
    }
  };
  out << "SPARK MAX | CAN ID " << joint.device_id << " | " << mode_name(driver.controls().front().mode)
      << " | slot " << driver.controls().front().slot
      << "\nSP absoluto en rotaciones del MOTOR. Inicialmente deshabilitado.\n" << help
      << "off/q dejan de enviar heartbeat; no son una parada de emergencia.\n" << std::flush;

  while (!stop_requested) {
    const auto next = std::chrono::steady_clock::now() + tick;
    driver.read();
    if (!input.receive()) break;
    const auto newline = input.buffer.find('\n');
    if (newline != std::string::npos) {
      std::istringstream line(input.buffer.substr(0, newline));
      input.buffer.erase(0, newline + 1);
      std::string command, extra;
      line >> command;
      const auto control = driver.controls().front();
      if (command == "sp") {
        double rotations = 0;
        if (!(line >> rotations) || (line >> extra) || !std::isfinite(rotations)) {
          out << "Formato: sp 0.5 (numero finito, punto decimal).\n";
        } else if (!driver.active()) {
          out << "Primero escribe on. No se guardo el SP.\n";
        } else {
          const double radians = movemaster::MoveMasterDriver::rotations_to_radians(rotations, joint);
          const double measured = driver.states().front().position;
          if (!std::isfinite(radians) || radians < joint.min_position_rad || radians > joint.max_position_rad) {
            driver.deactivate();
            out << "SP fuera de limites: eje deshabilitado.\n";
          } else if (control.mode == ControlMode::kPosition &&
              !(std::abs(radians - measured) <= joint.max_following_error_rad)) {
            // Position no tiene perfil: un salto grande iria a la salida maxima del PID.
            out << "En modo position el SP debe quedar a menos de "
                << joint.max_following_error_rad * joint.gear_ratio / kTau
                << " rot del PV (max_following_error_rad). Usa mode maxmotion para movimientos largos.\n";
          } else {
            target[0] = radians;
            out << "SP = " << rotations << " rot motor (" << radians << " rad articulacion).\n";
          }
        }
      } else if (command == "mode") {
        std::string name;
        if (!(line >> name) || (line >> extra) || (name != "position" && name != "maxmotion")) {
          out << "Formato: mode position o mode maxmotion.\n";
        } else if (change_control(name == "position" ? ControlMode::kPosition : ControlMode::kMAXMotionPosition,
                       control.slot)) {
          out << "Modo " << name << ", slot " << control.slot << '.';
          // Al cambiar de modo, el SP pendiente del modo anterior no debe aplicarse.
          if (driver.active()) {
            target[0] = driver.states().front().position;
            out << " Mantiene la posicion medida.";
          }
          out << '\n';
        }
      } else if (command == "slot") {
        int slot = 0;
        if (!(line >> slot) || (line >> extra)) out << "Formato: slot 0 (entero de 0 a 3).\n";
        else if (change_control(control.mode, slot)) out << "Slot " << slot << " en modo " << mode_name(control.mode) << ".\n";
      } else if (line >> extra) {
        out << "Comando no valido. " << help;
      } else if (command == "on") {
        if (driver.active()) out << "El eje ya esta habilitado.\n";
        else {
          driver.activate();  // Primero envia PV como SP; despues habilita.
          target[0] = driver.states().front().position;
          out << "Habilitado manteniendo la posicion medida.\n";
        }
      } else if (command == "off") {
        driver.deactivate();
        out << "Heartbeat detenido; deshabilitacion por watchdog del SPARK.\n";
      } else if (command == "pv") {
        const auto &state = driver.states().front();
        out << "Ultima PV = " << movemaster::MoveMasterDriver::radians_to_rotations(state.position, joint)
            << " rot motor | q = " << state.position << " rad | I = " << state.current
            << " A | " << mode_name(control.mode) << " slot " << control.slot << " | "
            << (driver.active() ? "habilitado" : "deshabilitado") << '\n';
      } else if (command == "q") break;
      else if (!command.empty()) out << help;
      out << std::flush;
    }
    if (stop_requested) break;
    if (driver.active()) driver.write(target);  // Repite SP + heartbeat segun period_s.
    std::this_thread::sleep_until(next);
  }
}
}  // namespace spark_console

#ifndef MOVEMASTER_CONSOLE_NO_MAIN
int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Uso: spark_console SPEC.json JOINTS.json can0\n";
    return 2;
  }
  try {
    auto config = movemaster::load_driver_config(argv[2], argv[1], argv[3]);
    if (config.joints.size() != 1) throw std::invalid_argument("Deja un solo joint en JOINTS.json");
    std::signal(SIGINT, spark_console::request_stop);
    std::signal(SIGTERM, spark_console::request_stop);
    movemaster::MoveMasterDriver driver(config);
    std::cout << "Configurando en RAM baseline, slots y feedback del SPARK...\n";
    driver.configure();
    spark_console::run(driver, config);
    std::cout << "Consola cerrada.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\nSe detuvieron los envios CAN.\n";
    return 1;
  }
}
#endif
