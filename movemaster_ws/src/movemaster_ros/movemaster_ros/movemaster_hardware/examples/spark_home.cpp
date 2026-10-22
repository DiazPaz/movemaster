// Homing de MoveMaster, fase 1: estado de la referencia de cada SPARK y medicion de topes.
//   spark_home SPEC.json JOINTS.json can0                           estado; no mueve nada
//   spark_home SPEC.json JOINTS.json can0 --measure joint_2 [+|-]   mide los topes de un eje
// Angulos en la convencion DH: + es antihorario alrededor del eje z de la articulacion.
#include "movemaster_hardware/movemaster_driver.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace spark_home {
using namespace movemaster;
constexpr double kDegreesPerRadian = 57.29577951308232;
volatile std::sig_atomic_t stop_requested = 0;
inline void request_stop(int) { stop_requested = 1; }
struct Interrupted : std::runtime_error { using std::runtime_error::runtime_error; };

struct Request {
  std::optional<std::size_t> joint;  // sin --measure: solo estado
  std::vector<int> directions;       // topes a medir: +1 antihorario, -1 horario
};

// Argumentos despues de "SPEC JOINTS canal".
inline Request parse(const DriverConfig &config, const std::vector<std::string> &args) {
  Request request;
  if (args.empty()) return request;
  if (args[0] != "--measure" || args.size() < 2 || args.size() > 3)
    throw std::invalid_argument("Se espera --measure joint_N [+|-]");
  for (std::size_t i = 0; i < config.joints.size(); ++i)
    if (config.joints[i].name == args[1]) request.joint = i;
  if (!request.joint) throw std::invalid_argument("Joint desconocido en JOINTS.json: " + args[1]);
  if (!config.joints[*request.joint].homing)
    throw std::invalid_argument(args[1] + " no tiene bloque homing en JOINTS.json");
  if (args.size() == 2) request.directions = {1, -1};
  else if (args[2] == "+") request.directions = {1};
  else if (args[2] == "-") request.directions = {-1};
  else throw std::invalid_argument("El sentido es + (antihorario) o - (horario), no " + args[2]);
  return request;
}

inline std::string amps(double current) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(1) << current << " A";
  return out.str();
}
inline std::string angle(double radians) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << radians << " rad (" << std::setprecision(1)
      << radians * kDegreesPerRadian << " grados)";
  return out.str();
}

// Que SPARK se reiniciaron. configure() ya tomo su postura de encendido como referencia y borro
// el aviso, asi que el controller_manager detecta un reinicio posterior. No habilita motores.
template<class Driver>
int status(Driver &driver, const DriverConfig &config, const std::vector<std::string> &restarted,
    std::ostream &out) {
  std::this_thread::sleep_for(std::chrono::milliseconds(3 * config.status_period_ms));
  driver.read();
  out << "Estado de los SPARK en " << config.channel << ":\n";
  for (std::size_t i = 0; i < config.joints.size(); ++i) {
    const auto &joint = config.joints[i];
    const bool reset = std::count(restarted.begin(), restarted.end(), joint.name) != 0;
    out << "  " << joint.name << "  CAN " << joint.device_id << (reset ? "  reiniciado  " : "  sin reinicio")
        << "  posicion " << angle(driver.states()[i].position) << '\n';
  }
  if (restarted.empty())
    out << "Ningun SPARK se reinicio desde la ultima referencia: la conservan.\n";
  else
    out << "Los SPARK reiniciados cuentan desde la postura en que se encendieron, que zero_offset_rad\n"
        "supone Home. Su aviso de reinicio quedo borrado: si vuelven a reiniciarse, el\n"
        "controller_manager lo detecta como fallo.\n";
  return 0;
}

// Busca cada tope pedido con StopSearch y vuelve a la posicion inicial despues de cada uno.
// El eje medido va en modo Position con su limite de corriente de homing; los demas se sostienen.
template<class Driver>
int measure(Driver &driver, const DriverConfig &config, std::size_t index, const std::vector<int> &directions,
    std::ostream &out) {
  const auto &joint = config.joints.at(index);
  const auto &homing = joint.homing.value();
  const double lead = 0.5 * joint.max_following_error_rad;
  const auto tick = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config.period_s));
  out << joint.name << ": angulos respecto a la referencia actual (la postura de encendido del SPARK,\n"
      "Home por convencion). Limite de corriente " << homing.current_limit_a << " A mientras mide.\n";
  driver.set_current_limit(index, homing.current_limit_a);
  // Al salir, por la razon que sea: sin heartbeat y con el limite de corriente de siempre.
  struct Restore {
    Driver &driver;
    std::size_t index;
    int amps;
    std::ostream &out;
    ~Restore() {
      driver.deactivate();
      try { driver.set_current_limit(index, amps); }
      catch (const std::exception &e) {
        out << "Aviso: no se restauro el limite de corriente (" << e.what() << "). El controller_manager\n"
            "lo vuelve a escribir al configurar.\n";
      }
    }
  } restore{driver, index, joint.spark.current_limit_a, out};
  driver.activate();
  std::vector<double> targets;
  for (const auto &state : driver.states()) targets.push_back(state.position);
  driver.set_control(index, ControlMode::kPosition, joint.slot);
  const double start = targets[index];
  auto last = std::chrono::steady_clock::now();
  // Un ciclo: leer, elegir el objetivo del eje medido con step y enviar todos los objetivos.
  const auto cycle = [&](auto step) {
    if (stop_requested) throw Interrupted("Interrumpido");
    const auto next = std::chrono::steady_clock::now() + tick;
    driver.read();
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    targets[index] = step(driver.states()[index], dt);
    driver.write(targets);
    std::this_thread::sleep_until(next);
  };
  const auto move_to = [&](double goal) {
    const double limit = 2 * std::abs(goal - driver.states()[index].position) / homing.fast_speed_rad_s + 2;
    double elapsed = 0;
    bool arrived = false;
    while (!arrived) {
      cycle([&](const JointState &state, double dt) {
        elapsed += dt;
        const double target = ramp_target(targets[index], goal, homing.fast_speed_rad_s, dt, state.position, lead);
        arrived = target == goal && std::abs(state.position - goal) <= 0.25 * lead;
        return target;
      });
      if (!arrived && elapsed > limit) throw std::runtime_error("el eje no regreso a la posicion inicial");
    }
  };
  std::map<int, double> stops;
  try {
    for (const int direction : directions) {
      const char *sign = direction > 0 ? "+" : "-";
      out << sign << ": buscando el tope " << (direction > 0 ? "antihorario" : "horario") << "...\n" << std::flush;
      StopSearch search(homing, direction, lead);
      for (auto phase = search.phase(); phase != StopSearch::Phase::kFound && phase != StopSearch::Phase::kFailed;) {
        cycle([&](const JointState &state, double dt) { return search.update(state.position, state.current, dt); });
        if (search.phase() == StopSearch::Phase::kBackoff && phase != StopSearch::Phase::kBackoff)
          out << "   contacto; retrocede y se acerca despacio\n" << std::flush;
        phase = search.phase();
      }
      if (search.phase() == StopSearch::Phase::kFailed) {
        out << "   Error: " << search.error() << ".\n   corriente: pico en movimiento libre "
            << amps(search.free_current_peak()) << ", al final " << amps(driver.states()[index].current)
            << "\n   Regresa a la posicion inicial.\n";
        move_to(start);
        return 1;
      }
      stops[direction] = search.stop();
      out << "   tope " << sign << " en " << angle(search.stop()) << "\n   corriente: pico en movimiento libre "
          << amps(search.free_current_peak()) << ", al topar " << amps(search.stop_current()) << '\n';
      if (std::abs(search.stop() - start) < homing.backoff_rad)
        out << "   Aviso: el eje casi no se movio. Si no hay un tope ahi, sube current_limit_a.\n";
      move_to(start);
    }
  } catch (const Interrupted &) {
    out << "Interrumpido: sin heartbeat, los SPARK se deshabilitan.\n";
    return 130;
  }
  out << "Resultado de " << joint.name << " (convencion DH, + = antihorario):\n";
  for (const auto &[direction, stop] : stops) out << "  tope " << (direction > 0 ? "+" : "-") << ": " << angle(stop) << '\n';
  if (stops.size() == 2) {
    const double lower = stops.at(-1) + homing.backoff_rad, upper = stops.at(1) - homing.backoff_rad;
    out << "  recorrido entre topes: " << angle(stops.at(1) - stops.at(-1)) << "\n"
        << "Para joints.json, con backoff_rad de margen:\n" << std::fixed << std::setprecision(3)
        << "  \"min_position_rad\": " << lower << ", \"max_position_rad\": " << upper << '\n';
  }
  return 0;
}
}  // namespace spark_home

#ifndef MOVEMASTER_HOME_NO_MAIN
int main(int argc, char **argv) {
  // An option among the first three arguments means an empty $SPEC or $JOINTS shifted them.
  if (argc < 4 || argv[1][0] == '-' || argv[2][0] == '-' || argv[3][0] == '-') {
    std::cerr << "Uso: spark_home SPEC.json JOINTS.json can0 [--measure joint_N [+|-]]\n"
        << "Sin --measure muestra que SPARK se reiniciaron, sin habilitar motores.\n"
        << "Si usas $SPEC y $JOINTS, definelos en esta terminal (MANUAL 4.5, paso 3).\n";
    return 2;
  }
  try {
    auto config = movemaster::load_driver_config(argv[2], argv[1], argv[3]);
    const auto request = spark_home::parse(config, std::vector<std::string>(argv + 4, argv + argc));
    if (request.joint) {
      // Medir los topes es lo que fija min/max_position_rad: el eje medido puede pasar de los
      // limites actuales, hasta max_travel_rad.
      auto &joint = config.joints[*request.joint];
      joint.min_position_rad -= joint.homing->max_travel_rad;
      joint.max_position_rad += joint.homing->max_travel_rad;
    }
    std::signal(SIGINT, spark_home::request_stop);
    std::signal(SIGTERM, spark_home::request_stop);
    movemaster::MoveMasterDriver driver(config);
    const auto restarted = driver.configure();
    if (!request.joint) return spark_home::status(driver, config, restarted, std::cout);
    return spark_home::measure(driver, config, *request.joint, request.directions, std::cout);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\nSe detuvieron los envios CAN.\n";
    return 1;
  }
}
#endif
