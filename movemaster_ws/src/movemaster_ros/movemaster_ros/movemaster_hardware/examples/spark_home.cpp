// Homing de MoveMaster: estado de la referencia de cada eje, medicion de topes y homing.
//   spark_home SPEC.json JOINTS.json can0                           estado; no mueve ni borra nada
//   spark_home SPEC.json JOINTS.json can0 --measure joint_2 [+|-]   mide los topes de un eje
//   spark_home SPEC.json JOINTS.json can0 --home [joint_N ...] [--pose joint_1=0,joint_2=-1.5708,...]
//       busca el tope homing.stop de cada eje sin referencia (los de homing_order, en ese orden, o
//       los nombrados aunque la tengan), le escribe a su SPARK la posicion que le corresponde
//       (stop_rad en el tope) y lleva el brazo a --pose.
// Angulos en la convencion DH: + es antihorario alrededor del eje z de la articulacion.
#include "movemaster_hardware/movemaster_driver.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

namespace spark_home {
using namespace movemaster;
constexpr double kDegreesPerRadian = 57.29577951308232;
// La muneca del gripper llego a su objetivo: su ESP32 cuenta pasos, sin error de seguimiento.
constexpr double kGripperArrivalRad = 0.01;
volatile std::sig_atomic_t stop_requested = 0;
inline void request_stop(int) { stop_requested = 1; }
struct Interrupted : std::runtime_error { using std::runtime_error::runtime_error; };

struct Request {
  std::optional<std::size_t> joint;    // --measure; sin --measure ni --home: solo estado
  std::vector<int> directions;         // topes a medir: +1 antihorario, -1 horario
  bool home = false;                   // --home
  std::vector<std::size_t> named;      // --home joint_N ...: solo estos ejes, en este orden
  std::map<std::size_t, double> pose;  // --pose: a donde va el brazo al final, rad
};

inline std::size_t joint_index(const DriverConfig &config, const std::string &name) {
  for (std::size_t i = 0; i < config.joints.size(); ++i)
    if (config.joints[i].name == name) return i;
  throw std::invalid_argument("Joint desconocido en JOINTS.json: " + name);
}
// "joint_1=0,joint_2=-1.5708": cada eje una vez, dentro de sus limites de JOINTS.json.
inline std::map<std::size_t, double> parse_pose(const DriverConfig &config, const std::string &text) {
  std::map<std::size_t, double> pose;
  std::istringstream items(text);
  for (std::string item; std::getline(items, item, ',');) {
    const auto equals = item.find('=');
    if (equals == std::string::npos)
      throw std::invalid_argument("--pose espera joint_N=rad separados por comas, no " + item);
    const auto index = joint_index(config, item.substr(0, equals));
    const auto &joint = config.joints[index];
    const auto number = item.substr(equals + 1);
    std::size_t used = 0;
    double value = std::numeric_limits<double>::quiet_NaN();
    try { value = std::stod(number, &used); } catch (const std::exception &) {}
    if (number.empty() || used != number.size() || !std::isfinite(value))
      throw std::invalid_argument("--pose: " + item + " no es un angulo en rad");
    if (value < joint.min_position_rad || value > joint.max_position_rad)
      throw std::invalid_argument("--pose: " + joint.name + " = " + number + " rad esta fuera de "
          "min_position_rad..max_position_rad");
    if (!pose.emplace(index, value).second) throw std::invalid_argument("--pose: " + joint.name + " repetido");
  }
  if (pose.empty()) throw std::invalid_argument("--pose espera joint_N=rad separados por comas");
  return pose;
}
// Argumentos despues de "SPEC JOINTS canal".
inline Request parse(const DriverConfig &config, const std::vector<std::string> &args) {
  Request request;
  if (args.empty()) return request;
  if (args[0] == "--home") {
    request.home = true;
    for (std::size_t i = 1; i < args.size(); ++i) {
      if (args[i] == "--pose") {
        if (i + 1 == args.size() || !request.pose.empty())
          throw std::invalid_argument("Se espera una sola --pose joint_N=rad,...");
        request.pose = parse_pose(config, args[++i]);
        continue;
      }
      const auto index = joint_index(config, args[i]);
      const auto &homing = config.joints[index].homing;
      if (!homing || homing->stop == 0)
        throw std::invalid_argument(args[i] + " no tiene homing.stop y homing.stop_rad en JOINTS.json");
      if (std::count(request.named.begin(), request.named.end(), index))
        throw std::invalid_argument(args[i] + " repetido");
      request.named.push_back(index);
    }
    if (request.named.empty() && config.homing_order.empty())
      throw std::invalid_argument("JOINTS.json no tiene homing_order: agregalo o nombra los ejes despues de --home");
    return request;
  }
  if (args[0] != "--measure" || args.size() < 2 || args.size() > 3)
    throw std::invalid_argument("Se espera --measure joint_N [+|-] o --home [joint_N ...] [--pose joint_N=rad,...]");
  request.joint = joint_index(config, args[1]);
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

// Que ejes se reiniciaron, con el driver configurado con SparkResetPolicy::kReport: el aviso
// HAS_RESET queda puesto para quien configure despues. No habilita motores.
template<class Driver>
int status(Driver &driver, const DriverConfig &config, std::ostream &out) {
  std::this_thread::sleep_for(std::chrono::milliseconds(3 * config.status_period_ms));
  driver.read();
  out << "Estado de los ejes en " << config.channel << ":\n";
  bool any = false;
  for (std::size_t i = 0; i < config.joints.size(); ++i) {
    const auto &joint = config.joints[i];
    const auto &state = driver.states()[i];
    any = any || state.spark_reset;
    out << "  " << joint.name << (joint.device == JointDevice::kGripper ? "  ESP32 " : "  CAN ") << joint.device_id
        << (state.spark_reset ? "  reiniciado  " : "  sin reinicio")
        << "  posicion " << angle(state.position) << '\n';
  }
  if (!any)
    out << "Ningun eje se reinicio desde que se tomo su referencia: la conservan.\n";
  else
    out << "Los ejes reiniciados no tienen referencia: cuentan desde la postura en que se encendieron.\n"
        "Su aviso de reinicio sigue puesto hasta que spark_home --home (el bringup lo corre al arrancar)\n"
        "busca sus topes y les escribe la referencia.\n";
  return 0;
}

// El lazo de spark_home: cada period_s lee, ajusta los objetivos y los envia todos. Los ejes que
// nadie mueve se sostienen donde estaban al activar.
template<class Driver>
struct Cycles {
  Driver &driver;
  std::chrono::steady_clock::duration tick;
  std::vector<double> targets;
  std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
  Cycles(Driver &d, double period_s)
      : driver(d), tick(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(period_s))) {
    for (const auto &state : driver.states()) targets.push_back(state.position);
  }
  // step(states, dt) ajusta targets.
  template<class Step> void operator()(Step step) {
    if (stop_requested) throw Interrupted("Interrumpido");
    const auto next = std::chrono::steady_clock::now() + tick;
    driver.read();
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    step(driver.states(), dt);
    driver.write(targets);
    std::this_thread::sleep_until(next);
  }
};

// Un eje que ramp_to lleva a goal a speed. lead: cuanto puede adelantarse su objetivo a la
// posicion medida (modo Position); tolerance: a que distancia de goal ya llego.
struct Move {
  std::size_t joint;
  double goal, speed, lead, tolerance;
};
// Mueve los ejes a la vez, cada uno con su rampa. Lanza what si no llegan en el doble del tiempo
// previsto, mas 2 s.
template<class Driver>
void ramp_to(Cycles<Driver> &cycles, const std::vector<Move> &moves, const std::string &what) {
  double expected = 0;
  for (const auto &m : moves)
    expected = std::max(expected, std::abs(m.goal - cycles.driver.states()[m.joint].position) / m.speed);
  double elapsed = 0;
  for (bool arrived = false; !arrived;) {
    cycles([&](const std::vector<JointState> &states, double dt) {
      elapsed += dt;
      arrived = true;
      for (const auto &m : moves) {
        auto &target = cycles.targets[m.joint];
        target = ramp_target(target, m.goal, m.speed, dt, states[m.joint].position, m.lead);
        arrived = arrived && target == m.goal && std::abs(states[m.joint].position - m.goal) <= m.tolerance;
      }
    });
    if (!arrived && elapsed > 2 * expected + 2) throw std::runtime_error(what);
  }
}

// Mientras un eje busca un tope: su limite de corriente de homing. Al salir, por la razon que
// sea: sin heartbeat, con el limite de corriente y el modo de control de siempre.
template<class Driver>
struct SearchGuard {
  Driver &driver;
  const JointConfig &joint;
  std::size_t index;
  std::ostream &out;
  SearchGuard(Driver &d, const JointConfig &j, std::size_t i, std::ostream &o) : driver(d), joint(j), index(i), out(o) {
    driver.set_current_limit(index, joint.homing->current_limit_a);
  }
  ~SearchGuard() {
    driver.deactivate();
    try {
      driver.set_control(index, joint.mode, joint.slot);
      driver.set_current_limit(index, joint.spark.current_limit_a);
    } catch (const std::exception &e) {
      out << "Aviso: no se restauro el limite de corriente (" << e.what() << "). El controller_manager\n"
          "lo vuelve a escribir al configurar.\n";
    }
  }
  SearchGuard(const SearchGuard &) = delete;
  SearchGuard &operator=(const SearchGuard &) = delete;
};

// Corre la busqueda en el eje index hasta que encuentra el tope o falla.
template<class Driver>
void search_stop(Cycles<Driver> &cycles, std::size_t index, StopSearch &search, std::ostream &out) {
  for (auto phase = search.phase(); phase != StopSearch::Phase::kFound && phase != StopSearch::Phase::kFailed;) {
    cycles([&](const std::vector<JointState> &states, double dt) {
      cycles.targets[index] = search.update(states[index].position, states[index].current, dt);
    });
    if (search.phase() == StopSearch::Phase::kBackoff && phase != StopSearch::Phase::kBackoff)
      out << "   contacto; retrocede y se acerca despacio\n" << std::flush;
    phase = search.phase();
  }
}
inline void report_failure(const StopSearch &search, double current, std::ostream &out) {
  out << "   Error: " << search.error() << ".\n   corriente: pico en movimiento libre "
      << amps(search.free_current_peak()) << ", al final " << amps(current) << '\n';
}

// Busca cada tope pedido con StopSearch y vuelve a la posicion inicial despues de cada uno.
// El eje medido va en modo Position con su limite de corriente de homing; los demas se sostienen.
template<class Driver>
int measure(Driver &driver, const DriverConfig &config, std::size_t index, const std::vector<int> &directions,
    std::ostream &out) {
  const auto &joint = config.joints.at(index);
  const auto &homing = joint.homing.value();
  const double lead = search_lead(joint.max_following_error_rad);
  out << joint.name << ": angulos respecto a "
      << (driver.referenced(index) ? "su referencia (convencion DH).\n"
                                   : "la postura en que se encendio: todavia no tiene referencia.\n")
      << "Limite de corriente " << homing.current_limit_a << " A mientras mide.\n";
  SearchGuard<Driver> guard(driver, joint, index, out);
  driver.activate();
  Cycles<Driver> cycles(driver, config.period_s);
  driver.set_control(index, ControlMode::kPosition, joint.slot);
  const double start = cycles.targets[index];
  const auto move_to = [&](double goal) {
    ramp_to(cycles, {{index, goal, homing.fast_speed_rad_s, lead, kArrivalFraction * lead}},
        "el eje no regreso a la posicion inicial");
  };
  std::map<int, double> stops;
  try {
    for (const int direction : directions) {
      const char *sign = direction > 0 ? "+" : "-";
      out << sign << ": buscando el tope " << (direction > 0 ? "antihorario" : "horario") << "...\n" << std::flush;
      StopSearch search(homing, direction, lead);
      search_stop(cycles, index, search, out);
      if (search.phase() == StopSearch::Phase::kFailed) {
        report_failure(search, driver.states()[index].current, out);
        out << "   Regresa a la posicion inicial.\n";
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

// Referencia un eje contra su tope homing.stop: lo busca, se retira backoff_rad y, ya sin
// heartbeat, le escribe al SPARK la posicion que corresponde (stop_rad en el tope).
template<class Driver>
int find_reference(Driver &driver, const DriverConfig &config, std::size_t index, std::ostream &out) {
  const auto &joint = config.joints.at(index);
  const auto &homing = joint.homing.value();
  const double lead = search_lead(joint.max_following_error_rad);
  out << joint.name << ": buscando el tope " << (homing.stop > 0 ? "+" : "-") << ", en " << angle(homing.stop_rad)
      << ", con " << homing.current_limit_a << " A...\n" << std::flush;
  SearchGuard<Driver> guard(driver, joint, index, out);
  driver.activate();
  Cycles<Driver> cycles(driver, config.period_s);
  driver.set_control(index, ControlMode::kPosition, joint.slot);
  const double start = cycles.targets[index];
  StopSearch search(homing, homing.stop, lead);
  search_stop(cycles, index, search, out);
  if (search.phase() == StopSearch::Phase::kFailed) {
    report_failure(search, driver.states()[index].current, out);
    out << "   " << joint.name << " queda sin referencia; sin heartbeat, los SPARK se deshabilitan.\n";
    return 1;
  }
  const double found = search.stop();
  ramp_to(cycles, {{index, found - homing.stop * homing.backoff_rad, homing.fast_speed_rad_s, lead,
      kArrivalFraction * lead}}, "el eje no se retiro del tope");
  driver.deactivate();
  driver.shift_reference(index, homing.stop_rad - found);
  out << "   tope a " << angle(found - start) << " de donde empezo, al topar " << amps(search.stop_current())
      << ".\n   Referenciado: queda en "
      << angle(driver.states()[index].position) << '\n';
  return 0;
}

// Lleva los ejes de pose a la vez, en linea recta en el espacio articular: cada uno a la velocidad
// con la que llega junto con el mas lento (fast_speed_rad_s de su homing; sin homing,
// max_velocity_rad_s). Los demas se sostienen.
template<class Driver>
void move_to_pose(Driver &driver, const DriverConfig &config, const std::map<std::size_t, double> &pose,
    std::ostream &out) {
  out << "Moviendo a --pose...\n" << std::flush;
  struct Guard {
    Driver &driver;
    const DriverConfig &config;
    const std::map<std::size_t, double> &pose;
    ~Guard() {
      driver.deactivate();
      for (const auto &entry : pose) {
        const auto &joint = config.joints[entry.first];
        // A validated joint always takes its own mode and slot back.
        try { if (joint.device == JointDevice::kSpark) driver.set_control(entry.first, joint.mode, joint.slot); }
        catch (const std::exception &) {}
      }
    }
  } guard{driver, config, pose};
  driver.activate();
  Cycles<Driver> cycles(driver, config.period_s);
  const auto speed = [&](const JointConfig &joint) {
    return joint.homing ? joint.homing->fast_speed_rad_s : joint.max_velocity_rad_s;
  };
  double duration = 0;
  for (const auto &[i, goal] : pose)
    duration = std::max(duration, std::abs(goal - cycles.targets[i]) / speed(config.joints[i]));
  std::vector<Move> moves;
  for (const auto &[i, goal] : pose) {
    const auto &joint = config.joints[i];
    const double distance = std::abs(goal - cycles.targets[i]);
    const double synchronized = distance > 0 ? distance / duration : speed(joint);
    if (joint.device == JointDevice::kSpark) {
      driver.set_control(i, ControlMode::kPosition, joint.slot);
      const double lead = search_lead(joint.max_following_error_rad);
      moves.push_back({i, goal, synchronized, lead, kArrivalFraction * lead});
    } else {
      // El ESP32 perfila el movimiento de la muneca: su objetivo no tiene que ir pegado a ella.
      moves.push_back({i, goal, synchronized, std::numeric_limits<double>::infinity(), kGripperArrivalRad});
    }
  }
  ramp_to(cycles, moves, "el brazo no llego a --pose");
  for (const auto &[i, goal] : pose)
    out << "  " << config.joints[i].name << "  " << angle(driver.states()[i].position) << '\n';
}

// Homing: referencia los ejes y lleva el brazo a request.pose. Busca el tope de los ejes de
// homing_order sin referencia (o de los nombrados), y los ejes sin referencia que no estan en
// homing_order toman la postura en que se encendieron. Si todos tienen referencia, no mueve
// nada. config: los limites de JOINTS.json, con los que se revisa --pose; el driver los tiene
// ampliados en max_travel_rad, porque antes de su referencia un eje cuenta desde donde se encendio.
template<class Driver>
int home(Driver &driver, const DriverConfig &config, const Request &request, std::ostream &out) {
  const auto has = [](const std::vector<std::size_t> &list, std::size_t i) {
    return std::find(list.begin(), list.end(), i) != list.end();
  };
  std::vector<std::size_t> search = request.named, power_on;
  if (request.named.empty()) {
    std::vector<std::size_t> ordered;
    for (const auto &name : config.homing_order) ordered.push_back(joint_index(config, name));
    for (const auto i : ordered)
      if (!driver.referenced(i)) search.push_back(i);
    for (std::size_t i = 0; i < config.joints.size(); ++i)
      if (!driver.referenced(i) && !has(ordered, i)) power_on.push_back(i);
  }
  for (const auto &entry : request.pose)
    if (!driver.referenced(entry.first) && !has(search, entry.first) && !has(power_on, entry.first))
      throw std::invalid_argument(config.joints[entry.first].name +
          " no tiene referencia y no se busca su tope: no puede ir a --pose");
  if (search.empty() && power_on.empty()) {
    out << "Todos los ejes conservan su referencia: no se busca ningun tope ni se mueve nada.\n";
    return 0;
  }
  try {
    for (const auto i : power_on) {
      driver.shift_reference(i, 0);
      out << config.joints[i].name << ": sin tope que buscar; su referencia es la postura en que se encendio, "
          << angle(driver.states()[i].position) << ".\n";
    }
    for (const auto i : search)
      if (const int code = find_reference(driver, config, i, out); code != 0) return code;
    if (request.pose.empty()) {
      out << "Ejes referenciados. Sin --pose se quedan donde termino su busqueda.\n";
      return 0;
    }
    move_to_pose(driver, config, request.pose, out);
  } catch (const Interrupted &) {
    out << "Interrumpido: sin heartbeat, los SPARK se deshabilitan.\n";
    return 130;
  }
  out << "Homing terminado.\n";
  return 0;
}
}  // namespace spark_home

#ifndef MOVEMASTER_HOME_NO_MAIN
int main(int argc, char **argv) {
  // An option among the first three arguments means an empty $SPEC or $JOINTS shifted them.
  if (argc < 4 || argv[1][0] == '-' || argv[2][0] == '-' || argv[3][0] == '-') {
    std::cerr << "Uso: spark_home SPEC.json JOINTS.json can0 [--measure joint_N [+|-]]\n"
        << "       spark_home SPEC.json JOINTS.json can0 --home [joint_N ...] [--pose joint_N=rad,...]\n"
        << "Sin opciones muestra que ejes se reiniciaron, sin habilitar motores.\n"
        << "Si usas $SPEC y $JOINTS, definelos en esta terminal (MANUAL 4.5, paso 3).\n";
    return 2;
  }
  try {
    const auto config = movemaster::load_driver_config(argv[2], argv[1], argv[3]);
    const auto request = spark_home::parse(config, std::vector<std::string>(argv + 4, argv + argc));
    // Sin referencia, un eje cuenta desde donde se encendio: sus topes pueden quedar hasta
    // max_travel_rad fuera de sus limites. El driver los amplia; --pose se revisa con los de JOINTS.json.
    auto widened = config;
    for (auto &joint : widened.joints)
      if (joint.homing && (request.home || (request.joint && &joint == &widened.joints[*request.joint]))) {
        joint.min_position_rad -= joint.homing->max_travel_rad;
        joint.max_position_rad += joint.homing->max_travel_rad;
      }
    std::signal(SIGINT, spark_home::request_stop);
    std::signal(SIGTERM, spark_home::request_stop);
    movemaster::MoveMasterDriver driver(widened);
    if (!request.joint && !request.home) {
      driver.configure(movemaster::SparkResetPolicy::kReport);
      return spark_home::status(driver, config, std::cout);
    }
    // kHoming: los ejes reiniciados siguen sin referencia hasta que --home se las escribe.
    driver.configure(movemaster::SparkResetPolicy::kHoming);
    if (request.home) return spark_home::home(driver, config, request, std::cout);
    return spark_home::measure(driver, config, *request.joint, request.directions, std::cout);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\nSe detuvieron los envios CAN.\n";
    return 1;
  }
}
#endif
