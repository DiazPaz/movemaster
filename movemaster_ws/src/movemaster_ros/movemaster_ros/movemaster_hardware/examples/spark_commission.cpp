// Puesta en marcha de los SPARK MAX: deja en su flash solo CAN ID, tipo de motor, idle mode y
// limite de corriente; todo lo demas vuelve al valor de fabrica. Sin --apply no transmite nada.
#include "movemaster_hardware/movemaster_driver.hpp"
#include "movemaster_hardware/socketcan.hpp"
#include <algorithm>
#include <iostream>
#include <set>

namespace spark_commission {
using namespace movemaster;

struct Request {
  bool apply = false;
  std::vector<const JointConfig *> joints;  // en el orden de JOINTS.json
};

// Argumentos despues de "SPEC JOINTS canal": --apply y, opcionalmente, nombres de joints.
inline Request parse(const DriverConfig &config, const std::vector<std::string> &args) {
  Request request;
  std::set<std::string> names;
  for (const auto &arg : args) {
    if (arg == "--apply") request.apply = true;
    else names.insert(arg);
  }
  for (const auto &name : names)
    if (std::none_of(config.joints.begin(), config.joints.end(), [&](const JointConfig &j) { return j.name == name; }))
      throw std::invalid_argument("Joint desconocido en JOINTS.json: " + name);
  for (const auto &joint : config.joints)
    if (names.empty() || names.count(joint.name)) request.joints.push_back(&joint);
  return request;
}

// Tramas que la herramienta necesita recibir: presencia, respuestas y heartbeat ajeno.
inline std::vector<std::uint32_t> filters(const SparkFrameDatabase &frames, const Request &request) {
  std::vector<std::uint32_t> ids{kEnableHeartbeatId};
  for (const auto *joint : request.joints)
    for (const auto *name : {"STATUS_0", "PARAMETER_WRITE_RESPONSE", "RESET_SAFE_PARAMETERS_RESPONSE",
             "PERSIST_PARAMETERS_RESPONSE"})
      ids.push_back(frames[name].arbitration_id(joint->device_id));
  return ids;
}

// Todo salvo abrir SocketCAN, para poder probarlo con un bus simulado.
inline int run(const DriverConfig &config, const Request &request, CANBus &bus, std::ostream &out,
    double listen_s = 0.5, double flash_timeout_s = 2.5) {
  const SparkFrameDatabase frames(config.spec_path);
  const auto survey = survey_bus(bus, frames, listen_s);
  bool ready = !survey.enable_heartbeat;
  out << "Puesta en marcha en " << config.channel << ". Queda en la flash de cada SPARK:\n";
  for (const auto *joint : request.joints) {
    const bool present = survey.sparks.count(joint->device_id) != 0;
    ready = ready && present;
    out << "  " << joint->name << "  CAN " << joint->device_id << (present ? "  presente" : "  NO RESPONDE")
        << "  | brushless, idle " << (joint->spark.idle_mode == IdleMode::kBrake ? "brake" : "coast")
        << ", limite " << joint->spark.current_limit_a << " A\n";
  }
  if (survey.enable_heartbeat)
    out << "Hay un heartbeat de habilitacion en el bus: cierra el controller_manager, "
        "spark_console o el backend Python antes de guardar.\n";
  if (!ready) return 1;
  if (!request.apply) {
    out << "Revision sin cambios. Con --apply, cada SPARK vuelve a sus parametros de fabrica\n"
        "(conserva CAN ID, tipo de motor e idle mode), recibe estos valores y los guarda en flash.\n";
    return 0;
  }
  for (const auto *joint : request.joints) {
    try {
      const SparkMAXMotionProtocol spark(config.spec_path, joint->device_id, config.parameter_layout);
      SparkSetup(bus, spark, config.response_timeout_s, flash_timeout_s).commission(joint->spark);
    } catch (const std::exception &e) {
      out << "Error en " << joint->name << ": " << e.what() << "\nLos ejes anteriores quedaron guardados. "
          "Repite la puesta en marcha de " << joint->name << " antes de usarlo.\n";
      return 1;
    }
    out << "  " << joint->name << ": guardado en flash.\n";
  }
  out << "Listo. El driver escribe en RAM todo lo demas cada vez que configura.\n";
  return 0;
}
}  // namespace spark_commission

#ifndef MOVEMASTER_COMMISSION_NO_MAIN
int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "Uso: spark_commission SPEC.json JOINTS.json can0 [--apply] [joint ...]\n"
        << "Sin --apply solo escucha el bus y muestra lo que quedaria en la flash.\n";
    return 2;
  }
  try {
    const auto config = movemaster::load_driver_config(argv[2], argv[1], argv[3]);
    movemaster::validate_driver_config(config);
    const auto request = spark_commission::parse(config, std::vector<std::string>(argv + 4, argv + argc));
    movemaster::SocketCAN bus(config.channel);
    bus.set_filters(spark_commission::filters(movemaster::SparkFrameDatabase(config.spec_path), request));
    return spark_commission::run(config, request, bus, std::cout);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
  }
}
#endif
