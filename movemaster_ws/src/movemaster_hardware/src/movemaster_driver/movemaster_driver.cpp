#include "movemaster_driver/movemaster_driver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace movemaster
{

namespace
{

constexpr double TWO_PI = 2.0 * M_PI;
constexpr unsigned STATUS_0_AND_2 = 0b101;

/// _float32() del backend: valida y cuantiza a float32 (lo que viaja por CAN).
double to_float32(
  double value, const std::string & name,
  std::optional<double> minimum = std::nullopt, bool positive = false)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + ": debe ser finito");
  }
  if (std::fabs(value) > std::numeric_limits<float>::max()) {
    throw std::invalid_argument(name + ": valor no representable como float32");
  }
  value = static_cast<double>(static_cast<float>(value));
  if (positive && value <= 0.0) {
    throw std::invalid_argument(name + ": debe ser mayor que cero");
  }
  if (minimum && value < *minimum) {
    throw std::invalid_argument(name + ": debe ser >= " + std::to_string(*minimum));
  }
  return value;
}

/// _parameter() del backend: parámetros de setup fuera del layout PIDF/MAXMotion.
sparkmax::ParameterDefinition setup_parameter(
  const std::string & name, int number, const std::string & kind)
{
  return {"setup", name, 0, number, kind};
}

std::map<std::string, double> validate_pidf(
  const sparkmax::ParameterGroup & group, const std::map<std::string, double> & gains)
{
  if (gains.empty()) {
    throw std::invalid_argument("Proporciona al menos una ganancia PIDF");
  }
  std::map<std::string, double> values;
  for (const auto & [key, value] : gains) {
    const auto & canonical = group[key].key;
    values[canonical] = to_float32(value, canonical, 0.0);
  }
  return values;
}

std::map<std::string, double> validate_profile(
  const sparkmax::ParameterGroup & group, const std::map<std::string, double> & profile)
{
  if (profile.empty()) {
    throw std::invalid_argument("Proporciona al menos un parámetro de MAXMotion");
  }
  std::map<std::string, double> values;
  for (const auto & [key, value] : profile) {
    const auto & canonical = group[key].key;
    values[canonical] = canonical == "allowed_profile_error" ?
      to_float32(value, canonical, 0.0) :
      to_float32(value, canonical, std::nullopt, true);  // RPM/s y RPM > 0
  }
  return values;
}

}  // namespace

// -----------------------------------------------------------------------------
// Construcción y ciclo de vida
// -----------------------------------------------------------------------------

MoveMasterDriver::MoveMasterDriver(DriverConfig config, std::shared_ptr<sparkmax::CanBus> bus)
: config_(std::move(config)), bus_(std::move(bus)), owns_bus_(bus_ == nullptr)
{
  const auto & c = config_;
  if (c.joints.empty()) {
    throw std::invalid_argument("Se requiere al menos una articulación");
  }
  if (c.slot < 0 || c.slot > 3) {
    throw std::invalid_argument("slot debe ser un entero de 0 a 3");
  }
  if (c.status_period_ms < 1 || c.status_period_ms > 1000) {
    throw std::invalid_argument("status_period_ms debe ser entero entre 1 y 1000");
  }
  if (!std::isfinite(c.feedback_timeout_s) ||
    c.feedback_timeout_s < 3 * c.status_period_ms / 1000.0)
  {
    throw std::invalid_argument("El watchdog requiere al menos 3 períodos STATUS");
  }
  if (!std::isfinite(c.response_timeout_s) || c.response_timeout_s <= 0) {
    throw std::invalid_argument("response_timeout_s debe ser positivo");
  }
  if (!std::isfinite(c.disable_settle_s) || c.disable_settle_s < 0) {
    throw std::invalid_argument("disable_settle_s debe ser finito y no negativo");
  }

  std::set<int> ids;
  for (const auto & joint : c.joints) {
    const auto where = "Articulación '" + joint.name + "': ";
    if (!ids.insert(joint.device_id).second) {
      throw std::invalid_argument(where + "device_id repetido");
    }
    if (!std::isfinite(joint.gear_ratio) || joint.gear_ratio == 0.0) {
      throw std::invalid_argument(where + "gear_ratio debe ser finito y distinto de cero");
    }
    if (!std::isfinite(joint.offset)) {
      throw std::invalid_argument(where + "offset debe ser finito");
    }

    Axis axis(joint, sparkmax::SparkMAXMotionProtocol(
        c.spec_path, joint.device_id, c.parameter_layout));
    if (!joint.pidf.empty()) {
      axis.config.pidf = validate_pidf(axis.protocol.pidf(c.slot), joint.pidf);
    }
    if (!joint.motion_profile.empty()) {
      axis.config.motion_profile =
        validate_profile(axis.protocol.maxmotion(c.slot), joint.motion_profile);
    }
    axes_.push_back(std::move(axis));

    if (joint.position_limits) {
      const auto [lo, hi] = *joint.position_limits;
      if (!(std::isfinite(lo) && std::isfinite(hi) && lo < hi)) {
        throw std::invalid_argument(where + "límites: mínimo finito < máximo finito");
      }
      const double a = joint_to_motor(axes_.size() - 1, lo);
      const double b = joint_to_motor(axes_.size() - 1, hi);
      axes_.back().limits_rot = std::pair{std::min(a, b), std::max(a, b)};
    }
  }

  for (std::size_t i = 0; i < axes_.size(); ++i) {
    const auto & p = axes_[i].protocol;
    for (const char * name : {"STATUS_0", "STATUS_2", "PARAMETER_WRITE_RESPONSE",
        "SET_STATUSES_ENABLED_RESPONSE", "STOP_FOLLOWER_MODE_RESPONSE",
        "PERSIST_PARAMETERS_RESPONSE"})
    {
      rx_frames_[p.frames[name].arbitration_id(p.device_id)] = {i, name};
    }
  }

  hb_on_.arbitration_id = HEARTBEAT_ID;
  hb_on_.data = sparkmax::Bytes(8, 0xFF);
  hb_on_.dlc = 8;
  hb_on_.frame_name = "REFERENCE_HEARTBEAT";
}

MoveMasterDriver::~MoveMasterDriver()
{
  try {
    close();
  } catch (...) {
  }
}

void MoveMasterDriver::open()
{
  if (running_) {
    throw std::runtime_error("Driver ya iniciado");
  }
  if (!bus_) {
    std::vector<sparkmax::CanFilter> filters;
    for (const auto & [can_id, _] : rx_frames_) {
      filters.push_back({can_id});
    }
    bus_ = std::make_shared<sparkmax::SocketCanBus>(config_.channel, filters);
  }
  running_ = true;
}

void MoveMasterDriver::close()
{
  // Igual que heartbeat.stop() del ejemplo: dejar de transmitir. El watchdog
  // del firmware determina cuándo se deshabilita el motor.
  armed_ = false;
  running_ = false;
  if (owns_bus_) {
    bus_.reset();
  }
}

// -----------------------------------------------------------------------------
// Configuración (bloqueante, siempre desarmado)
// -----------------------------------------------------------------------------

void MoveMasterDriver::check_running() const
{
  if (!running_ || !bus_) {
    throw std::runtime_error("Driver no iniciado o cerrado");
  }
  if (fault_) {
    throw std::runtime_error(*fault_);
  }
}

void MoveMasterDriver::check_configurable()
{
  check_running();
  if (armed_) {
    throw std::runtime_error("Ejecuta disarm() antes de cambiar el setup");
  }
  // Después de detener el heartbeat, respetar una ventana de silencio antes de
  // escribir parámetros (no confirma el estado físico). Se sigue recibiendo.
  if (last_enable_tx_at_) {
    const auto quiet_until = *last_enable_tx_at_ +
      std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(config_.disable_settle_s));
    while (Clock::now() < quiet_until) {
      if (auto msg = bus_->recv(0.005)) {receive(*msg);}
    }
    check_running();
  }
}

void MoveMasterDriver::initialize()
{
  check_configurable();
  if (initialized_) {
    throw std::runtime_error("initialize() sólo se permite una vez por instancia");
  }

  const auto period = static_cast<double>(config_.status_period_ms);
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    auto & axis = axes_[i];
    const auto & p = axis.protocol;

    std::vector<Step> steps;
    steps.push_back({axis.config.name + ".stop_follower",
        p.frames["STOP_FOLLOWER_MODE"].packet(p.device_id), "STOP_FOLLOWER_MODE_RESPONSE"});
    // Tabla REV SparkParameters compatible con la librería. Todo en RAM.
    for (const auto & [name, pid, kind, value] : {
        std::tuple{"feedback_sensor", 9, "uint", 1.0},       // kPrimaryEncoder = 1
        std::tuple{"position_factor", 112, "float", 1.0},
        std::tuple{"velocity_factor", 113, "float", 1.0},
        std::tuple{"position_wrapping", 149, "bool", 0.0},
        std::tuple{"status0_period_ms", 158, "uint", period},
        std::tuple{"status2_period_ms", 160, "uint", period},
      })
    {
      steps.push_back(write_step(axis, setup_parameter(name, pid, kind), value));
    }
    steps.push_back({axis.config.name + ".enable_status_0_2",
        p.frames["SET_STATUSES_ENABLED"].packet(
          p.device_id, {{"MASK", STATUS_0_AND_2}, {"ENABLED_BITFIELD", STATUS_0_AND_2}}),
        "SET_STATUSES_ENABLED_RESPONSE"});

    for (const auto & step : steps) {
      execute(i, step);
    }
    // Descarta telemetría recibida con las unidades anteriores.
    axis.position_at.reset();
    axis.current_at.reset();
    axis.pv.reset();
    axis.velocity.reset();
    axis.current.reset();
  }
  initialized_ = true;

  for (std::size_t i = 0; i < axes_.size(); ++i) {
    if (!axes_[i].config.pidf.empty()) {set_pidf(i, axes_[i].config.pidf);}
    if (!axes_[i].config.motion_profile.empty()) {
      set_motion_profile(i, axes_[i].config.motion_profile);
    }
  }
  if (config_.persist_parameters) {
    persist_parameters();
  }
}

void MoveMasterDriver::set_pidf(std::size_t axis, const std::map<std::string, double> & gains)
{
  auto & a = axes_.at(axis);
  const auto & group = a.protocol.pidf(config_.slot);
  const auto values = validate_pidf(group, gains);
  check_configurable();
  if (!initialized_) {
    throw std::runtime_error("Espera a que initialize() termine correctamente");
  }
  for (const auto & [key, value] : values) {
    execute(axis, write_step(a, group[key], value));
    a.pidf_keys.insert(key);
  }
}

void MoveMasterDriver::set_motion_profile(
  std::size_t axis, const std::map<std::string, double> & profile)
{
  auto & a = axes_.at(axis);
  const auto & group = a.protocol.maxmotion(config_.slot);
  const auto values = validate_profile(group, profile);
  check_configurable();
  if (!initialized_) {
    throw std::runtime_error("Espera a que initialize() termine correctamente");
  }
  for (const auto & [key, value] : values) {
    execute(axis, write_step(a, group[key], value));
    a.profile_keys.insert(key);
  }
}

void MoveMasterDriver::persist_parameters()
{
  check_configurable();
  if (!initialized_) {
    throw std::runtime_error("Espera a que initialize() termine correctamente");
  }
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    const auto & p = axes_[i].protocol;
    const auto & frame = p.frames["PERSIST_PARAMETERS"];
    const double magic = frame.signal("MAGIC_NUMBER").decoded_min.value();
    // Sólo RESULT_CODE=0 confirma. Un 255 mantiene la espera hasta 2.5 s.
    execute(i, {axes_[i].config.name + ".persist_parameters",
        frame.packet(p.device_id, {{"MAGIC_NUMBER", magic}}),
        "PERSIST_PARAMETERS_RESPONSE", std::nullopt, std::nullopt, 2.5});
  }
  // Tras confirmarse, esperar 0.25 s antes de continuar (sin dejar de recibir).
  const auto until = Clock::now() + std::chrono::milliseconds(250);
  while (Clock::now() < until) {
    if (auto msg = bus_->recv(0.005)) {receive(*msg);}
  }
}

bool MoveMasterDriver::wait_for_feedback(double timeout_s)
{
  check_running();
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(timeout_s));
  while (true) {
    const auto now = Clock::now();
    bool all_fresh = true;
    for (const auto & axis : axes_) {
      all_fresh = all_fresh && feedback_fresh(axis, now);
    }
    if (all_fresh) {return true;}
    if (fault_ || now >= deadline) {return false;}
    if (auto msg = bus_->recv(0.005)) {receive(*msg);}
  }
}

MoveMasterDriver::Step MoveMasterDriver::write_step(
  const Axis & axis, const sparkmax::ParameterDefinition & parameter, double value)
{
  return {axis.config.name + "." + parameter.key,
    axis.protocol.parameter_write_packet(parameter, value),
    "PARAMETER_WRITE_RESPONSE", parameter,
    sparkmax::SparkMAXMotionProtocol::pack_parameter_value(value, parameter.value_type)};
}

void MoveMasterDriver::execute(std::size_t axis, const Step & step)
{
  send(step.packet);
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(step.timeout_s.value_or(config_.response_timeout_s)));

  while (true) {
    const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
    if (remaining <= 0.0) {break;}
    const auto msg = bus_->recv(std::min(0.005, remaining));
    if (!msg) {continue;}
    const auto rx = receive(*msg);
    if (fault_) {throw std::runtime_error(*fault_);}
    if (rx && rx->axis == axis && ack(step, *rx)) {return;}
  }
  fail("Timeout CAN esperando " + step.label + "; setup puede estar parcial");
}

bool MoveMasterDriver::ack(const Step & step, const Received & rx)
{
  if (step.response != rx.name) {
    return false;
  }
  if (step.parameter) {
    const auto response = axes_[rx.axis].protocol.decode_parameter_write_response(rx.data);
    if (response.parameter_id != step.parameter->parameter_id) {
      return false;
    }
    const int expected_type = sparkmax::PARAMETER_TYPE_CODE.at(step.parameter->value_type);
    if (!response.success) {
      fail(step.label + ": PARAMETER_WRITE rechazado, código " +
        std::to_string(response.result_code));
    }
    if (response.parameter_type_code != expected_type || response.raw_value != step.raw_value) {
      fail(step.label + ": tipo/valor confirmado no coincide con la escritura");
    }
    return true;
  }
  if (rx.name == "SET_STATUSES_ENABLED_RESPONSE") {
    if (static_cast<unsigned>(rx.decoded.at("SPECIFIED_MASK")) != STATUS_0_AND_2) {
      return false;
    }
    if (rx.decoded.at("RESULT_CODE") != 0 ||
      (static_cast<unsigned>(rx.decoded.at("ENABLED_BITFIELD")) & STATUS_0_AND_2) !=
      STATUS_0_AND_2)
    {
      fail(step.label + ": no se pudieron habilitar STATUS_0 y STATUS_2");
    }
    return true;
  }
  if (rx.name == "PERSIST_PARAMETERS_RESPONSE") {
    const int result = static_cast<int>(rx.decoded.at("RESULT_CODE"));
    if (result == 255) {
      return false;  // No es éxito: conservar la misma solicitud y su deadline.
    }
    if (result != 0) {
      fail(step.label + ": persistencia rechazada, RESULT_CODE=" + std::to_string(result));
    }
  }
  return true;
}

// -----------------------------------------------------------------------------
// Lazo de control
// -----------------------------------------------------------------------------

std::optional<MoveMasterDriver::Received> MoveMasterDriver::receive(
  const sparkmax::CANPacket & msg)
{
  if (msg.is_error_frame) {
    trip("Trama de error CAN recibida");
    return std::nullopt;
  }
  if (!msg.is_extended_id || msg.is_remote_frame) {
    return std::nullopt;
  }
  const auto it = rx_frames_.find(msg.arbitration_id);
  if (it == rx_frames_.end()) {
    return std::nullopt;
  }
  const auto & [index, name] = it->second;
  auto & axis = axes_[index];

  sparkmax::SignalValues decoded;
  try {
    decoded = axis.protocol.frames[name].decode_payload(msg.data);
    for (const auto & [_, value] : decoded) {
      if (!std::isfinite(value)) {throw std::invalid_argument("Telemetría no finita");}
    }
  } catch (const std::invalid_argument &) {
    ++malformed_;
    return std::nullopt;
  }

  const auto now = Clock::now();
  ++rx_count_;
  if (name == "STATUS_2") {
    axis.pv = decoded.at("PRIMARY_ENCODER_POSITION");
    axis.velocity = decoded.at("PRIMARY_ENCODER_VELOCITY");
    axis.position_at = now;
  } else if (name == "STATUS_0") {
    axis.current = decoded.at("CURRENT");
    axis.current_at = now;
    axis.primary_lock = decoded.at("PRIMARY_HEARTBEAT_LOCK") != 0.0;
  }
  return Received{index, name, msg.data, std::move(decoded)};
}

void MoveMasterDriver::read()
{
  if (!running_ || !bus_) {
    return;
  }
  try {
    // Drena lo pendiente; el límite acota el tiempo si el bus está saturado.
    for (int n = 0; n < 512; ++n) {
      const auto msg = bus_->recv(0.0);
      if (!msg) {break;}
      receive(*msg);
    }
  } catch (const std::exception & e) {
    trip(std::string("Fallo de recepción CAN: ") + e.what());
  }
}

std::vector<double> MoveMasterDriver::arm()
{
  check_running();
  if (armed_) {
    throw std::runtime_error("Los ejes ya están habilitados");
  }
  if (!initialized_) {
    throw std::runtime_error("Inicialización/configuración pendiente");
  }
  const auto now = Clock::now();
  std::vector<double> targets;
  for (const auto & axis : axes_) {
    const auto & name = axis.config.name;
    for (const char * key : {"p", "i", "d", "f"}) {
      if (!axis.pidf_keys.count(key)) {
        throw std::runtime_error(name + ": configura P, I, D y F antes de arm()");
      }
    }
    for (const char * key : {"max_acceleration", "cruise_velocity"}) {
      if (!axis.profile_keys.count(key)) {
        throw std::runtime_error(
                name + ": configura max_acceleration y cruise_velocity antes de arm()");
      }
    }
    if (!feedback_fresh(axis, now)) {
      throw std::runtime_error(name + ": se requieren STATUS_0 y STATUS_2 recientes");
    }
    targets.push_back(check_target(axis, *axis.pv));
  }
  // Mantener la PV medida: nunca revivir un objetivo de una sesión anterior.
  std::vector<double> joint_targets;
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    axes_[i].sp = targets[i];
    joint_targets.push_back(motor_to_joint(i, targets[i]));
  }
  armed_ = true;
  return joint_targets;
}

void MoveMasterDriver::write(const std::vector<double> & position_commands)
{
  if (!armed_) {
    return;  // Desarmado no se envía heartbeat ni SP.
  }
  if (position_commands.size() != axes_.size()) {
    throw std::invalid_argument("Se esperaban " + std::to_string(axes_.size()) + " comandos");
  }

  const auto now = Clock::now();
  for (const auto & axis : axes_) {
    if (!feedback_fresh(axis, now)) {
      fail("Watchdog: STATUS_0 o STATUS_2 sin actualizar en " + axis.config.name);
    }
  }

  // Validar todo antes de transmitir: o se envían todos los SP o ninguno.
  std::vector<double> targets;
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    const double command = position_commands[i];
    targets.push_back(std::isnan(command) ?
      *axes_[i].sp : check_target(axes_[i], joint_to_motor(i, command)));
  }

  // Siempre cargar el SP antes del heartbeat que habilita.
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    axes_[i].sp = targets[i];
    send(axes_[i].protocol.maxmotion_setpoint_packet(targets[i], config_.slot));
  }
  send(hb_on_);
  last_enable_tx_at_ = Clock::now();
}

void MoveMasterDriver::disarm()
{
  // Detiene heartbeat/SP; la deshabilitación depende del watchdog del SPARK.
  // No es una parada inmediata ni un freno de seguridad.
  armed_ = false;
}

void MoveMasterDriver::send(const sparkmax::CANPacket & packet)
{
  try {
    bus_->send(packet, 0.005);
  } catch (const std::exception & e) {
    fail(std::string("Fallo de transmisión CAN: ") + e.what());
  }
  ++tx_count_;
}

void MoveMasterDriver::trip(const std::string & reason)
{
  if (!fault_) {fault_ = reason;}
  armed_ = false;
}

void MoveMasterDriver::fail(const std::string & reason)
{
  trip(reason);
  throw std::runtime_error(*fault_);
}

// -----------------------------------------------------------------------------
// Consulta y conversiones
// -----------------------------------------------------------------------------

bool MoveMasterDriver::fresh(
  const std::optional<Clock::time_point> & at, Clock::time_point now) const
{
  return at && std::chrono::duration<double>(now - *at).count() <= config_.feedback_timeout_s;
}

bool MoveMasterDriver::feedback_fresh(const Axis & axis, Clock::time_point now) const
{
  return fresh(axis.position_at, now) && fresh(axis.current_at, now);
}

double MoveMasterDriver::check_target(const Axis & axis, double rotations) const
{
  const double value = to_float32(rotations, axis.config.name + " setpoint");
  if (axis.limits_rot) {
    const auto [lo, hi] = *axis.limits_rot;
    if (!(lo <= rotations && rotations <= hi && lo <= value && value <= hi)) {
      throw std::invalid_argument(
              axis.config.name + ": SP fuera de los límites [" + std::to_string(lo) + ", " +
              std::to_string(hi) + "] rot");
    }
  }
  return value;
}

double MoveMasterDriver::joint_to_motor(std::size_t axis, double joint_rad) const
{
  const auto & j = axes_.at(axis).config;
  return (joint_rad - j.offset) * j.gear_ratio / TWO_PI;
}

double MoveMasterDriver::motor_to_joint(std::size_t axis, double motor_rot) const
{
  const auto & j = axes_.at(axis).config;
  return j.offset + motor_rot * TWO_PI / j.gear_ratio;
}

double MoveMasterDriver::motor_rpm_to_joint(std::size_t axis, double motor_rpm) const
{
  return motor_rpm * TWO_PI / 60.0 / axes_.at(axis).config.gear_ratio;
}

JointState MoveMasterDriver::state(std::size_t axis) const
{
  const auto & a = axes_.at(axis);
  JointState s;
  if (a.pv) {s.position = motor_to_joint(axis, *a.pv);}
  if (a.velocity) {s.velocity = motor_rpm_to_joint(axis, *a.velocity);}
  if (a.current) {s.current = *a.current;}
  s.fresh = feedback_fresh(a, Clock::now());
  return s;
}

Telemetry MoveMasterDriver::telemetry(std::size_t axis) const
{
  const auto & a = axes_.at(axis);
  const auto now = Clock::now();
  const auto age = [&](const std::optional<Clock::time_point> & at) -> std::optional<double> {
      if (!at) {return std::nullopt;}
      return std::chrono::duration<double>(now - *at).count();
    };
  const bool position_fresh = fresh(a.position_at, now);
  std::optional<double> error;
  if (a.sp && a.pv && position_fresh) {error = *a.sp - *a.pv;}
  return Telemetry{
    a.sp, a.pv, a.velocity, a.current, error, age(a.position_at), age(a.current_at),
    position_fresh, fresh(a.current_at, now), armed_, initialized_, fault_,
    rx_count_, tx_count_, malformed_, a.primary_lock};
}

}  // namespace movemaster
