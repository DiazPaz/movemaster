#include "movemaster_driver/movemaster_driver.hpp"

#include <cmath>
#include <set>
#include <sstream>

namespace movemaster_driver
{

using sparkmax_protocol::CANPacket;
using sparkmax_protocol::CanFilter;
using sparkmax_protocol::ParameterDefinition;
using sparkmax_protocol::PositionProtocol;
using sparkmax_protocol::SparkFrameDatabase;

namespace
{
constexpr std::chrono::microseconds kRxPollTimeout{5000};
constexpr std::chrono::milliseconds kPersistSettle{250};
constexpr double kLimitTolerance = 1e-9;

constexpr const char * kResponseFrames[] = {
  "PARAMETER_WRITE_RESPONSE",
  "SET_STATUSES_ENABLED_RESPONSE",
  "STOP_FOLLOWER_MODE_RESPONSE",
  "PERSIST_PARAMETERS_RESPONSE",
};

void requireFinite(double value, const std::string & what)
{
  if (!std::isfinite(value)) {throw std::invalid_argument(what + " debe ser finito");}
}

double secondsSince(MoveMasterDriver::Clock::time_point now,
  MoveMasterDriver::Clock::time_point then)
{
  return std::chrono::duration<double>(now - then).count();
}
}  // namespace

// -----------------------------------------------------------------------------
// Configuración
// -----------------------------------------------------------------------------

void DriverConfig::validate() const
{
  if (frames_json_path.empty()) {
    throw std::invalid_argument("frames_json_path vacío (ruta de spark-frames-2.1.0)");
  }
  if (can_interface.empty()) {throw std::invalid_argument("can_interface vacío");}
  if (joints.empty()) {throw std::invalid_argument("Se requiere al menos un eje");}
  if (joints.size() > config::kMaxJoints) {
    throw std::invalid_argument("MoveMasterDriver soporta como máximo " +
            std::to_string(config::kMaxJoints) + " SPARK MAX");
  }
  std::set<std::string> names;
  std::set<int> ids;
  for (const auto & joint : joints) {
    const std::string who = "Eje '" + joint.name + "': ";
    if (joint.name.empty()) {throw std::invalid_argument("Todos los ejes requieren nombre");}
    if (!names.insert(joint.name).second) {
      throw std::invalid_argument(who + "nombre duplicado");
    }
    if (joint.can_id < 1 || joint.can_id > 62) {
      throw std::invalid_argument(who + "can_id debe estar entre 1 y 62");
    }
    if (!ids.insert(joint.can_id).second) {
      throw std::invalid_argument(who + "can_id duplicado (" +
              std::to_string(joint.can_id) + ")");
    }
    try {
      joint.transmission.validate();
    } catch (const std::invalid_argument & error) {
      throw std::invalid_argument(who + error.what());
    }
    if (std::isnan(joint.min_position_rad) || std::isnan(joint.max_position_rad) ||
      !(joint.min_position_rad < joint.max_position_rad))
    {
      throw std::invalid_argument(who + "se requiere min_position_rad < max_position_rad");
    }
    if (joint.pid_slot < 0 || joint.pid_slot > 3) {
      throw std::invalid_argument(who + "pid_slot debe estar entre 0 y 3");
    }
    const auto & g = joint.gains;
    for (const auto & [value, label] : {std::pair{g.p, "p"}, std::pair{g.i, "i"},
        std::pair{g.d, "d"}, std::pair{g.f, "f"}})
    {
      requireFinite(value, who + label);
      if (value < 0.0) {throw std::invalid_argument(who + label + " debe ser >= 0");}
    }
    if (g.output_min) {
      requireFinite(*g.output_min, who + "output_min");
      if (*g.output_min < -1.0 || *g.output_min > 0.0) {
        throw std::invalid_argument(who + "output_min debe estar en [-1, 0]");
      }
    }
    if (g.output_max) {
      requireFinite(*g.output_max, who + "output_max");
      if (*g.output_max < 0.0 || *g.output_max > 1.0) {
        throw std::invalid_argument(who + "output_max debe estar en [0, 1]");
      }
    }
    if (g.i_zone) {requireFinite(*g.i_zone, who + "i_zone");}
    if (g.d_filter) {requireFinite(*g.d_filter, who + "d_filter");}
  }
  if (status_period_ms < 1 || status_period_ms > 1000) {
    throw std::invalid_argument("status_period_ms debe estar entre 1 y 1000");
  }
  if (feedback_timeout < std::chrono::milliseconds(3 * status_period_ms)) {
    throw std::invalid_argument("feedback_timeout debe ser >= 3 periodos de STATUS");
  }
  if (response_timeout.count() <= 0 || persist_timeout.count() <= 0) {
    throw std::invalid_argument("Los timeouts de respuesta deben ser positivos");
  }
  if (disable_settle.count() < 0 || tx_timeout.count() < 0 || min_tx_period.count() < 0) {
    throw std::invalid_argument("Los tiempos no pueden ser negativos");
  }
  if (max_consecutive_tx_failures < 1) {
    throw std::invalid_argument("max_consecutive_tx_failures debe ser >= 1");
  }
  if (heartbeat.dlc > 8) {throw std::invalid_argument("heartbeat.dlc debe ser <= 8");}
}

DriverConfig makeDefaultConfig(const std::string & frames_json_path)
{
  DriverConfig cfg;
  cfg.frames_json_path = frames_json_path;
  for (std::size_t i = 0; i < config::kJointCount; ++i) {
    JointConfig joint;
    joint.name = config::kJointNames[i];
    joint.can_id = config::kCanIds[i];
    joint.transmission.gear_ratio = config::kGearRatios[i];
    joint.transmission.inverted = config::kInverted[i];
    joint.transmission.offset_rad = config::kOffsetsRad[i];
    joint.min_position_rad = config::kMinPositionRad[i];
    joint.max_position_rad = config::kMaxPositionRad[i];
    cfg.joints.push_back(joint);
  }
  return cfg;
}

// -----------------------------------------------------------------------------
// Ciclo de vida
// -----------------------------------------------------------------------------

MoveMasterDriver::MoveMasterDriver(
  DriverConfig config, std::unique_ptr<sparkmax_protocol::CanTransport> transport)
: config_(std::move(config)), transport_(std::move(transport))
{
  config_.validate();
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
  if (open_) {return;}
  config_.validate();
  if (!frames_) {
    frames_ = SparkFrameDatabase::loadFile(config_.frames_json_path);
  }

  axes_.clear();
  routes_.clear();
  axes_.reserve(config_.joints.size());
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    Axis axis;
    axis.config = config_.joints[i];
    axis.protocol = std::make_unique<PositionProtocol>(frames_, axis.config.can_id);
    const auto id = axis.config.can_id;
    routes_[frames_->at("STATUS_0").arbitrationId(id)] = {i, RxKind::Status0};
    routes_[frames_->at("STATUS_1").arbitrationId(id)] = {i, RxKind::Status1};
    routes_[frames_->at("STATUS_2").arbitrationId(id)] = {i, RxKind::Status2};
    for (const char * response : kResponseFrames) {
      routes_[frames_->at(response).arbitrationId(id)] = {i, RxKind::Response};
    }
    axes_.push_back(std::move(axis));
  }

  std::vector<CanFilter> filters;
  filters.reserve(routes_.size());
  for (const auto & route : routes_) {
    filters.push_back(CanFilter{route.first, CANPacket::kExtendedIdMask, true});
  }

  heartbeat_ = CANPacket{};
  heartbeat_.arbitration_id = config_.heartbeat.arbitration_id;
  heartbeat_.data = config_.heartbeat.data;
  heartbeat_.dlc = config_.heartbeat.dlc;
  heartbeat_.frame_name = "REFERENCE_HEARTBEAT";

  if (!transport_) {
    transport_ = std::make_unique<sparkmax_protocol::SocketCanTransport>(config_.can_interface);
  }
  transport_->open(filters);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    fault_.clear();
    last_enable_tx_.reset();
    consecutive_tx_failures_ = 0;
  }
  faulted_ = false;
  armed_ = false;
  configured_ = false;
  stop_rx_ = false;
  rx_thread_ = std::thread(&MoveMasterDriver::rxLoop, this);
  open_ = true;
}

void MoveMasterDriver::close()
{
  armed_ = false;
  if (rx_thread_.joinable()) {
    stop_rx_ = true;
    rx_thread_.join();
  }
  if (transport_) {transport_->close();}
  open_ = false;
  configured_ = false;
}

void MoveMasterDriver::requireOpen() const
{
  if (!open_) {throw DriverError("MoveMasterDriver: el bus no está abierto (open())");}
}

void MoveMasterDriver::requireDisarmedAndHealthy(const char * operation) const
{
  requireOpen();
  if (armed_) {throw DriverError(std::string(operation) + ": ejecuta disarm() primero");}
  if (faulted_) {throw DriverError(std::string(operation) + ": fallo enclavado: " + fault());}
}

void MoveMasterDriver::waitDisableSettle()
{
  std::optional<Clock::time_point> last;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last = last_enable_tx_;
  }
  if (last) {
    // Ventana de silencio tras cortar el heartbeat antes de tocar parámetros
    // (no confirma el estado físico del motor).
    std::this_thread::sleep_until(*last + config_.disable_settle);
  }
}

// -----------------------------------------------------------------------------
// Recepción
// -----------------------------------------------------------------------------

void MoveMasterDriver::rxLoop()
{
  while (!stop_rx_) {
    CANPacket packet;
    bool received = false;
    try {
      received = transport_->receive(packet, kRxPollTimeout);
    } catch (const std::exception & error) {
      trip(std::string("Fallo del transporte CAN: ") + error.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    if (received) {handleFrame(packet);}
  }
}

void MoveMasterDriver::handleFrame(const CANPacket & packet)
{
  if (packet.is_error_frame) {
    ++error_frames_;
    if (config_.trip_on_error_frame) {trip("Trama de error CAN recibida");}
    return;
  }
  if (!packet.is_extended_id || packet.is_remote_frame) {return;}
  const auto route = routes_.find(packet.arbitration_id);
  if (route == routes_.end()) {return;}

  Axis & axis = axes_[route->second.axis];
  const auto now = Clock::now();
  try {
    switch (route->second.kind) {
      case RxKind::Status0: {
          const auto status = axis.protocol->decodeStatus0(packet);
          std::lock_guard<std::mutex> lock(state_mutex_);
          axis.status0 = status;
          axis.status0_at = now;
          break;
        }
      case RxKind::Status1: {
          const auto status = axis.protocol->decodeStatus1(packet);
          std::lock_guard<std::mutex> lock(state_mutex_);
          axis.status1 = status;
          axis.status1_at = now;
          break;
        }
      case RxKind::Status2: {
          const auto status = axis.protocol->decodeStatus2(packet);
          if (!std::isfinite(status.position) || !std::isfinite(status.velocity)) {
            ++malformed_frames_;
            return;
          }
          std::lock_guard<std::mutex> lock(state_mutex_);
          axis.status2 = status;
          axis.status2_at = now;
          break;
        }
      case RxKind::Response: {
          std::lock_guard<std::mutex> lock(pending_mutex_);
          if (pending_active_ && !pending_done_ && pending_accept_ != nullptr &&
            packet.arbitration_id == pending_arbitration_id_ && (*pending_accept_)(packet))
          {
            pending_response_ = packet;
            pending_done_ = true;
            pending_cv_.notify_all();
          }
          break;
        }
    }
    ++rx_frames_;
  } catch (const std::exception &) {
    ++malformed_frames_;
  }
}

void MoveMasterDriver::trip(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (fault_.empty()) {fault_ = reason;}
  }
  armed_ = false;
  faulted_ = true;
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_cv_.notify_all();
}

// -----------------------------------------------------------------------------
// Transacciones de configuración
// -----------------------------------------------------------------------------

bool MoveMasterDriver::send(const CANPacket & packet)
{
  try {
    if (transport_->send(packet, config_.tx_timeout)) {
      ++tx_frames_;
      return true;
    }
    ++tx_failures_;
    return false;
  } catch (const std::exception & error) {
    ++tx_failures_;
    trip(std::string("Error de transmisión CAN: ") + error.what());
    return false;
  }
}

CANPacket MoveMasterDriver::transact(
  const CANPacket & request, const char * response_frame, std::size_t axis,
  const Accept & accept, std::chrono::milliseconds timeout, const std::string & label)
{
  std::lock_guard<std::mutex> transaction(transaction_mutex_);
  if (faulted_) {throw DriverError(fault());}
  const uint32_t expected = frames_->at(response_frame).arbitrationId(axes_[axis].config.can_id);

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_active_ = true;
    pending_done_ = false;
    pending_arbitration_id_ = expected;
    pending_accept_ = &accept;
  }
  auto clear = [this]() {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_active_ = false;
      pending_accept_ = nullptr;
    };

  if (!send(request)) {
    clear();
    trip(label + ": no se pudo transmitir la solicitud");
    throw DriverError(fault());
  }

  bool done = false;
  CANPacket response;
  {
    std::unique_lock<std::mutex> lock(pending_mutex_);
    pending_cv_.wait_for(lock, timeout, [this]() {return pending_done_ || faulted_.load();});
    done = pending_done_;
    response = pending_response_;
    pending_active_ = false;
    pending_accept_ = nullptr;
  }
  if (done) {return response;}
  if (!faulted_) {
    trip("Timeout CAN esperando " + label + "; la configuración puede estar parcial");
  }
  throw DriverError(fault());
}

void MoveMasterDriver::writeParameter(
  std::size_t axis, const ParameterDefinition & parameter, double value, const std::string & label)
{
  const auto & protocol = *axes_[axis].protocol;
  uint32_t raw = 0;
  try {
    raw = sparkmax_protocol::packParameterValue(value, parameter.value_type);
  } catch (const std::exception & error) {
    throw DriverError(label + ": " + error.what());
  }
  const Accept accept = [&protocol, &parameter](const CANPacket & packet) {
      return protocol.decodeParameterWriteResponse(packet).parameter_id == parameter.parameter_id;
    };
  const CANPacket packet = transact(
    protocol.parameterWriteRawPacket(parameter.parameter_id, raw), "PARAMETER_WRITE_RESPONSE",
    axis, accept, config_.response_timeout, label);

  const auto response = protocol.decodeParameterWriteResponse(packet);
  if (!response.success) {
    std::ostringstream out;
    out << label << ": PARAMETER_WRITE rechazado, código " <<
      static_cast<int>(response.result_code) << " (" <<
      sparkmax_protocol::parameterResultToString(response.result_code) << ")";
    trip(out.str());
    throw DriverError(fault());
  }
  if (response.parameter_type_code != static_cast<uint8_t>(parameter.value_type) ||
    response.raw_value != raw)
  {
    trip(label + ": tipo/valor confirmado no coincide con la escritura");
    throw DriverError(fault());
  }
}

void MoveMasterDriver::configureAxis(std::size_t index)
{
  Axis & axis = axes_[index];
  const PositionProtocol & protocol = *axis.protocol;
  const std::string who = axis.config.name + " (ID " + std::to_string(axis.config.can_id) + ")";

  // 1. Salir del modo seguidor para aceptar setpoints dirigidos a este SPARK.
  const Accept any = [](const CANPacket &) {return true;};
  transact(protocol.stopFollowerModePacket(), "STOP_FOLLOWER_MODE_RESPONSE", index, any,
    config_.response_timeout, who + ": stop_follower");

  // 2..5. Encoder primario, factores = 1 (rotaciones / RPM del motor), sin wrapping,
  //       periodos de STATUS_0 y STATUS_2.
  const auto & setup = protocol.group("setup");
  writeParameter(index, setup["feedback_sensor"], 1, who + ": feedback_sensor");
  writeParameter(index, setup["position_factor"], 1.0, who + ": position_factor");
  writeParameter(index, setup["velocity_factor"], 1.0, who + ": velocity_factor");
  writeParameter(index, setup["position_wrapping"], 0, who + ": position_wrapping");
  writeParameter(index, setup["status0_period_ms"], config_.status_period_ms,
    who + ": status0_period_ms");
  writeParameter(index, setup["status2_period_ms"], config_.status_period_ms,
    who + ": status2_period_ms");

  // 6. Habilitar STATUS_0, STATUS_2 (y STATUS_1) preservando los demás bits.
  const uint16_t mask = config_.enable_status1 ? 0b111 : 0b101;
  const Accept status_accept = [&protocol, mask](const CANPacket & packet) {
      return protocol.decodeSetStatusesEnabledResponse(packet).specified_mask == mask;
    };
  const auto status_packet = transact(protocol.setStatusesEnabledPacket(mask, mask),
      "SET_STATUSES_ENABLED_RESPONSE", index, status_accept, config_.response_timeout,
      who + ": set_statuses_enabled");
  const auto status = protocol.decodeSetStatusesEnabledResponse(status_packet);
  if (!status.success || (status.enabled_bitfield & mask) != mask) {
    trip(who + ": no se pudieron habilitar las tramas STATUS");
    throw DriverError(fault());
  }

  // 7. Ganancias del lazo de posición.
  applyGains(index, axis.config.gains);

  // Descartar telemetría previa a la configuración.
  std::lock_guard<std::mutex> lock(state_mutex_);
  axis.status0_at.reset();
  axis.status1_at.reset();
  axis.status2_at.reset();
  axis.has_reference = false;
}

void MoveMasterDriver::applyGains(std::size_t index, const PidfGains & gains)
{
  Axis & axis = axes_[index];
  const auto & pidf = axis.protocol->pidf(axis.config.pid_slot);
  const std::string who = axis.config.name + " (ID " + std::to_string(axis.config.can_id) +
    ") slot " + std::to_string(axis.config.pid_slot);
  writeParameter(index, pidf["p"], gains.p, who + ": P");
  writeParameter(index, pidf["i"], gains.i, who + ": I");
  writeParameter(index, pidf["d"], gains.d, who + ": D");
  writeParameter(index, pidf["f"], gains.f, who + ": F");
  if (gains.i_zone) {writeParameter(index, pidf["i_zone"], *gains.i_zone, who + ": i_zone");}
  if (gains.d_filter) {
    writeParameter(index, pidf["d_filter"], *gains.d_filter, who + ": d_filter");
  }
  if (gains.output_min) {
    writeParameter(index, pidf["output_min"], *gains.output_min, who + ": output_min");
  }
  if (gains.output_max) {
    writeParameter(index, pidf["output_max"], *gains.output_max, who + ": output_max");
  }
}

void MoveMasterDriver::configure()
{
  requireDisarmedAndHealthy("configure");
  waitDisableSettle();
  configured_ = false;
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    configureAxis(i);
  }
  if (config_.persist_parameters) {persistParameters();}
  configured_ = true;
}

void MoveMasterDriver::persistParameters()
{
  requireDisarmedAndHealthy("persistParameters");
  waitDisableSettle();
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    const auto & protocol = *axes_[i].protocol;
    const std::string who = axes_[i].config.name + " (ID " +
      std::to_string(axes_[i].config.can_id) + ")";
    // RESULT_CODE = 255 no es éxito: se sigue esperando la misma solicitud.
    const Accept accept = [&protocol](const CANPacket & packet) {
        return protocol.decodePersistParametersResponse(packet) != 255;
      };
    const auto packet = transact(protocol.persistParametersPacket(),
        "PERSIST_PARAMETERS_RESPONSE", i, accept, config_.persist_timeout,
        who + ": persist_parameters");
    const uint8_t result = protocol.decodePersistParametersResponse(packet);
    if (result != 0) {
      trip(who + ": persistencia rechazada, RESULT_CODE=" + std::to_string(result));
      throw DriverError(fault());
    }
    std::this_thread::sleep_for(kPersistSettle);
  }
}

void MoveMasterDriver::writeGains(std::size_t joint, const PidfGains & gains)
{
  requireDisarmedAndHealthy("writeGains");
  if (joint >= axes_.size()) {throw std::out_of_range("Índice de eje inválido");}
  JointConfig candidate = axes_[joint].config;
  candidate.gains = gains;
  DriverConfig check = config_;
  check.joints[joint] = candidate;
  check.validate();
  waitDisableSettle();
  applyGains(joint, gains);
  axes_[joint].config.gains = gains;
  config_.joints[joint].gains = gains;
}

// -----------------------------------------------------------------------------
// Armado y lazo de control
// -----------------------------------------------------------------------------

bool MoveMasterDriver::feedbackFreshLocked(Clock::time_point now) const
{
  for (const auto & axis : axes_) {
    if (!axis.status0_at || !axis.status2_at) {return false;}
    if (now - *axis.status0_at > config_.feedback_timeout ||
      now - *axis.status2_at > config_.feedback_timeout)
    {
      return false;
    }
  }
  return !axes_.empty();
}

bool MoveMasterDriver::feedbackFresh() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return feedbackFreshLocked(Clock::now());
}

bool MoveMasterDriver::waitForFeedback(std::chrono::milliseconds timeout)
{
  requireOpen();
  const auto deadline = Clock::now() + timeout;
  while (true) {
    if (faulted_) {return false;}
    if (feedbackFresh()) {return true;}
    if (Clock::now() >= deadline) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void MoveMasterDriver::arm()
{
  requireOpen();
  if (faulted_) {throw DriverError("arm(): fallo enclavado: " + fault());}
  if (armed_) {return;}
  if (!configured_) {throw DriverError("arm(): ejecuta configure() primero");}

  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now = Clock::now();
  if (!feedbackFreshLocked(now)) {
    throw DriverError("arm(): se requieren STATUS_0 y STATUS_2 recientes de todos los ejes");
  }
  for (const auto & axis : axes_) {
    const double q = axis.config.transmission.motorRotationsToJoint(axis.status2.position);
    if (q < axis.config.min_position_rad - kLimitTolerance ||
      q > axis.config.max_position_rad + kLimitTolerance)
    {
      std::ostringstream out;
      out << "arm(): " << axis.config.name << " está fuera de sus límites (" << q << " rad en [" <<
        axis.config.min_position_rad << ", " << axis.config.max_position_rad << "])";
      throw DriverError(out.str());
    }
  }
  // Se mantiene la posición medida: armar nunca manda el eje a cero.
  for (auto & axis : axes_) {
    axis.reference_motor_rot = axis.status2.position;
    axis.has_reference = true;
  }
  consecutive_tx_failures_ = 0;
  next_tx_ = now;
  armed_ = true;
}

void MoveMasterDriver::disarm()
{
  // Se deja de enviar heartbeat/SP; el SPARK se deshabilita por su watchdog.
  armed_ = false;
}

bool MoveMasterDriver::setReference(std::size_t joint, double position_rad)
{
  if (joint >= axes_.size() || !std::isfinite(position_rad)) {return false;}
  Axis & axis = axes_[joint];
  bool exact = true;
  double target = position_rad;
  if (target < axis.config.min_position_rad) {
    target = axis.config.min_position_rad;
    exact = false;
  } else if (target > axis.config.max_position_rad) {
    target = axis.config.max_position_rad;
    exact = false;
  }
  const double motor = axis.config.transmission.jointToMotorRotations(target);
  if (!std::isfinite(motor) || std::fabs(motor) > std::numeric_limits<float>::max()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  axis.reference_motor_rot = motor;
  axis.has_reference = true;
  return exact;
}

bool MoveMasterDriver::sendReferences()
{
  if (faulted_) {return false;}
  if (!armed_) {return true;}

  const auto now = Clock::now();
  std::array<double, config::kMaxJoints> references{};
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (config_.min_tx_period.count() > 0) {
      if (now < next_tx_) {return true;}
      next_tx_ = (now - next_tx_ > config_.min_tx_period) ? now + config_.min_tx_period :
        next_tx_ + config_.min_tx_period;
    }
    fresh = feedbackFreshLocked(now);
    for (std::size_t i = 0; i < axes_.size(); ++i) {
      references[i] = axes_[i].reference_motor_rot;
    }
  }
  if (!fresh) {
    trip("Watchdog: STATUS_0 o STATUS_2 sin actualizar en " +
      std::to_string(config_.feedback_timeout.count()) + " ms");
    return false;
  }

  bool all_sent = true;
  for (std::size_t i = 0; i < axes_.size(); ++i) {
    CANPacket packet;
    try {
      packet = axes_[i].protocol->positionSetpointPacket(references[i], axes_[i].config.pid_slot);
    } catch (const std::exception & error) {
      trip(axes_[i].config.name + ": setpoint inválido: " + error.what());
      return false;
    }
    // Siempre se carga el SP antes del heartbeat, para no revivir un objetivo viejo.
    if (!send(packet)) {all_sent = false;}
  }
  if (faulted_ || !armed_) {return !faulted_;}
  if (!send(heartbeat_)) {all_sent = false;}

  bool too_many_failures = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_enable_tx_ = Clock::now();
    consecutive_tx_failures_ = all_sent ? 0 : consecutive_tx_failures_ + 1;
    too_many_failures = consecutive_tx_failures_ >= config_.max_consecutive_tx_failures;
  }
  if (too_many_failures) {
    trip("Buffer de TX CAN lleno de forma persistente (¿bus sin nodos o bus-off?)");
  }
  return !faulted_;
}

bool MoveMasterDriver::checkWatchdog()
{
  if (armed_ && !feedbackFresh()) {
    trip("Watchdog: STATUS_0 o STATUS_2 sin actualizar en " +
      std::to_string(config_.feedback_timeout.count()) + " ms");
  }
  return !faulted_;
}

JointState MoveMasterDriver::state(std::size_t joint) const
{
  if (joint >= axes_.size()) {throw std::out_of_range("Índice de eje inválido");}
  const Axis & axis = axes_[joint];
  const auto & tx = axis.config.transmission;
  const auto timeout = std::chrono::duration<double>(config_.feedback_timeout).count();
  JointState s;
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now = Clock::now();
  if (axis.status2_at) {
    s.motor_position_rot = axis.status2.position;
    s.motor_velocity_rpm = axis.status2.velocity;
    s.position_rad = tx.motorRotationsToJoint(axis.status2.position);
    s.velocity_rad_s = tx.motorRpmToJointVelocity(axis.status2.velocity);
    s.position_age_s = secondsSince(now, *axis.status2_at);
    s.position_fresh = s.position_age_s <= timeout;
  }
  if (axis.status0_at) {
    s.current_a = axis.status0.current;
    s.applied_output = axis.status0.applied_output;
    s.bus_voltage_v = axis.status0.voltage;
    s.motor_temperature_c = axis.status0.motor_temperature;
    s.primary_heartbeat_lock = axis.status0.primary_heartbeat_lock;
    s.hard_limit_forward = axis.status0.hard_forward_limit;
    s.hard_limit_reverse = axis.status0.hard_reverse_limit;
    s.current_age_s = secondsSince(now, *axis.status0_at);
    s.current_fresh = s.current_age_s <= timeout;
  }
  if (axis.status1_at) {
    s.active_faults = axis.status1.active_faults;
    s.sticky_faults = axis.status1.sticky_faults;
    s.active_warnings = axis.status1.active_warnings;
  }
  if (axis.has_reference) {
    s.reference_rad = tx.motorRotationsToJoint(axis.reference_motor_rot);
  }
  return s;
}

void MoveMasterDriver::readStates(JointState * out, std::size_t count) const
{
  const std::size_t n = count < axes_.size() ? count : axes_.size();
  for (std::size_t i = 0; i < n; ++i) {out[i] = state(i);}
}

// -----------------------------------------------------------------------------
// Utilidades
// -----------------------------------------------------------------------------

void MoveMasterDriver::setJointPosition(std::size_t joint, double joint_rad)
{
  requireDisarmedAndHealthy("setJointPosition");
  if (joint >= axes_.size()) {throw std::out_of_range("Índice de eje inválido");}
  requireFinite(joint_rad, "joint_rad");
  const double motor = axes_[joint].config.transmission.jointToMotorRotations(joint_rad);
  if (!send(axes_[joint].protocol->setPrimaryEncoderPositionPacket(motor))) {
    throw DriverError("setJointPosition: no se pudo transmitir");
  }
  // La trama no tiene respuesta en el JSON: se exige telemetría nueva.
  std::lock_guard<std::mutex> lock(state_mutex_);
  axes_[joint].status2_at.reset();
}

void MoveMasterDriver::clearSparkFaults()
{
  requireOpen();
  for (const auto & axis : axes_) {
    if (!send(axis.protocol->clearFaultsPacket())) {
      throw DriverError("clearSparkFaults: no se pudo transmitir");
    }
  }
}

std::string MoveMasterDriver::fault() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return fault_;
}

DriverStatistics MoveMasterDriver::statistics() const
{
  DriverStatistics s;
  s.rx_frames = rx_frames_.load();
  s.tx_frames = tx_frames_.load();
  s.malformed_frames = malformed_frames_.load();
  s.tx_failures = tx_failures_.load();
  s.error_frames = error_frames_.load();
  return s;
}

const JointConfig & MoveMasterDriver::jointConfig(std::size_t joint) const
{
  if (joint >= config_.joints.size()) {throw std::out_of_range("Índice de eje inválido");}
  return config_.joints[joint];
}

const PositionProtocol & MoveMasterDriver::protocol(std::size_t joint) const
{
  requireOpen();
  if (joint >= axes_.size()) {throw std::out_of_range("Índice de eje inválido");}
  return *axes_[joint].protocol;
}

}  // namespace movemaster_driver
