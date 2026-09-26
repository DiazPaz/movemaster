// movemaster_driver.hpp
//
// MoveMasterDriver: N ejes (3 SPARK MAX en el Movemaster) sobre un único bus
// SocketCAN. Es la versión multi-eje de TeachPendantBackend, sin hilo propio:
//
//   * read()  -> drena la recepción CAN (STATUS_0 / STATUS_2) sin bloquear.
//   * write() -> envía MAXMOTION_POSITION_SETPOINT de cada eje + 1 heartbeat.
//
// El llamador (MovemasterHardware / controller_manager) marca el período. Si
// ese lazo se detiene, se detiene también el heartbeat y el watchdog del SPARK
// deshabilita los motores: el mismo criterio de disarm() del backend Python.
//
// El PID y el perfil MAXMotion se ejecutan dentro de cada SPARK MAX.
// Secuencia: open() -> initialize() -> wait_for_feedback() -> arm() ->
//            [read() / write()]* -> disarm() -> close().
// Un fallo queda enclavado (fault()); hay que crear otra instancia.
// No es thread-safe: un único hilo debe llamar a todos los métodos.

#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "sparkmax_protocol/can_bus.hpp"
#include "sparkmax_protocol/sparkmax_motion_protocol.hpp"

namespace movemaster
{

// Valores exactos del heartbeat validado en la Raspberry Pi 5. No está en el
// JSON SPARK y es global: habilita a todos los SPARK del bus a la vez.
constexpr std::uint32_t HEARTBEAT_ID = 0x01011840;

struct JointConfig
{
  std::string name;
  int device_id{1};
  /// Rotaciones del motor por rotación de la articulación (signo = sentido).
  double gear_ratio{1.0};
  /// Ángulo de la articulación [rad] cuando el encoder del motor marca 0 rot.
  double offset{0.0};
  /// Límites de la articulación [rad] para validar objetivos (opcional).
  std::optional<std::pair<double, double>> position_limits{};
  /// {"p","i","d","f"} del slot configurado.
  std::map<std::string, double> pidf{};
  /// {"max_acceleration" [RPM/s], "cruise_velocity" [RPM],
  ///  "allowed_profile_error" [rot]} del slot configurado.
  std::map<std::string, double> motion_profile{};
};

struct DriverConfig
{
  std::string spec_path;
  std::string channel{"can0"};
  int slot{0};
  int status_period_ms{20};
  double feedback_timeout_s{0.300};
  double response_timeout_s{0.500};
  double disable_settle_s{0.500};
  bool persist_parameters{false};
  std::vector<JointConfig> joints{};
  sparkmax::ParameterLayout parameter_layout{sparkmax::DEFAULT_PARAMETER_LAYOUT};
};

/// Estado de una articulación en unidades SI (lo que consume ros2_control).
struct JointState
{
  double position{std::numeric_limits<double>::quiet_NaN()};  // rad
  double velocity{std::numeric_limits<double>::quiet_NaN()};  // rad/s
  double current{std::numeric_limits<double>::quiet_NaN()};   // A
  bool fresh{false};
};

/// Instantánea por eje en unidades del motor (equivale a Telemetry).
struct Telemetry
{
  std::optional<double> sp_rot;
  std::optional<double> pv_rot;
  std::optional<double> velocity_rpm;
  std::optional<double> current_a;
  std::optional<double> error_rot;
  std::optional<double> position_age_s;
  std::optional<double> current_age_s;
  bool position_fresh;
  bool current_fresh;
  bool armed;
  bool initialized;
  std::optional<std::string> fault;
  std::uint64_t rx_frames;
  std::uint64_t tx_frames;
  std::uint64_t malformed_frames;
  bool primary_heartbeat_lock;
};

class MoveMasterDriver
{
public:
  /// `bus` permite inyectar un bus (p. ej. simulado); si es nulo, open() crea
  /// un SocketCanBus propio sobre config.channel.
  explicit MoveMasterDriver(DriverConfig config, std::shared_ptr<sparkmax::CanBus> bus = nullptr);
  ~MoveMasterDriver();

  MoveMasterDriver(const MoveMasterDriver &) = delete;
  MoveMasterDriver & operator=(const MoveMasterDriver &) = delete;

  // ---- ciclo de vida ----------------------------------------------------------
  void open();
  void close();

  /// Configura cada SPARK (encoder, unidades, STATUS_0/2, PIDF y perfil
  /// MAXMotion) esperando y verificando cada respuesta. Bloqueante, desarmado.
  void initialize();
  void set_pidf(std::size_t axis, const std::map<std::string, double> & gains);
  void set_motion_profile(std::size_t axis, const std::map<std::string, double> & profile);
  void persist_parameters();

  /// Espera STATUS_0 y STATUS_2 recientes de todos los ejes.
  bool wait_for_feedback(double timeout_s);

  // ---- lazo de control ----------------------------------------------------------
  void read();
  /// Habilita manteniendo la PV medida de cada eje. Devuelve los SP [rad].
  std::vector<double> arm();
  /// Comandos de posición [rad] por eje; NaN conserva el SP anterior.
  void write(const std::vector<double> & position_commands);
  void disarm();

  // ---- consulta -------------------------------------------------------------------
  std::size_t size() const {return axes_.size();}
  const JointConfig & joint(std::size_t axis) const {return axes_.at(axis).config;}
  JointState state(std::size_t axis) const;
  Telemetry telemetry(std::size_t axis) const;
  bool armed() const {return armed_;}
  bool initialized() const {return initialized_;}
  const std::optional<std::string> & fault() const {return fault_;}

  // ---- conversiones rad <-> rotaciones ---------------------------------------
  double joint_to_motor(std::size_t axis, double joint_rad) const;
  double motor_to_joint(std::size_t axis, double motor_rot) const;
  double motor_rpm_to_joint(std::size_t axis, double motor_rpm) const;

private:
  using Clock = std::chrono::steady_clock;

  struct Axis
  {
    Axis(JointConfig c, sparkmax::SparkMAXMotionProtocol p)
    : config(std::move(c)), protocol(std::move(p)) {}

    JointConfig config;
    sparkmax::SparkMAXMotionProtocol protocol;
    std::optional<std::pair<double, double>> limits_rot{};
    std::optional<double> sp, pv, velocity, current;
    std::optional<Clock::time_point> position_at, current_at;
    bool primary_lock{false};
    std::set<std::string> pidf_keys, profile_keys;
  };

  struct Step
  {
    std::string label;
    sparkmax::CANPacket packet;
    std::string response;
    std::optional<sparkmax::ParameterDefinition> parameter{};
    std::optional<std::uint32_t> raw_value{};
    std::optional<double> timeout_s{};
  };

  struct Received
  {
    std::size_t axis;
    std::string name;
    sparkmax::Bytes data;
    sparkmax::SignalValues decoded;
  };

  void check_running() const;
  void check_configurable();
  void trip(const std::string & reason);
  [[noreturn]] void fail(const std::string & reason);

  Step write_step(const Axis & axis, const sparkmax::ParameterDefinition & parameter, double value);
  void execute(std::size_t axis, const Step & step);
  bool ack(const Step & step, const Received & rx);
  std::optional<Received> receive(const sparkmax::CANPacket & msg);
  void send(const sparkmax::CANPacket & packet);

  bool fresh(const std::optional<Clock::time_point> & at, Clock::time_point now) const;
  bool feedback_fresh(const Axis & axis, Clock::time_point now) const;
  double check_target(const Axis & axis, double rotations) const;

  DriverConfig config_;
  std::vector<Axis> axes_;
  std::shared_ptr<sparkmax::CanBus> bus_;
  bool owns_bus_;
  std::map<std::uint32_t, std::pair<std::size_t, std::string>> rx_frames_;
  sparkmax::CANPacket hb_on_;

  bool running_{false};
  bool initialized_{false};
  bool armed_{false};
  std::optional<std::string> fault_{};
  std::optional<Clock::time_point> last_enable_tx_at_{};
  std::uint64_t rx_count_{0}, tx_count_{0}, malformed_{0};
};

}  // namespace movemaster
