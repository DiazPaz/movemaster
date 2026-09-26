// movemaster_driver.hpp
//
// MoveMasterDriver: maneja N SPARK MAX (hasta 6; configuración actual: 3) en
// Position Control sobre un único bus SocketCAN.
//
//   * manejo de los SPARK MAX: inicialización/configuración con confirmación de
//     cada PARAMETER_WRITE, un PositionProtocol por controlador
//   * conversiones rad ↔ rotaciones del motor con reductor (GearTransmission)
//   * lectura de estado: hilo RX que decodifica STATUS_0/1/2
//   * envío de referencias: POSITION_SETPOINT de cada eje + heartbeat
//
// Ciclo de vida (lo sigue MovemasterHardware):
//   open() → configure() → waitForFeedback() → arm() →
//   [setReference()/sendReferences() + state() en cada ciclo] → disarm() → close()
//
// Hilos: el hilo RX interno es el único que llama receive(). configure() es
// bloqueante (no usar en el lazo RT). sendReferences()/setReference()/readStates()
// no asignan memoria y están pensados para read()/write() de ros2_control.
//
// Seguridad (igual criterio que teach_pendant_backend.py):
//   * El heartbeat (0x01011840, 8 × 0xFF) sólo se envía mientras está armado.
//     disarm() deja de enviarlo y la deshabilitación depende del watchdog del
//     firmware del SPARK: NO es un paro de emergencia.
//   * Si STATUS_0 o STATUS_2 de cualquier eje deja de llegar durante
//     feedback_timeout estando armado, el driver se desarma y enclava el fallo.
//   * Un fallo queda enclavado hasta close() + open().
//   * El heartbeat es global para el bus: debe haber UN solo emisor (no ejecutar
//     a la vez movemaster_ros / teach_pendant_backend.py sobre el mismo can0).

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "movemaster_driver/gear_transmission.hpp"
#include "movemaster_driver/movemaster_config.hpp"
#include "sparkmax_protocol/position_protocol.hpp"
#include "sparkmax_protocol/socketcan.hpp"

namespace movemaster_driver
{

class DriverError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Ganancias del lazo de posición del SPARK (unidades del SPARK: rotaciones del motor).
struct PidfGains
{
  double p{config::kDefaultP};
  double i{config::kDefaultI};
  double d{config::kDefaultD};
  double f{config::kDefaultF};
  // Opcionales: sólo se escriben si tienen valor (IDs a verificar con SparkParameters).
  std::optional<double> i_zone;
  std::optional<double> d_filter;
  std::optional<double> output_min;   // duty cycle −1..0
  std::optional<double> output_max;   // duty cycle 0..1
};

struct JointConfig
{
  std::string name;
  uint8_t can_id{1};
  GearTransmission transmission;
  double min_position_rad{-std::numeric_limits<double>::infinity()};
  double max_position_rad{std::numeric_limits<double>::infinity()};
  int pid_slot{config::kPidSlot};
  PidfGains gains;
};

/// Heartbeat del ejemplo validado en Raspberry Pi 5 (no está en el JSON de SPARK).
struct HeartbeatConfig
{
  uint32_t arbitration_id{0x01011840u};
  std::array<uint8_t, 8> data{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t dlc{8};
};

struct DriverConfig
{
  std::string can_interface{config::kCanInterface};
  /// Ruta de spark-frames-2.1.0 (share/sparkmax_protocol/spec/ al instalar).
  std::string frames_json_path;
  std::vector<JointConfig> joints;

  int status_period_ms{config::kStatusPeriodMs};
  /// Habilita también STATUS_1 (fallas/advertencias) con su periodo por defecto.
  bool enable_status1{true};
  std::chrono::milliseconds feedback_timeout{300};
  std::chrono::milliseconds response_timeout{500};
  std::chrono::milliseconds persist_timeout{2500};
  /// Silencio tras dejar de enviar heartbeat antes de escribir parámetros.
  std::chrono::milliseconds disable_settle{500};
  std::chrono::microseconds tx_timeout{5000};
  /// Periodo mínimo entre envíos de referencias+heartbeat (0 = en cada llamada).
  std::chrono::microseconds min_tx_period{0};
  /// Envíos fallidos consecutivos (buffer TX lleno) que enclavan un fallo.
  int max_consecutive_tx_failures{10};
  /// Guarda los parámetros en flash al final de configure() (opcional).
  bool persist_parameters{false};
  /// Enclava un fallo al recibir una trama de error CAN (como el backend Python).
  bool trip_on_error_frame{true};
  HeartbeatConfig heartbeat;

  /// Lanza std::invalid_argument si la configuración es incoherente.
  void validate() const;
};

/// Configuración por defecto: 3 SPARK MAX con los valores de movemaster_config.hpp.
DriverConfig makeDefaultConfig(const std::string & frames_json_path);

/// Estado de una articulación (instantánea).
struct JointState
{
  // Articulación (tras el reductor)
  double position_rad{std::numeric_limits<double>::quiet_NaN()};
  double velocity_rad_s{std::numeric_limits<double>::quiet_NaN()};
  double reference_rad{std::numeric_limits<double>::quiet_NaN()};
  // Motor (lo que reporta el SPARK)
  double motor_position_rot{std::numeric_limits<double>::quiet_NaN()};
  double motor_velocity_rpm{std::numeric_limits<double>::quiet_NaN()};
  double current_a{std::numeric_limits<double>::quiet_NaN()};
  double applied_output{std::numeric_limits<double>::quiet_NaN()};
  double bus_voltage_v{std::numeric_limits<double>::quiet_NaN()};
  double motor_temperature_c{std::numeric_limits<double>::quiet_NaN()};
  // Frescura / diagnóstico
  bool position_fresh{false};
  bool current_fresh{false};
  double position_age_s{std::numeric_limits<double>::infinity()};
  double current_age_s{std::numeric_limits<double>::infinity()};
  bool primary_heartbeat_lock{false};
  uint8_t active_faults{0};
  uint8_t sticky_faults{0};
  uint8_t active_warnings{0};
  bool hard_limit_forward{false};
  bool hard_limit_reverse{false};
};

struct DriverStatistics
{
  uint64_t rx_frames{0};
  uint64_t tx_frames{0};
  uint64_t malformed_frames{0};
  uint64_t tx_failures{0};
  uint64_t error_frames{0};
};

class MoveMasterDriver
{
public:
  using Clock = std::chrono::steady_clock;

  /// @param transport  bus a usar; nullptr → SocketCanTransport(config.can_interface).
  explicit MoveMasterDriver(DriverConfig config,
    std::unique_ptr<sparkmax_protocol::CanTransport> transport = nullptr);
  ~MoveMasterDriver();

  MoveMasterDriver(const MoveMasterDriver &) = delete;
  MoveMasterDriver & operator=(const MoveMasterDriver &) = delete;

  // ---- ciclo de vida (bloqueante, fuera del lazo RT) ---------------------------
  /// Carga el JSON, crea un PositionProtocol por eje, abre el bus y lanza el hilo RX.
  void open();
  /// Configura cada SPARK en orden y confirma cada paso (ver configureAxis()).
  void configure();
  /// Espera STATUS_0 y STATUS_2 recientes de todos los ejes.
  bool waitForFeedback(std::chrono::milliseconds timeout);
  /// Habilita: la referencia de cada eje = su posición medida actual.
  void arm();
  /// Deja de enviar referencias y heartbeat.
  void disarm();
  /// Desarma, detiene el hilo RX y cierra el bus.
  void close();

  // ---- lazo de control (sin asignaciones) ---------------------------------------
  /// Fija la referencia de un eje en rad de la articulación.
  /// Devuelve false si se ignoró (no finita) o se recortó a los límites.
  bool setReference(std::size_t joint, double position_rad);
  /// Envía POSITION_SETPOINT de cada eje + heartbeat si está armado.
  /// Devuelve false si hay (o se acaba de enclavar) un fallo.
  bool sendReferences();
  /// Instantánea del estado de un eje.
  JointState state(std::size_t joint) const;
  /// Copia el estado de todos los ejes a `out` (tamaño >= jointCount()).
  void readStates(JointState * out, std::size_t count) const;
  /// Comprueba el watchdog de realimentación; enclava fallo si está armado y caducó.
  bool checkWatchdog();

  // ---- utilidades (desarmado) -------------------------------------------------------
  /// Guarda en flash los parámetros de todos los SPARK (confirmado por RESULT_CODE=0).
  void persistParameters();
  /// Reescribe el encoder del motor para que la articulación valga `joint_rad`.
  void setJointPosition(std::size_t joint, double joint_rad);
  /// Envía CLEAR_FAULTS a todos los SPARK (fallas "sticky"; no borra el fallo del driver).
  void clearSparkFaults();
  /// Escribe nuevas ganancias PIDF en un eje (confirmadas).
  void writeGains(std::size_t joint, const PidfGains & gains);

  // ---- consulta ---------------------------------------------------------------------
  bool isOpen() const {return open_.load();}
  bool isConfigured() const {return configured_.load();}
  bool isArmed() const {return armed_.load();}
  bool hasFault() const {return faulted_.load();}
  std::string fault() const;
  bool feedbackFresh() const;
  DriverStatistics statistics() const;
  std::size_t jointCount() const {return config_.joints.size();}
  const DriverConfig & config() const {return config_;}
  const JointConfig & jointConfig(std::size_t joint) const;
  const sparkmax_protocol::PositionProtocol & protocol(std::size_t joint) const;

private:
  enum class RxKind { Status0, Status1, Status2, Response };
  struct RxRoute
  {
    std::size_t axis;
    RxKind kind;
  };

  struct Axis
  {
    JointConfig config;
    std::unique_ptr<sparkmax_protocol::PositionProtocol> protocol;
    // Protegido por state_mutex_
    sparkmax_protocol::Status0 status0;
    sparkmax_protocol::Status1 status1;
    sparkmax_protocol::Status2 status2;
    std::optional<Clock::time_point> status0_at;
    std::optional<Clock::time_point> status1_at;
    std::optional<Clock::time_point> status2_at;
    double reference_motor_rot{0.0};
    bool has_reference{false};
  };

  using Accept = std::function<bool (const sparkmax_protocol::CANPacket &)>;

  void rxLoop();
  void handleFrame(const sparkmax_protocol::CANPacket & packet);
  void trip(const std::string & reason);
  void requireOpen() const;
  void requireDisarmedAndHealthy(const char * operation) const;
  void waitDisableSettle();
  bool send(const sparkmax_protocol::CANPacket & packet);
  bool feedbackFreshLocked(Clock::time_point now) const;

  sparkmax_protocol::CANPacket transact(const sparkmax_protocol::CANPacket & request,
    const char * response_frame, std::size_t axis, const Accept & accept,
    std::chrono::milliseconds timeout, const std::string & label);
  void writeParameter(std::size_t axis, const sparkmax_protocol::ParameterDefinition & parameter,
    double value, const std::string & label);
  void configureAxis(std::size_t axis);
  void applyGains(std::size_t axis, const PidfGains & gains);

  DriverConfig config_;
  std::unique_ptr<sparkmax_protocol::CanTransport> transport_;
  sparkmax_protocol::SparkFrameDatabase::Ptr frames_;
  std::vector<Axis> axes_;
  std::unordered_map<uint32_t, RxRoute> routes_;
  sparkmax_protocol::CANPacket heartbeat_;

  std::atomic<bool> open_{false};
  std::atomic<bool> configured_{false};
  std::atomic<bool> armed_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<bool> stop_rx_{false};
  std::thread rx_thread_;

  mutable std::mutex state_mutex_;
  std::string fault_;
  std::optional<Clock::time_point> last_enable_tx_;
  Clock::time_point next_tx_{};
  int consecutive_tx_failures_{0};

  // Transacción de configuración en curso (una a la vez).
  std::mutex transaction_mutex_;
  std::mutex pending_mutex_;
  std::condition_variable pending_cv_;
  bool pending_active_{false};
  bool pending_done_{false};
  uint32_t pending_arbitration_id_{0};
  const Accept * pending_accept_{nullptr};
  sparkmax_protocol::CANPacket pending_response_;

  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> tx_frames_{0};
  std::atomic<uint64_t> malformed_frames_{0};
  std::atomic<uint64_t> tx_failures_{0};
  std::atomic<uint64_t> error_frames_{0};
};

}  // namespace movemaster_driver
