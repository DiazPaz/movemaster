// socketcan.hpp
//
// Capa de transporte CAN. CanTransport es la interfaz abstracta (permite usar un
// bus simulado en pruebas); SocketCanTransport la implementa sobre SocketCAN de
// Linux (PF_CAN/SOCK_RAW), el mismo mecanismo que python-can con interface="socketcan".
//
// El bitrate NO se configura aquí: se usa el que ya tenga la interfaz, p.ej.
//   sudo ip link set can0 up type can bitrate 1000000

#pragma once

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "sparkmax_protocol/can_packet.hpp"

namespace sparkmax_protocol
{

/// Filtro de recepción: se acepta la trama si (id & mask) == (filter.id & mask).
struct CanFilter
{
  uint32_t id{0};
  uint32_t mask{CANPacket::kExtendedIdMask};
  bool extended{true};
};

class CanTransportError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class CanTransport
{
public:
  virtual ~CanTransport() = default;

  /// Abre el bus con los filtros indicados (vacío = recibir todo).
  virtual void open(const std::vector<CanFilter> & filters) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  /// Envía una trama. Devuelve false si no hubo espacio en el buffer de TX dentro
  /// del timeout; lanza CanTransportError ante errores del sistema.
  virtual bool send(const CANPacket & packet, std::chrono::microseconds timeout) = 0;

  /// Espera una trama hasta `timeout`. Devuelve false si no llegó ninguna.
  /// Las tramas de error del controlador se entregan con is_error_frame = true.
  virtual bool receive(CANPacket & packet, std::chrono::microseconds timeout) = 0;

  virtual std::string name() const = 0;
};

class SocketCanTransport : public CanTransport
{
public:
  /// @param interface_name  p.ej. "can0" o "vcan0"
  /// @param receive_error_frames  entrega tramas de error (bus-off, controlador...)
  explicit SocketCanTransport(std::string interface_name, bool receive_error_frames = true);
  ~SocketCanTransport() override;

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;

  void open(const std::vector<CanFilter> & filters) override;
  void close() override;
  bool isOpen() const override {return fd_ >= 0;}
  bool send(const CANPacket & packet, std::chrono::microseconds timeout) override;
  bool receive(CANPacket & packet, std::chrono::microseconds timeout) override;
  std::string name() const override {return "socketcan:" + interface_name_;}

private:
  std::string interface_name_;
  bool receive_error_frames_{true};
  int fd_{-1};
};

}  // namespace sparkmax_protocol
