// can_bus.hpp
//
// Transporte CAN. Sustituye a can.Bus(interface="socketcan") de python-can con
// la misma idea: send(packet, timeout) y recv(timeout).

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "sparkmax_protocol/can_packet.hpp"

namespace sparkmax
{

/// Interfaz mínima de bus; permite inyectar un bus simulado en pruebas.
class CanBus
{
public:
  virtual ~CanBus() = default;
  virtual void send(const CANPacket & packet, double timeout_s = 0.005) = 0;
  /// Devuelve std::nullopt si no llegó nada dentro de timeout_s.
  virtual std::optional<CANPacket> recv(double timeout_s) = 0;
};

struct CanFilter
{
  std::uint32_t can_id;
  std::uint32_t can_mask{0x1FFFFFFF};
  bool extended{true};
};

/// SocketCAN RAW (can0, vcan0, ...). No configura el bitrate: usa el de la
/// interfaz, igual que python-can.
class SocketCanBus : public CanBus
{
public:
  explicit SocketCanBus(const std::string & channel, const std::vector<CanFilter> & filters = {});
  ~SocketCanBus() override;

  SocketCanBus(const SocketCanBus &) = delete;
  SocketCanBus & operator=(const SocketCanBus &) = delete;

  void send(const CANPacket & packet, double timeout_s = 0.005) override;
  std::optional<CANPacket> recv(double timeout_s) override;
  void shutdown();

  const std::string & channel() const {return channel_;}

private:
  std::string channel_;
  int fd_{-1};
};

}  // namespace sparkmax
