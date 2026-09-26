#pragma once
#include "movemaster_hardware/sparkmax_json_protocol.hpp"

namespace movemaster {
// RAII, classic CAN, nonblocking fd. Does not configure bitrate/link state.
class SocketCAN final : public CANBus {
 public:
  explicit SocketCAN(const std::string &channel);
  ~SocketCAN() override;
  SocketCAN(const SocketCAN &) = delete;
  SocketCAN &operator=(const SocketCAN &) = delete;
  void set_filters(const std::vector<std::uint32_t> &extended_ids);
  void send(const CANPacket &packet, double timeout = 0.0) override;
  std::optional<CANPacket> recv(double timeout = 0.0) override;
 private:
  int fd_ = -1;
  bool ready(short events, double timeout) const;
};
}  // namespace movemaster
