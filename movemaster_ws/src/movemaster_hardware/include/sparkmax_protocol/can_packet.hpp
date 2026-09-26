// can_packet.hpp
//
// Small transport-neutral CAN packet (traducción de CANPacket en
// sparkmax_json_protocol.py).

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

struct can_frame;  // <linux/can.h>

namespace sparkmax
{

using Bytes = std::vector<std::uint8_t>;

struct CANPacket
{
  std::uint32_t arbitration_id{0};
  Bytes data{};
  bool is_extended_id{true};
  bool is_remote_frame{false};
  std::optional<std::uint8_t> dlc{};
  std::optional<std::string> frame_name{};
  // Sólo en recepción: equivale a can.Message.is_error_frame de python-can.
  bool is_error_frame{false};

  /// Equivalente a to_python_can(): convierte a la trama nativa de SocketCAN.
  can_frame to_can_frame() const;

  /// Inverso de to_can_frame(): lo que python-can entrega en bus.recv().
  static CANPacket from_can_frame(const can_frame & frame);

  /// Equivalente a __repr__.
  std::string repr() const;
};

}  // namespace sparkmax
