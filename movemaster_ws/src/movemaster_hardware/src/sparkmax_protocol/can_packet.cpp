#include "sparkmax_protocol/can_packet.hpp"

#include <linux/can.h>

#include <algorithm>
#include <cstdio>
#include <stdexcept>

namespace sparkmax
{

can_frame CANPacket::to_can_frame() const
{
  if (data.size() > CAN_MAX_DLEN) {
    throw std::invalid_argument("CANPacket: classic CAN admite máximo 8 bytes");
  }
  can_frame frame{};
  frame.can_id = arbitration_id & (is_extended_id ? CAN_EFF_MASK : CAN_SFF_MASK);
  if (is_extended_id) {frame.can_id |= CAN_EFF_FLAG;}
  if (is_remote_frame) {frame.can_id |= CAN_RTR_FLAG;}
  frame.can_dlc = dlc.value_or(static_cast<std::uint8_t>(data.size()));
  std::copy(data.begin(), data.end(), frame.data);
  return frame;
}

CANPacket CANPacket::from_can_frame(const can_frame & frame)
{
  CANPacket packet;
  packet.is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
  packet.is_remote_frame = (frame.can_id & CAN_RTR_FLAG) != 0;
  packet.is_error_frame = (frame.can_id & CAN_ERR_FLAG) != 0;
  packet.arbitration_id = frame.can_id & (packet.is_extended_id ? CAN_EFF_MASK : CAN_SFF_MASK);
  const auto len = std::min<std::uint8_t>(frame.can_dlc, CAN_MAX_DLEN);
  packet.dlc = frame.can_dlc;
  if (!packet.is_remote_frame) {packet.data.assign(frame.data, frame.data + len);}
  return packet;
}

std::string CANPacket::repr() const
{
  std::string bytes;
  if (data.empty()) {
    bytes = "<RTR/no data>";
  } else {
    char hex[4];
    for (std::size_t i = 0; i < data.size(); ++i) {
      std::snprintf(hex, sizeof(hex), i ? " %02X" : "%02X", data[i]);
      bytes += hex;
    }
  }
  char id[16];
  std::snprintf(id, sizeof(id), "0x%08X", arbitration_id);
  return "CANPacket(frame=" + (frame_name ? "'" + *frame_name + "'" : std::string("None")) +
         ", id=" + id + ", dlc=" + (dlc ? std::to_string(*dlc) : std::string("None")) +
         ", data=" + bytes + ")";
}

}  // namespace sparkmax
