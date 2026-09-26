// can_packet.hpp
//
// Trama CAN independiente del transporte (equivalente a CANPacket de
// sparkmax_json_protocol.py). SocketCanTransport la convierte a struct can_frame.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace sparkmax_protocol
{

struct CANPacket
{
  static constexpr std::size_t kMaxDataLength = 8;
  static constexpr uint32_t kExtendedIdMask = 0x1FFFFFFFu;

  uint32_t arbitration_id{0};
  std::array<uint8_t, kMaxDataLength> data{};
  uint8_t dlc{0};
  bool is_extended_id{true};
  bool is_remote_frame{false};
  bool is_error_frame{false};
  /// Nombre de la trama en el JSON. Apunta a memoria de SparkFrameDatabase (o a
  /// un literal): sólo es válido mientras la base de tramas siga viva.
  std::string_view frame_name{};

  /// Número de dispositivo (6 bits bajos del ID de arbitraje FRC).
  uint8_t deviceId() const {return static_cast<uint8_t>(arbitration_id & 0x3Fu);}
  /// ID con el número de dispositivo en cero (= "arbId" del JSON).
  uint32_t baseArbitrationId() const {return arbitration_id & ~0x3Fu;}

  std::string dataHex() const;
  std::string toString() const;
};

}  // namespace sparkmax_protocol
