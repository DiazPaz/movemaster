// frame_spec.hpp
//
// FrameSpec: una entrada de "periodicFrames"/"nonPeriodicFrames" del JSON.
// SparkFrameDatabase: carga el JSON de REV y da acceso por nombre de trama.
// (Equivalentes a FrameSpec y SparkFrameDatabase de sparkmax_json_protocol.py.)

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "sparkmax_protocol/can_packet.hpp"
#include "sparkmax_protocol/json.hpp"
#include "sparkmax_protocol/signal_codec.hpp"

namespace sparkmax_protocol
{

/// Valores a codificar, por nombre de señal. Las señales omitidas valen 0,
/// salvo las constantes del JSON (decodedMin == decodedMax), que se rellenan solas.
using SignalValues = std::map<std::string, SignalValue, std::less<>>;

/// Resultado de decodificar una trama completa.
class DecodedFrame
{
public:
  using Storage = std::map<std::string, DecodedSignal, std::less<>>;

  void set(const std::string & name, DecodedSignal signal) {signals_[name] = signal;}
  bool contains(std::string_view name) const {return signals_.find(name) != signals_.end();}
  const DecodedSignal & at(std::string_view name) const;
  double value(std::string_view name) const {return at(name).value;}
  uint64_t raw(std::string_view name) const {return at(name).raw;}

  Storage::const_iterator begin() const {return signals_.begin();}
  Storage::const_iterator end() const {return signals_.end();}
  std::size_t size() const {return signals_.size();}

private:
  Storage signals_;
};

class FrameSpec
{
public:
  static constexpr uint8_t kMaxDeviceId = 63;

  static FrameSpec fromJson(const std::string & key, const std::string & section,
    const json::Value & spec);

  const std::string & key() const {return key_;}
  const std::string & section() const {return section_;}
  const std::string & name() const {return name_;}
  const std::string & description() const {return description_;}
  bool periodic() const {return section_ == "periodicFrames";}

  /// "arbId" del JSON (definido con número de dispositivo = 0).
  uint32_t baseArbId() const {return base_arb_id_;}
  std::size_t lengthBytes() const {return length_bytes_;}
  bool rtr() const {return rtr_;}
  int apiClass() const {return api_class_;}
  int apiIndex() const {return api_index_;}
  int defaultPeriodMs() const {return default_period_ms_;}

  const std::vector<SignalSpec> & signals() const {return signals_;}
  const SignalSpec * findSignal(std::string_view signal_name) const;
  /// Lanza SpecError si la señal no existe.
  const SignalSpec & signal(std::string_view signal_name) const;

  /// ID de arbitraje para un dispositivo (0..63).
  uint32_t arbitrationId(uint8_t device_id) const;
  /// true si `arbitration_id` corresponde a esta trama (cualquier dispositivo).
  bool matches(uint32_t arbitration_id) const
  {
    return (arbitration_id & ~0x3Fu) == (base_arb_id_ & ~0x3Fu);
  }

  /// Codifica la carga útil. Con require_all, toda señal no constante debe venir en values.
  uint64_t encodeRaw(const SignalValues & values, bool require_all = false) const;
  std::array<uint8_t, CANPacket::kMaxDataLength> encodePayload(
    const SignalValues & values, bool require_all = false) const;

  /// Decodifica la carga útil; el tamaño debe coincidir exactamente con lengthBytes.
  DecodedFrame decodePayload(const uint8_t * data, std::size_t length) const;
  DecodedFrame decodePayload(const CANPacket & packet) const
  {
    return decodePayload(packet.data.data(), packet.dlc);
  }

  /// Construye la trama para un dispositivo.
  CANPacket packet(uint8_t device_id, const SignalValues & values = {}) const;
  /// Construye la trama a partir de bits ya empaquetados (ruta sin asignaciones).
  CANPacket packetFromRaw(uint8_t device_id, uint64_t raw_payload) const;

  /// Convierte una carga little-endian a entero de 64 bits y viceversa.
  static uint64_t payloadToRaw(const uint8_t * data, std::size_t length);
  static void rawToPayload(uint64_t raw, uint8_t * data, std::size_t length);

private:
  std::string key_;
  std::string section_;
  std::string name_;
  std::string description_;
  uint32_t base_arb_id_{0};
  std::size_t length_bytes_{0};
  bool rtr_{false};
  int api_class_{-1};
  int api_index_{-1};
  int default_period_ms_{-1};
  std::vector<SignalSpec> signals_;
};

struct DeviceInfo
{
  std::string device_type;
  int device_type_number{-1};
  std::string manufacturer;
  int manufacturer_number{-1};
};

class SparkFrameDatabase
{
public:
  using Ptr = std::shared_ptr<const SparkFrameDatabase>;

  static Ptr loadFile(const std::string & json_path);
  static Ptr loadString(std::string_view json_text, std::string source = "<memoria>");
  static Ptr fromJson(const json::Value & root, std::string source);

  const std::string & source() const {return source_;}
  const std::string & framesVersion() const {return frames_version_;}
  const std::string & specVersion() const {return spec_version_;}
  const DeviceInfo & deviceInfo() const {return device_info_;}

  bool contains(std::string_view key) const {return find(key) != nullptr;}
  const FrameSpec * find(std::string_view key) const;
  /// Lanza SpecError si la trama no existe.
  const FrameSpec & at(std::string_view key) const;
  const FrameSpec & operator[](std::string_view key) const {return at(key);}

  /// Trama correspondiente a un ID recibido (ignora los 6 bits de dispositivo).
  const FrameSpec * matchArbitrationId(uint32_t arbitration_id) const;

  /// Busca texto en clave, nombre o descripción (equivale a find() en Python).
  std::vector<const FrameSpec *> search(std::string_view text) const;

  const std::vector<FrameSpec> & frames() const {return frames_;}
  std::size_t size() const {return frames_.size();}

private:
  SparkFrameDatabase() = default;

  std::string source_;
  std::string frames_version_{"unknown"};
  std::string spec_version_{"unknown"};
  DeviceInfo device_info_;
  std::vector<FrameSpec> frames_;
  std::unordered_map<std::string, std::size_t> by_key_;
  std::unordered_map<uint32_t, std::size_t> by_base_id_;
};

}  // namespace sparkmax_protocol
