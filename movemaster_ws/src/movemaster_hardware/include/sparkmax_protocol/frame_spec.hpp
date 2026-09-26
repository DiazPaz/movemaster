// frame_spec.hpp
//
// Traducción de FrameSpec y SparkFrameDatabase.

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sparkmax_protocol/can_packet.hpp"
#include "sparkmax_protocol/signal_codec.hpp"

namespace sparkmax
{

/// Valores de señales por nombre: {"SETPOINT": 0.5, "PID_SLOT": 0, ...}.
using SignalValues = std::map<std::string, double>;

/// Dictionary-like wrapper around one frame entry in the JSON.
class FrameSpec
{
public:
  FrameSpec(std::string key, std::string section, nlohmann::ordered_json spec);

  std::string key;
  std::string section;

  /// Acceso tipo diccionario a la entrada JSON original (frame["name"]).
  const nlohmann::ordered_json & operator[](const std::string & field) const;
  const nlohmann::ordered_json & raw() const {return spec_;}

  std::uint32_t base_arb_id() const {return base_arb_id_;}
  std::size_t length_bytes() const {return length_bytes_;}
  const std::vector<SignalSpec> & signals() const {return signals_;}
  const SignalSpec & signal(const std::string & name) const;
  bool rtr() const {return rtr_;}

  std::uint32_t arbitration_id(int device_id) const;

  Bytes encode_payload(const SignalValues & values, bool require_all = false) const;
  SignalValues decode_payload(const Bytes & data) const;

  CANPacket packet(int device_id, const SignalValues & values = {}) const;

private:
  nlohmann::ordered_json spec_;
  std::uint32_t base_arb_id_;
  std::size_t length_bytes_;
  bool rtr_;
  std::vector<SignalSpec> signals_;
};

/// Loads REV's SPARK frame JSON and offers dictionary-style frame access.
class SparkFrameDatabase
{
public:
  explicit SparkFrameDatabase(const std::string & json_path);

  std::string path;
  nlohmann::ordered_json raw;
  std::map<std::string, FrameSpec> frames;

  const FrameSpec & operator[](const std::string & key) const;
  bool contains(const std::string & key) const {return frames.count(key) != 0;}
  std::size_t size() const {return frames.size();}
  auto begin() const {return frames.begin();}
  auto end() const {return frames.end();}

  std::string frames_version() const;
  nlohmann::ordered_json device_info() const;

  std::map<std::string, const FrameSpec *> find(const std::string & text) const;
};

}  // namespace sparkmax
