// sparkmax_motion_protocol.hpp
//
// Parser/codec for REV SPARK CAN-frame JSON specs, specialized at the high level
// for MAXMotion Position Control (traducción de sparkmax_json_protocol.py).
//
// Design goals
// ------------
// 1. Frame arbitration IDs, lengths, signal positions, types, endianness and
//    scales come from the REV JSON file rather than being duplicated in C++.
// 2. PIDF/MAXMotion parameter addressing is a configurable parameter layout.
// 3. Only MAXMotion Position Control is exposed at the high-level control API.
// 4. The low-level frame parser remains generic enough to inspect/encode/decode
//    any little-endian frame in the JSON.
//
// This module does not own the heartbeat. MoveMasterDriver owns it.
//
// Dictionary access (Python -> C++):
//   spark["pidf"][0]["p"]                    -> spark.pidf(0)["p"]
//   spark["maxmotion"][0]["cruisevelocity"]  -> spark.maxmotion(0)["cruisevelocity"]
//   spark["frames"]["MAXMOTION_POSITION_SETPOINT"]
//                                            -> spark.frames["MAXMOTION_POSITION_SETPOINT"]
//   spark["setpoint"]                        -> spark.setpoint()

#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sparkmax_protocol/can_bus.hpp"
#include "sparkmax_protocol/can_packet.hpp"
#include "sparkmax_protocol/frame_spec.hpp"
#include "sparkmax_protocol/parameter.hpp"

namespace sparkmax
{

/// Resultado de decode_parameter_write_response() / write_parameter().
struct ParameterWriteResult
{
  int parameter_id{0};
  int parameter_type_code{0};
  std::string parameter_type{"unknown"};
  double current_value{0.0};
  std::uint32_t raw_value{0};
  int result_code{0};
  bool success{false};
  // Sólo los rellena write_parameter():
  std::optional<double> requested_value{};
  std::optional<ParameterDefinition> parameter{};
  std::optional<bool> value_matches{};
};

class SparkMAXMotionProtocol
{
public:
  explicit SparkMAXMotionProtocol(
    const std::string & json_path,
    int device_id = 1,
    const ParameterLayout & parameter_layout = DEFAULT_PARAMETER_LAYOUT,
    int slot_count = 4);

  int device_id;
  int slot_count;
  SparkFrameDatabase frames;
  ParameterLayout parameter_layout;

  // ---- dictionary access -----------------------------------------------------
  const ParameterGroup & pidf(int slot) const {return pidf_.at(slot);}
  const ParameterGroup & maxmotion(int slot) const {return maxmotion_.at(slot);}
  const ParameterGroup & group(const std::string & name, int slot) const;
  const FrameSpec & setpoint() const {return frames["MAXMOTION_POSITION_SETPOINT"];}

  // ---- parameter value packing -----------------------------------------------
  static std::uint32_t pack_parameter_value(double value, const std::string & value_type);
  static double unpack_parameter_value(std::uint32_t raw_u32, const std::string & value_type);

  // ---- frame builders --------------------------------------------------------
  CANPacket parameter_write_packet(const ParameterDefinition & parameter, double value) const;
  CANPacket parameter_read_packet(const ParameterDefinition & parameter) const;
  CANPacket maxmotion_setpoint_packet(
    double setpoint,
    int slot = 0,
    double arbitrary_feedforward = 0.0,
    int arbitrary_feedforward_units = 0) const;

  // ---- decoders --------------------------------------------------------------
  ParameterWriteResult decode_parameter_write_response(const Bytes & data) const;
  double decode_parameter_read_response(
    const ParameterDefinition & parameter, const Bytes & data) const;

  // ---- optional synchronous helpers ------------------------------------------
  /// Send PARAMETER_WRITE and wait synchronously for its response.
  /// NOTE: This consumes frames from bus.recv().
  ParameterWriteResult write_parameter(
    CanBus & bus,
    const ParameterDefinition & parameter,
    double value,
    double timeout = 1.0,
    bool verify = true) const;

  CANPacket send_setpoint(
    CanBus & bus,
    double setpoint,
    int slot = 0,
    double arbitrary_feedforward = 0.0,
    int arbitrary_feedforward_units = 0) const;

  /// Write only the keys supplied in pidf/maxmotion and return responses.
  std::map<std::string, ParameterWriteResult> configure_slot(
    CanBus & bus,
    int slot = 0,
    const std::map<std::string, double> & pidf = {},
    const std::map<std::string, double> & maxmotion = {},
    double timeout = 1.0) const;

  /// Return a serializable summary useful for debugging/introspection.
  nlohmann::ordered_json describe() const;

private:
  std::map<int, ParameterGroup> build_group_by_slot(const std::string & group) const;

  std::map<int, ParameterGroup> pidf_;
  std::map<int, ParameterGroup> maxmotion_;
};

}  // namespace sparkmax
