#pragma once

#include <nlohmann/json.hpp>
#include <linux/can.h>
#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace movemaster {
using Json = nlohmann::ordered_json;
using Bytes = std::vector<std::uint8_t>;
extern const Json DEFAULT_PARAMETER_LAYOUT;
extern const std::map<std::string, int> PARAMETER_TYPE_CODE;
extern const std::map<int, std::string> PARAMETER_TYPE_NAME;

struct SpecError : std::runtime_error { using std::runtime_error::runtime_error; };
struct TimeoutError : std::runtime_error { using std::runtime_error::runtime_error; };

struct CANPacket {
  std::uint32_t arbitration_id = 0;
  Bytes data;
  bool is_extended_id = true;
  bool is_remote_frame = false;
  std::optional<int> dlc;
  std::optional<std::string> frame_name;
  // Native equivalent of Python's to_python_can(). No Python runtime required.
  can_frame to_socketcan() const;
  static CANPacket from_socketcan(const can_frame &frame);
  std::string repr() const;
};

// Single-consumer transport contract, also usable with a fake bus in tests.
class CANBus {
 public:
  virtual ~CANBus() = default;
  virtual void send(const CANPacket &packet, double timeout = 0.0) = 0;
  virtual std::optional<CANPacket> recv(double timeout = 0.0) = 0;
};

class SignalCodec {
 public:
  static void _require_little_endian(const Json &signal_spec);
  static std::uint32_t _float_to_u32(double value);
  static double _u32_to_float(std::uint32_t value);
  static std::uint64_t encode_bits(const Json &signal_spec, const Json &decoded_value);
  static Json decode_bits(const Json &signal_spec, std::uint64_t raw_bits);
};

class FrameSpec {
 public:
  std::string key, section;
  FrameSpec(std::string key, std::string section, Json spec);
  const Json &operator[](const std::string &name) const { return _spec.at(name); }
  Json get(const std::string &name, Json fallback = nullptr) const;
  auto begin() const { return _spec.begin(); }
  auto end() const { return _spec.end(); }
  std::size_t size() const { return _spec.size(); }
  std::uint32_t base_arb_id() const;
  int length_bytes() const;
  const Json &signals() const;
  bool rtr() const;
  std::uint32_t arbitration_id(int device_id) const;
  Bytes encode_payload(const Json &values = Json::object(), bool require_all = false) const;
  Json decode_payload(const Bytes &data) const;
  CANPacket packet(int device_id, const Json &values = Json::object()) const;
 private:
  Json _spec;
};

class SparkFrameDatabase {
 public:
  std::filesystem::path path;
  Json raw;
  std::map<std::string, FrameSpec> frames;
  explicit SparkFrameDatabase(const std::filesystem::path &json_path);
  const FrameSpec &operator[](const std::string &key) const { return frames.at(key); }
  auto begin() const { return frames.begin(); }
  auto end() const { return frames.end(); }
  std::size_t size() const { return frames.size(); }
  bool contains(const std::string &key) const { return frames.count(key) != 0; }
  std::string frames_version() const;
  Json device_info() const;
  std::map<std::string, FrameSpec> find(const std::string &text) const;
};

struct ParameterDefinition {
  std::string group, key;
  int slot = 0, parameter_id = 0;
  std::string value_type = "float", description;
  std::optional<std::string> unit;
  int pair_start_id() const { return parameter_id & ~1; }
  int pair_index() const { return parameter_id - pair_start_id(); }
  std::string read_frame_name() const;
};
void to_json(Json &out, const ParameterDefinition &parameter);

class ParameterGroup {
 public:
  using Definitions = std::map<std::string, ParameterDefinition>;
  ParameterGroup(Definitions canonical, std::map<std::string, std::string> aliases);
  static std::string _normalize(std::string key);
  const ParameterDefinition &operator[](const std::string &key) const;
  auto begin() const { return _canonical.begin(); }
  auto end() const { return _canonical.end(); }
  std::size_t size() const { return _canonical.size(); }
  Definitions as_dict() const { return _canonical; }
 private:
  Definitions _canonical;
  std::map<std::string, std::string> _aliases;
};

class SparkMAXMotionProtocol {
 public:
  using Slots = std::map<int, ParameterGroup>;
  // A small typed facade preserves spark["pidf"][0]["p"] and spark["frames"][name].
  class Access {
   public:
    explicit Access(const Slots *slots) : slots_(slots) {}
    explicit Access(const SparkFrameDatabase *frames) : frames_(frames) {}
    explicit Access(const FrameSpec *frame) : frame_(frame) {}
    const ParameterGroup &operator[](int slot) const;
    const FrameSpec &operator[](const std::string &name) const;
    const FrameSpec &as_frame() const;
   private:
    const Slots *slots_ = nullptr;
    const SparkFrameDatabase *frames_ = nullptr;
    const FrameSpec *frame_ = nullptr;
  };

  int device_id, slot_count;
  SparkFrameDatabase frames;
  Json parameter_layout;
  explicit SparkMAXMotionProtocol(const std::filesystem::path &json_path,
      int device_id = 1, Json parameter_layout = nullptr, int slot_count = 4);
  // The facade holds references to this object's fields; keep its address stable.
  SparkMAXMotionProtocol(const SparkMAXMotionProtocol &) = delete;
  SparkMAXMotionProtocol &operator=(const SparkMAXMotionProtocol &) = delete;
  const Access &operator[](const std::string &key) const { return _access.at(key); }
  auto begin() const { return _access.begin(); }
  auto end() const { return _access.end(); }
  std::size_t size() const { return _access.size(); }

  static std::uint32_t pack_parameter_value(const Json &value, std::string value_type);
  static Json unpack_parameter_value(std::uint32_t raw_u32, std::string value_type);
  CANPacket parameter_write_packet(const ParameterDefinition &parameter, const Json &value) const;
  CANPacket parameter_read_packet(const ParameterDefinition &parameter) const;
  CANPacket maxmotion_setpoint_packet(double setpoint, int slot = 0,
      double arbitrary_feedforward = 0.0, int arbitrary_feedforward_units = 0) const;
  Json decode_parameter_write_response(const Bytes &data) const;
  Json decode_parameter_read_response(const ParameterDefinition &parameter, const Bytes &data) const;
  static void _send_packet(CANBus &bus, const CANPacket &packet);
  // Synchronous helpers: only use when this caller exclusively owns bus.recv().
  Json write_parameter(CANBus &bus, const ParameterDefinition &parameter,
      const Json &value, double timeout = 1.0, bool verify = true) const;
  CANPacket send_setpoint(CANBus &bus, double setpoint, int slot = 0,
      double arbitrary_feedforward = 0.0, int arbitrary_feedforward_units = 0) const;
  Json configure_slot(CANBus &bus, int slot = 0, const Json &pidf = Json::object(),
      const Json &maxmotion = Json::object(), double timeout = 1.0) const;
  Json describe() const;
 private:
  Slots _pidf, _maxmotion;
  std::map<std::string, Access> _access;
  Slots _build_group_by_slot(const std::string &group) const;
};

// Architectural name, without renaming the already validated Python API.
using SparkMaxProtocol = SparkMAXMotionProtocol;
}  // namespace movemaster
