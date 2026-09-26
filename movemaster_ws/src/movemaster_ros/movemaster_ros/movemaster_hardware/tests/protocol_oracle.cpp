#include "movemaster_hardware/sparkmax_json_protocol.hpp"
#include <deque>
#include <iostream>
#include <memory>

using namespace movemaster;
namespace {
Json packet_json(const CANPacket &p) {
  return {{"arbitration_id", p.arbitration_id}, {"data", p.data},
      {"is_extended_id", p.is_extended_id}, {"is_remote_frame", p.is_remote_frame},
      {"dlc", p.dlc ? Json(*p.dlc) : Json(nullptr)},
      {"frame_name", p.frame_name ? Json(*p.frame_name) : Json(nullptr)}};
}
struct Bus : CANBus {
  std::deque<CANPacket> incoming;
  std::vector<CANPacket> sent;
  void send(const CANPacket &p, double) override { sent.push_back(p); }
  std::optional<CANPacket> recv(double) override {
    if (incoming.empty()) return std::nullopt;
    auto p = incoming.front(); incoming.pop_front(); return p;
  }
};
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  std::map<int, std::unique_ptr<SparkMAXMotionProtocol>> protocols;
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      const Json q = Json::parse(line);
      const int id = q.value("device_id", 1);
      if (!protocols.count(id)) protocols[id] = std::make_unique<SparkMAXMotionProtocol>(argv[1], id);
      auto &p = *protocols.at(id);
      const auto op = q.at("op").get<std::string>();
      Json out;
      if (op == "frame") out = p.frames[q.at("frame")].encode_payload(q.at("values"), q.value("require_all", false));
      else if (op == "decode") out = p.frames[q.at("frame")].decode_payload(q.at("data").get<Bytes>());
      else if (op == "packet") out = packet_json(p.frames[q.at("frame")].packet(id, q.value("values", Json::object())));
      else if (op == "setpoint") out = packet_json(p.maxmotion_setpoint_packet(q.at("setpoint").get<double>(),
          q.value("slot", 0), q.value("ff", 0.0), q.value("units", 0)));
      else if (op == "encode_bits") out = SignalCodec::encode_bits(q.at("spec"), q.at("value"));
      else if (op == "decode_bits") out = SignalCodec::decode_bits(q.at("spec"), q.at("raw").get<std::uint64_t>());
      else if (op == "pack") out = p.pack_parameter_value(q.at("value"), q.at("type"));
      else if (op == "unpack") out = p.unpack_parameter_value(q.at("raw").get<std::uint32_t>(), q.at("type"));
      else if (op == "describe") out = p.describe();
      else if (op == "find") {
        out = Json::array();
        for (const auto &entry : p.frames.find(q.at("text"))) out.push_back(entry.first);
      } else if (op == "parameter") {
        const auto &d = p[q.at("group")][q.at("slot").get<int>()][q.at("key")];
        out = {{"definition", d}, {"write", packet_json(p.parameter_write_packet(d, q.at("value")))},
            {"read", packet_json(p.parameter_read_packet(d))}};
      } else if (op == "read_response") {
        const auto &d = p[q.at("group")][q.at("slot").get<int>()][q.at("key")];
        out = p.decode_parameter_read_response(d, q.at("data").get<Bytes>());
      } else if (op == "write_response") out = p.decode_parameter_write_response(q.at("data").get<Bytes>());
      else if (op == "sync_write") {
        const auto &d = p[q.at("group")][q.at("slot").get<int>()][q.at("key")];
        Bus bus;
        for (const auto &response : q.at("responses"))
          bus.incoming.push_back({p.frames["PARAMETER_WRITE_RESPONSE"].arbitration_id(id), response.get<Bytes>(),
              true, false, 7, "PARAMETER_WRITE_RESPONSE"});
        out = p.write_parameter(bus, d, q.at("value"), 0.005, q.value("verify", true));
      } else if (op == "custom") {
        SparkMAXMotionProtocol custom(argv[1], id, q.at("layout"), q.value("slot_count", 4));
        out = custom.describe();
      } else if (op == "socketcan_roundtrip") {
        const auto original = p.frames[q.at("frame")].packet(id, q.value("values", Json::object()));
        auto converted = CANPacket::from_socketcan(original.to_socketcan());
        converted.frame_name = original.frame_name;
        out = packet_json(converted);
      } else throw std::invalid_argument("Unknown oracle operation");
      std::cout << Json({{"ok", true}, {"value", out}}).dump() << '\n';
    } catch (const std::exception &e) {
      std::cout << Json({{"ok", false}, {"error", e.what()}}).dump() << '\n';
    }
  }
}
