// Vectores de referencia generados con sparkmax_json_protocol.py (Python),
// que ya fue validado con un SPARK MAX real. La traducción C++ debe producir
// exactamente los mismos bytes.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "sparkmax_protocol/sparkmax_motion_protocol.hpp"

using sparkmax::Bytes;
using sparkmax::SparkMAXMotionProtocol;

namespace
{

Bytes hex(const std::string & text)
{
  Bytes out;
  for (std::size_t i = 0; i + 1 < text.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
  }
  return out;
}

class ProtocolTest : public ::testing::Test
{
protected:
  SparkMAXMotionProtocol p{SPARK_FRAMES_JSON, 3};
};

}  // namespace

TEST_F(ProtocolTest, LoadsDatabase)
{
  EXPECT_EQ(p.frames.frames_version(), "2.1.0");
  EXPECT_EQ(p.frames.size(), 328u);
  EXPECT_EQ(p.frames.find("maxmotion").size(), 3u);
  EXPECT_EQ(p.frames["MAXMOTION_POSITION_SETPOINT"].arbitration_id(3), 33882627u);
  EXPECT_EQ(p.frames["STATUS_0"].arbitration_id(3), 0x205b803u);
  EXPECT_THROW(p.frames["STATUS_0"].arbitration_id(64), std::invalid_argument);
}

TEST_F(ProtocolTest, MaxMotionSetpointPacket)
{
  auto pk = p.maxmotion_setpoint_packet(0.5, 1, -1.2345, 1);
  EXPECT_EQ(pk.arbitration_id, 0x2050203u);
  EXPECT_EQ(pk.data, hex("0000003f10fb0500"));
  EXPECT_EQ(pk.dlc, 8);
  EXPECT_FALSE(pk.is_remote_frame);

  EXPECT_EQ(p.maxmotion_setpoint_packet(-12.75).data, hex("00004cc100000000"));
  EXPECT_EQ(
    p.maxmotion_setpoint_packet(0.5).repr(),
    "CANPacket(frame='MAXMOTION_POSITION_SETPOINT', id=0x02050203, dlc=8, "
    "data=00 00 00 3F 00 00 00 00)");

  EXPECT_THROW(p.maxmotion_setpoint_packet(0.0, 4), std::invalid_argument);
  EXPECT_THROW(p.maxmotion_setpoint_packet(0.0, 0, 0.0, 2), std::invalid_argument);
}

TEST_F(ProtocolTest, RoundsHalfToEvenLikePython)
{
  const auto & f = p.setpoint();
  EXPECT_EQ(f.encode_payload({{"ARBITRARY_FEEDFORWARD", 2.5 * 0.0009765923}}),
    hex("0000000002000000"));
  EXPECT_EQ(f.encode_payload({{"ARBITRARY_FEEDFORWARD", 3.5 * 0.0009765923}}),
    hex("0000000004000000"));
}

TEST_F(ProtocolTest, ParameterLayoutAndAliases)
{
  EXPECT_EQ(p.pidf(0)["p"].parameter_id, 13);
  EXPECT_EQ(p.pidf(3)["f"].parameter_id, 40);
  EXPECT_EQ(p.maxmotion(2)["maxaccel"].parameter_id, 177);
  EXPECT_EQ(p.maxmotion(1)["Cruise Velocity"].key, "cruise_velocity");
  EXPECT_EQ(p.pidf(0)["p"].read_frame_name(), "READ_PARAMETER_12_AND_13");
  EXPECT_THROW(p.pidf(0)["x"], std::out_of_range);
}

TEST_F(ProtocolTest, ParameterPackets)
{
  auto pk = p.parameter_write_packet(p.pidf(0)["p"], 0.05);
  EXPECT_EQ(pk.arbitration_id, 0x2053803u);
  EXPECT_EQ(pk.data, hex("0dcdcc4c3d"));
  EXPECT_EQ(pk.dlc, 5);

  EXPECT_EQ(p.parameter_write_packet(p.maxmotion(1)["cruisevelocity"], 900).data,
    hex("ab00006144"));

  auto rd = p.parameter_read_packet(p.pidf(0)["p"]);
  EXPECT_EQ(rd.arbitration_id, 0x2053d83u);
  EXPECT_TRUE(rd.is_remote_frame);
  EXPECT_TRUE(rd.data.empty());
  EXPECT_EQ(rd.dlc, 8);

  EXPECT_DOUBLE_EQ(p.decode_parameter_read_response(p.pidf(0)["i"], hex("0000803f0000003f")), 1.0);

  EXPECT_EQ(SparkMAXMotionProtocol::pack_parameter_value(-5, "int"), 4294967291u);
  EXPECT_EQ(SparkMAXMotionProtocol::pack_parameter_value(1, "bool"), 1u);
  EXPECT_THROW(SparkMAXMotionProtocol::pack_parameter_value(-1, "uint"), std::invalid_argument);
}

TEST_F(ProtocolTest, OtherCommandFrames)
{
  auto st = p.frames["SET_STATUSES_ENABLED"].packet(3, {{"MASK", 5}, {"ENABLED_BITFIELD", 5}});
  EXPECT_EQ(st.arbitration_id, 0x2050403u);
  EXPECT_EQ(st.data, hex("05000500"));

  auto persist = p.frames["PERSIST_PARAMETERS"].packet(3, {{"MAGIC_NUMBER", 15011}});
  EXPECT_EQ(persist.arbitration_id, 0x205ffc3u);
  EXPECT_EQ(persist.data, hex("a33a"));
  EXPECT_THROW(
    p.frames["PERSIST_PARAMETERS"].packet(3, {{"MAGIC_NUMBER", 1}}), std::invalid_argument);

  auto stop = p.frames["STOP_FOLLOWER_MODE"].packet(3);
  EXPECT_EQ(stop.arbitration_id, 0x2057c83u);
  EXPECT_TRUE(stop.data.empty());
  EXPECT_EQ(stop.dlc, 0);
}

TEST_F(ProtocolTest, DecodesStatusFrames)
{
  auto st2 = p.frames["STATUS_2"].decode_payload(hex("0000c8420000203f"));
  EXPECT_DOUBLE_EQ(st2.at("PRIMARY_ENCODER_VELOCITY"), 100.0);
  EXPECT_DOUBLE_EQ(st2.at("PRIMARY_ENCODER_POSITION"), 0.625);

  auto st0 = p.frames["STATUS_0"].decode_payload(hex("1234abcdef567890"));
  EXPECT_DOUBLE_EQ(st0.at("APPLIED_OUTPUT"), 0.41087984862819293);
  EXPECT_DOUBLE_EQ(st0.at("VOLTAGE"), 25.633699633699543);
  EXPECT_DOUBLE_EQ(st0.at("CURRENT"), 140.51282051282038);
  EXPECT_DOUBLE_EQ(st0.at("MOTOR_TEMPERATURE"), 86.0);
  EXPECT_DOUBLE_EQ(st0.at("PRIMARY_HEARTBEAT_LOCK"), 1.0);

  EXPECT_THROW(p.frames["STATUS_2"].decode_payload(hex("0000")), std::invalid_argument);
}

TEST_F(ProtocolTest, DecodesParameterWriteResponse)
{
  auto r = p.decode_parameter_write_response(hex("a6030000614400"));
  EXPECT_EQ(r.parameter_id, 166);
  EXPECT_EQ(r.parameter_type_code, 3);
  EXPECT_EQ(r.parameter_type, "float");
  EXPECT_DOUBLE_EQ(r.current_value, 900.0);
  EXPECT_EQ(r.result_code, 0);
  EXPECT_TRUE(r.success);
}

TEST_F(ProtocolTest, DescribeSummary)
{
  const auto d = p.describe();
  EXPECT_EQ(d["frames_version"], "2.1.0");
  EXPECT_EQ(d["device_id"], 3);
  EXPECT_EQ(d["maxmotion_position_setpoint"]["device_arb_id"], 33882627u);
  EXPECT_EQ(d["pidf"]["1"]["p"]["parameter_id"], 21);
}
