// Pruebas de SparkMaxProtocol sin hardware.
// Los bytes esperados se generaron con sparkmax_json_protocol.py (misma JSON),
// para garantizar que el port C++ produce exactamente las mismas tramas.

#include <cstring>
#include <string>

#include "sparkmax_protocol/json.hpp"
#include "sparkmax_protocol/position_protocol.hpp"
#include "sparkmax_protocol/sparkmax_protocol.hpp"
#include "test_harness.hpp"

using namespace sparkmax_protocol;

namespace
{
SparkFrameDatabase::Ptr database()
{
  static SparkFrameDatabase::Ptr db = SparkFrameDatabase::loadFile(SPARK_FRAMES_JSON);
  return db;
}

std::string hex(const CANPacket & p) {return p.dataHex();}

CANPacket incoming(uint32_t id, std::initializer_list<uint8_t> bytes)
{
  CANPacket p;
  p.arbitration_id = id;
  p.dlc = static_cast<uint8_t>(bytes.size());
  std::size_t i = 0;
  for (uint8_t b : bytes) {p.data[i++] = b;}
  return p;
}
}  // namespace

// ---- JSON -------------------------------------------------------------------

TEST_CASE("json: tipos básicos, orden y enteros de 64 bits")
{
  const auto v = json::parse(
    R"({"b": 2, "a": [1, -3, 2.5e1, true, null], "s": "xé\n", "big": 18446744073709551615})");
  CHECK(v.isObject());
  CHECK_EQ(v.asObject()[0].key, std::string("b"));   // conserva el orden
  CHECK_EQ(v.at("a").asArray()[1].asInt64(), -3);
  CHECK_NEAR(v.at("a").asArray()[2].asDouble(), 25.0, 0.0);
  CHECK(v.at("a").asArray()[3].asBool());
  CHECK(v.at("a").asArray()[4].isNull());
  CHECK_EQ(v.at("s").asString(), std::string("x\xc3\xa9\n"));
  CHECK_EQ(v.at("big").asUint64(), 18446744073709551615ull);
  CHECK_THROWS(json::parse("{\"a\": }"));
  CHECK_THROWS(json::parse("[1, 2] x"));
}

TEST_CASE("json: carga spark-frames-2.1.0")
{
  const auto db = database();
  CHECK_EQ(db->framesVersion(), std::string("2.1.0"));
  CHECK_EQ(db->deviceInfo().manufacturer_number, 5);
  CHECK_EQ(db->size(), static_cast<std::size_t>(13 + 315));
  CHECK(db->contains("POSITION_SETPOINT"));
  CHECK_EQ(db->at("POSITION_SETPOINT").baseArbId(), 0x02050100u);
  CHECK_EQ(db->at("STATUS_2").lengthBytes(), static_cast<std::size_t>(8));
  CHECK(db->at("READ_PARAMETER_12_AND_13").rtr());
  CHECK(db->matchArbitrationId(0x0205B883) == &db->at("STATUS_2"));
  CHECK(!db->search("maxmotion").empty());
  CHECK_THROWS(db->at("NO_EXISTE"));
}

// ---- SignalCodec ---------------------------------------------------------------

TEST_CASE("codec: int con signo, escala y rangos")
{
  const auto & ff = database()->at("POSITION_SETPOINT").signal("ARBITRARY_FEEDFORWARD");
  const uint64_t bits = SignalCodec::encodeBits(ff, SignalValue(-2.0));
  CHECK_EQ(bits, 0xF800u);
  CHECK_NEAR(SignalCodec::decodeBits(ff, bits).value, -2.0, 1e-3);
  CHECK_THROWS(SignalCodec::encodeBits(ff, SignalValue(40.0)));  // > encodedMax
  const auto & slot = database()->at("POSITION_SETPOINT").signal("PID_SLOT");
  CHECK_THROWS(SignalCodec::encodeBits(slot, SignalValue(4)));
}

TEST_CASE("codec: señales big-endian no soportadas (como en Python)")
{
  const auto & build = database()->at("GET_FIRMWARE_VERSION").signal("BUILD");
  CHECK(build.is_big_endian);
  CHECK_THROWS(SignalCodec::decodeBits(build, 0));
}

// ---- Paridad con sparkmax_json_protocol.py ---------------------------------------

TEST_CASE("paridad Python: PARAMETER_WRITE")
{
  MAXMotionProtocol spark(database(), 3);
  auto p = spark.parameterWritePacket(spark.pidf(0)["p"], 0.05);
  CHECK_EQ(p.arbitration_id, 0x02053803u);
  CHECK_EQ(static_cast<int>(p.dlc), 5);
  CHECK_EQ(hex(p), std::string("0D CD CC 4C 3D"));
  CHECK_EQ(hex(spark.parameterWritePacket(spark.pidf(2)["f"], -1.25)),
    std::string("20 00 00 A0 BF"));
  CHECK_EQ(hex(spark.parameterWritePacket(spark.maxmotion(1)["cruisevelocity"], 900)),
    std::string("AB 00 00 61 44"));
}

TEST_CASE("paridad Python: MAXMOTION_POSITION_SETPOINT")
{
  MAXMotionProtocol spark(database(), 3);
  auto a = spark.maxmotionSetpointPacket(0.5, 1, 1.5, FeedforwardUnits::DutyCycle);
  CHECK_EQ(a.arbitration_id, 0x02050203u);
  CHECK_EQ(hex(a), std::string("00 00 00 3F 00 06 05 00"));
  CHECK_EQ(hex(spark.maxmotionSetpointPacket(-12.75, 3, -2.0)),
    std::string("00 00 4C C1 00 F8 03 00"));
}

TEST_CASE("paridad Python: SET_STATUSES_ENABLED, PERSIST y lectura RTR")
{
  PositionProtocol spark(database(), 3);
  CHECK_EQ(hex(spark.setStatusesEnabledPacket(0b111, 0b111)), std::string("07 00 07 00"));
  auto persist = spark.persistParametersPacket();  // MAGIC_NUMBER automático
  CHECK_EQ(persist.arbitration_id, 0x0205FFC3u);
  CHECK_EQ(hex(persist), std::string("A3 3A"));
  auto read = spark.parameterReadPacket(spark.pidf(0)["p"]);
  CHECK_EQ(read.arbitration_id, 0x02053D83u);
  CHECK(read.is_remote_frame);
  CHECK_EQ(static_cast<int>(read.dlc), 8);
}

TEST_CASE("paridad Python: STATUS_0 / STATUS_2 / PARAMETER_WRITE_RESPONSE")
{
  PositionProtocol spark(database(), 3);
  const auto s0 = spark.decodeStatus0(
    incoming(0x0205B803, {0x10, 0x27, 0x34, 0x12, 0x56, 0x2A, 0x21, 0x00}));
  CHECK_NEAR(s0.applied_output, 0.3082369457075716, 1e-12);
  CHECK_NEAR(s0.voltage, 4.131868131868117, 1e-9);
  CHECK_NEAR(s0.current, 50.439560439560395, 1e-9);
  CHECK_NEAR(s0.motor_temperature, 42.0, 0.0);
  CHECK(s0.hard_forward_limit);
  CHECK(!s0.hard_reverse_limit);
  CHECK(s0.primary_heartbeat_lock);

  const auto s2 = spark.decodeStatus2(
    incoming(0x0205B883, {0x00, 0x00, 0xF7, 0x42, 0x00, 0x00, 0x88, 0xC0}));
  CHECK_NEAR(s2.velocity, 123.5, 0.0);
  CHECK_NEAR(s2.position, -4.25, 0.0);

  const auto r = spark.decodeParameterWriteResponse(
    incoming(0x02053843, {0x0D, 0x03, 0xCD, 0xCC, 0x4C, 0x3D, 0x00}));
  CHECK_EQ(static_cast<int>(r.parameter_id), 13);
  CHECK(r.parameter_type == ParameterType::Float);
  CHECK(r.success);
  CHECK_NEAR(r.current_value, 0.05, 1e-7);

  // Tramas de otro dispositivo o de longitud incorrecta se rechazan.
  CHECK_THROWS(spark.decodeStatus2(incoming(0x0205B884, {0, 0, 0, 0, 0, 0, 0, 0})));
  CHECK_THROWS(spark.decodeStatus2(incoming(0x0205B883, {0, 0, 0, 0})));
}

// ---- PositionProtocol ----------------------------------------------------------

TEST_CASE("PositionProtocol: POSITION_SETPOINT según spark-frames-2.1.0")
{
  PositionProtocol spark(database(), 3);
  CHECK_EQ(std::string(spark.controlType()), std::string("Position Control"));
  auto p = spark.positionSetpointPacket(2.5, 2, 0.3);
  CHECK_EQ(p.arbitration_id, 0x02050103u);
  CHECK_EQ(static_cast<int>(p.dlc), 8);
  CHECK_EQ(hex(p), std::string("00 00 20 40 33 01 02 00"));  // idéntico a Python
  CHECK_EQ(std::string(p.frame_name), std::string("POSITION_SETPOINT"));

  const auto back = spark.decodePositionSetpoint(p);
  CHECK_NEAR(back.motor_rotations, 2.5, 0.0);
  CHECK_EQ(back.slot, 2);
  CHECK_NEAR(back.arbitrary_feedforward, 0.3, 1e-3);

  CHECK_THROWS(spark.positionSetpointPacket(std::nan(""), 0));
  CHECK_THROWS(spark.positionSetpointPacket(1e40, 0));
  CHECK_THROWS(spark.positionSetpointPacket(1.0, 4));
}

TEST_CASE("PositionProtocol: catálogo pidf/setup y alias")
{
  PositionProtocol spark(database(), 1);
  CHECK_EQ(static_cast<int>(spark.pidf(0)["kp"].parameter_id), 13);
  CHECK_EQ(static_cast<int>(spark.pidf(1)["P"].parameter_id), 21);
  CHECK_EQ(static_cast<int>(spark.pidf(3)["output_max"].parameter_id), 44);
  CHECK_EQ(static_cast<int>(spark.group("setup")["position_wrapping"].parameter_id), 149);
  CHECK(spark.group("setup")["feedback_sensor"].value_type == ParameterType::Uint);
  CHECK_EQ(spark.pidf(0)["p"].readFrameName(), std::string("READ_PARAMETER_12_AND_13"));
  CHECK_THROWS(spark.pidf(0)["no_existe"]);
  CHECK_THROWS(spark.group("maxmotion"));
  CHECK_THROWS(PositionProtocol(database(), 64));
}

TEST_CASE("parámetros: empaquetado por tipo")
{
  CHECK_EQ(packParameterValue(1.0, ParameterType::Float), 0x3F800000u);
  CHECK_EQ(packParameterValue(-1, ParameterType::Int), 0xFFFFFFFFu);
  CHECK_EQ(packParameterValue(20, ParameterType::Uint), 20u);
  CHECK_EQ(packParameterValue(1, ParameterType::Bool), 1u);
  CHECK_THROWS(packParameterValue(-1, ParameterType::Uint));
  CHECK_NEAR(unpackParameterValue(0xFFFFFFFFu, ParameterType::Int), -1.0, 0.0);
}

TEST_CASE("comandos: SET_PRIMARY_ENCODER_POSITION rellena DATA_TYPE constante")
{
  PositionProtocol spark(database(), 2);
  auto p = spark.setPrimaryEncoderPositionPacket(1.0);
  CHECK_EQ(p.arbitration_id, 0x02052802u);
  CHECK_EQ(hex(p), std::string("00 00 80 3F 03"));
  auto stop = spark.stopFollowerModePacket();
  CHECK_EQ(static_cast<int>(stop.dlc), 0);
  CHECK_EQ(stop.arbitration_id, 0x02057C82u);
}

TEST_MAIN()
