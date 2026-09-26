// Pruebas de MoveMasterDriver sin hardware: un bus simulado responde como tres
// SPARK MAX (ACK de parámetros, STATUS_0/STATUS_2 periódicos). Representa las
// comunicaciones; no valida dinámica, PID ni firmware real.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "movemaster_driver/movemaster_driver.hpp"

using movemaster::DriverConfig;
using movemaster::JointConfig;
using movemaster::MoveMasterDriver;
using sparkmax::CANPacket;
using sparkmax::SparkFrameDatabase;

namespace
{

using Clock = std::chrono::steady_clock;

class FakeSparkBus : public sparkmax::CanBus
{
public:
  FakeSparkBus() : db(SPARK_FRAMES_JSON) {}

  void send(const CANPacket & packet, double) override
  {
    sent.push_back(packet);
    const int dev = static_cast<int>(packet.arbitration_id & 0x3F);
    const auto is = [&](const char * name) {
        return db[name].arbitration_id(dev) == packet.arbitration_id;
      };

    if (silent) {return;}
    if (is("PARAMETER_WRITE")) {
      const auto v = db["PARAMETER_WRITE"].decode_payload(packet.data);
      const int id = static_cast<int>(v.at("PARAMETER_ID"));
      reply(dev, "PARAMETER_WRITE_RESPONSE", {
          {"PARAMETER_ID", id}, {"PARAMETER_TYPE", type_of(id)}, {"VALUE", v.at("VALUE")},
          {"RESULT_CODE", id == nack_parameter ? 1 : 0}});
    } else if (is("STOP_FOLLOWER_MODE")) {
      reply(dev, "STOP_FOLLOWER_MODE_RESPONSE", {});
    } else if (is("SET_STATUSES_ENABLED")) {
      const auto v = db["SET_STATUSES_ENABLED"].decode_payload(packet.data);
      reply(dev, "SET_STATUSES_ENABLED_RESPONSE", {
          {"RESULT_CODE", 0}, {"SPECIFIED_MASK", v.at("MASK")},
          {"ENABLED_BITFIELD", v.at("ENABLED_BITFIELD")}});
      status_enabled.insert(dev);
    } else if (is("PERSIST_PARAMETERS")) {
      reply(dev, "PERSIST_PARAMETERS_RESPONSE", {{"RESULT_CODE", 255}});
      reply(dev, "PERSIST_PARAMETERS_RESPONSE", {{"RESULT_CODE", 0}});
    } else if (is("MAXMOTION_POSITION_SETPOINT")) {
      position[dev] = db["MAXMOTION_POSITION_SETPOINT"].decode_payload(packet.data).at("SETPOINT");
    }
  }

  std::optional<CANPacket> recv(double timeout_s) override
  {
    if (rx.empty() && auto_status && Clock::now() - last_status > std::chrono::milliseconds(10)) {
      last_status = Clock::now();
      for (int dev : status_enabled) {
        reply(dev, "STATUS_2", {{"PRIMARY_ENCODER_VELOCITY", 0.0},
            {"PRIMARY_ENCODER_POSITION", position[dev]}});
        reply(dev, "STATUS_0", {{"CURRENT", 1.5}, {"PRIMARY_HEARTBEAT_LOCK", 1}});
      }
    }
    if (rx.empty()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(std::min(timeout_s, 0.001)));
      return std::nullopt;
    }
    auto packet = rx.front();
    rx.pop_front();
    return packet;
  }

  std::vector<CANPacket> sent_named(const std::string & name) const
  {
    std::vector<CANPacket> out;
    for (const auto & p : sent) {
      if (p.frame_name == name) {out.push_back(p);}
    }
    return out;
  }

  SparkFrameDatabase db;
  std::vector<CANPacket> sent;
  std::deque<CANPacket> rx;
  std::set<int> status_enabled;
  std::map<int, double> position{{1, 0.25}, {2, -0.5}, {3, 1.0}};
  bool auto_status{true};
  bool silent{false};
  int nack_parameter{-1};

private:
  static int type_of(int id)
  {
    if (id == 9 || id == 158 || id == 160) {return 2;}  // uint
    if (id == 149) {return 4;}                          // bool
    return 3;                                           // float
  }

  void reply(int dev, const char * name, const sparkmax::SignalValues & values)
  {
    rx.push_back(db[name].packet(dev, values));
  }

  Clock::time_point last_status{};
};

JointConfig joint(const std::string & name, int id, double gear)
{
  JointConfig j;
  j.name = name;
  j.device_id = id;
  j.gear_ratio = gear;
  j.pidf = {{"p", 1.0}, {"i", 0.0}, {"d", 0.0}, {"f", 0.0}};
  j.motion_profile = {{"max_acceleration", 500.0}, {"cruise_velocity", 900.0},
    {"allowed_profile_error", 0.1}};
  return j;
}

DriverConfig config()
{
  DriverConfig c;
  c.spec_path = SPARK_FRAMES_JSON;
  c.response_timeout_s = 0.1;
  c.disable_settle_s = 0.0;
  c.joints = {joint("joint_1", 1, 100.0), joint("joint_2", 2, -50.0), joint("joint_3", 3, 10.0)};
  return c;
}

class DriverTest : public ::testing::Test
{
protected:
  std::shared_ptr<FakeSparkBus> bus{std::make_shared<FakeSparkBus>()};

  std::unique_ptr<MoveMasterDriver> ready(DriverConfig c = config())
  {
    auto driver = std::make_unique<MoveMasterDriver>(c, bus);
    driver->open();
    driver->initialize();
    EXPECT_TRUE(driver->wait_for_feedback(1.0));
    return driver;
  }
};

}  // namespace

TEST_F(DriverTest, ConvertsRadiansAndRotations)
{
  MoveMasterDriver driver(config(), bus);
  EXPECT_NEAR(driver.joint_to_motor(0, 2 * M_PI), 100.0, 1e-12);
  EXPECT_NEAR(driver.motor_to_joint(1, -50.0), 2 * M_PI, 1e-12);
  EXPECT_NEAR(driver.motor_rpm_to_joint(2, 600.0), 2 * M_PI, 1e-12);
  EXPECT_NEAR(driver.motor_to_joint(0, driver.joint_to_motor(0, 0.3)), 0.3, 1e-12);
}

TEST_F(DriverTest, InitializesEveryAxisInOrder)
{
  auto driver = ready();
  EXPECT_TRUE(driver->initialized());
  // Por eje: stop follower + 6 parámetros + statuses + 4 PIDF + 3 perfil.
  EXPECT_EQ(bus->sent_named("STOP_FOLLOWER_MODE").size(), 3u);
  EXPECT_EQ(bus->sent_named("SET_STATUSES_ENABLED").size(), 3u);
  EXPECT_EQ(bus->sent_named("PARAMETER_WRITE").size(), 3u * (6 + 4 + 3));
  EXPECT_TRUE(bus->sent_named("MAXMOTION_POSITION_SETPOINT").empty());
  EXPECT_TRUE(bus->sent_named("REFERENCE_HEARTBEAT").empty());
}

TEST_F(DriverTest, ArmHoldsMeasuredPositionThenStreamsSetpointsAndHeartbeat)
{
  auto driver = ready();
  const auto targets = driver->arm();
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_NEAR(targets[0], driver->motor_to_joint(0, 0.25), 1e-9);
  EXPECT_NEAR(driver->state(1).position, driver->motor_to_joint(1, -0.5), 1e-9);

  bus->sent.clear();
  driver->write({0.1, std::nan(""), -0.2});
  ASSERT_EQ(bus->sent.size(), 4u);  // 3 SP + 1 heartbeat, en ese orden
  EXPECT_EQ(bus->sent[3].arbitration_id, movemaster::HEARTBEAT_ID);
  EXPECT_EQ(bus->sent[3].data, sparkmax::Bytes(8, 0xFF));

  const auto & sp = bus->db["MAXMOTION_POSITION_SETPOINT"];
  EXPECT_FLOAT_EQ(sp.decode_payload(bus->sent[0].data).at("SETPOINT"),
    static_cast<float>(driver->joint_to_motor(0, 0.1)));
  EXPECT_FLOAT_EQ(sp.decode_payload(bus->sent[1].data).at("SETPOINT"), -0.5f);  // NaN = mantener
  EXPECT_FLOAT_EQ(sp.decode_payload(bus->sent[2].data).at("SETPOINT"),
    static_cast<float>(driver->joint_to_motor(2, -0.2)));

  driver->disarm();
  bus->sent.clear();
  driver->write({0.0, 0.0, 0.0});
  EXPECT_TRUE(bus->sent.empty());  // desarmado: silencio
}

TEST_F(DriverTest, RejectedParameterLatchesFault)
{
  bus->nack_parameter = 13;  // P del slot 0
  MoveMasterDriver driver(config(), bus);
  driver.open();
  EXPECT_THROW(driver.initialize(), std::runtime_error);
  ASSERT_TRUE(driver.fault());
  EXPECT_NE(driver.fault()->find("rechazado"), std::string::npos);
  EXPECT_THROW(driver.arm(), std::runtime_error);
}

TEST_F(DriverTest, MissingResponseTimesOut)
{
  bus->silent = true;
  MoveMasterDriver driver(config(), bus);
  driver.open();
  EXPECT_THROW(driver.initialize(), std::runtime_error);
  EXPECT_NE(driver.fault()->find("Timeout"), std::string::npos);
}

TEST_F(DriverTest, WatchdogTripsWhenStatusStops)
{
  auto driver = ready();
  driver->arm();
  driver->write({std::nan(""), std::nan(""), std::nan("")});
  bus->auto_status = false;
  bus->rx.clear();
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  driver->read();
  EXPECT_THROW(driver->write({0.0, 0.0, 0.0}), std::runtime_error);
  EXPECT_FALSE(driver->armed());
  EXPECT_NE(driver->fault()->find("Watchdog"), std::string::npos);
}

TEST_F(DriverTest, OutOfLimitTargetIsRejectedWithoutTransmitting)
{
  auto c = config();
  c.joints[0].position_limits = std::pair{-1.0, 1.0};
  auto driver = ready(c);
  driver->arm();
  bus->sent.clear();
  EXPECT_THROW(driver->write({2.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_TRUE(bus->sent.empty());
}

TEST_F(DriverTest, ArmRequiresGainsAndProfile)
{
  auto c = config();
  c.joints[2].pidf = {{"p", 1.0}};
  auto driver = ready(c);
  EXPECT_THROW(driver->arm(), std::runtime_error);
}

TEST_F(DriverTest, PersistWaitsFor255ThenSucceeds)
{
  auto c = config();
  c.persist_parameters = true;
  auto driver = ready(c);
  EXPECT_EQ(bus->sent_named("PERSIST_PARAMETERS").size(), 3u);
  EXPECT_FALSE(driver->fault());
}

TEST_F(DriverTest, RejectsInvalidConfiguration)
{
  auto c = config();
  c.joints[1].device_id = 1;
  EXPECT_THROW(MoveMasterDriver(c, bus), std::invalid_argument);
  c = config();
  c.joints[0].motion_profile["cruise_velocity"] = 0.0;
  EXPECT_THROW(MoveMasterDriver(c, bus), std::invalid_argument);
  c = config();
  c.joints[0].gear_ratio = 0.0;
  EXPECT_THROW(MoveMasterDriver(c, bus), std::invalid_argument);
}
