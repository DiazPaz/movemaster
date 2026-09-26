// Pruebas de MoveMasterDriver sobre un bus simulado con 3 SPARK MAX.

#include <cmath>
#include <cstring>
#include <memory>

#include "fake_spark_bus.hpp"
#include "movemaster_driver/movemaster_driver.hpp"
#include "test_harness.hpp"

using namespace movemaster_driver;
using namespace std::chrono_literals;

namespace
{
DriverConfig testConfig()
{
  DriverConfig cfg = makeDefaultConfig(SPARK_FRAMES_JSON);
  cfg.joints[0].transmission.gear_ratio = 100.0;
  cfg.joints[1].transmission.gear_ratio = 50.0;
  cfg.joints[1].transmission.inverted = true;
  cfg.joints[2].transmission.gear_ratio = GearTransmission::ratioFromStages({5.0, 4.0});
  cfg.joints[2].pid_slot = 1;
  cfg.joints[2].gains.p = 0.5;
  cfg.response_timeout = 200ms;
  cfg.disable_settle = 0ms;
  return cfg;
}

struct Rig
{
  FakeSparkBus * bus;
  std::unique_ptr<MoveMasterDriver> driver;
};

Rig makeRig(DriverConfig cfg = testConfig())
{
  auto bus = std::make_unique<FakeSparkBus>(std::vector<uint8_t>{1, 2, 3});
  Rig rig{bus.get(), nullptr};
  rig.driver = std::make_unique<MoveMasterDriver>(std::move(cfg), std::move(bus));
  return rig;
}

float setpointOf(const sparkmax_protocol::CANPacket & p)
{
  float value = 0.0f;
  std::memcpy(&value, p.data.data(), 4);
  return value;
}
}  // namespace

// ---- Conversiones con reductor --------------------------------------------------

TEST_CASE("GearTransmission: rad <-> rotaciones del motor con reductor")
{
  GearTransmission g{100.0, false, 0.0};
  CHECK_NEAR(g.jointToMotorRotations(kTwoPi), 100.0, 1e-12);       // 1 vuelta de salida
  CHECK_NEAR(g.jointToMotorRotations(kPi / 2), 25.0, 1e-12);
  CHECK_NEAR(g.motorRotationsToJoint(50.0), kPi, 1e-12);
  CHECK_NEAR(g.jointVelocityToMotorRpm(kTwoPi), 6000.0, 1e-9);     // 1 rev/s de salida
  CHECK_NEAR(g.motorRpmToJointVelocity(6000.0), kTwoPi, 1e-12);

  GearTransmission inv{50.0, true, 0.5};
  const double q = 1.2345;
  CHECK_NEAR(inv.motorRotationsToJoint(inv.jointToMotorRotations(q)), q, 1e-12);
  CHECK(inv.jointToMotorRotations(1.5) < 0.0);                     // invertido
  CHECK_NEAR(inv.jointToMotorRotations(0.5), 0.0, 1e-12);          // offset
  CHECK_NEAR(GearTransmission::ratioFromStages({5, 4, 5}), 100.0, 0.0);
  CHECK_THROWS(GearTransmission::ratioFromStages({5, 0}));
  CHECK_THROWS((GearTransmission{0.0, false, 0.0}.validate()));
  CHECK_NEAR(radiansToRotations(kPi), 0.5, 1e-15);
  CHECK_NEAR(rpmToRadPerSec(60.0), kTwoPi, 1e-12);
}

TEST_CASE("DriverConfig: por defecto 3 SPARK MAX y validación")
{
  auto cfg = makeDefaultConfig(SPARK_FRAMES_JSON);
  CHECK_EQ(cfg.joints.size(), config::kJointCount);
  CHECK_EQ(cfg.joints.size(), static_cast<std::size_t>(3));
  CHECK_EQ(static_cast<int>(cfg.joints[2].can_id), 3);
  CHECK_NEAR(cfg.joints[0].transmission.gear_ratio, config::kGearRatios[0], 0.0);
  cfg.validate();

  auto dup = cfg;
  dup.joints[1].can_id = 1;
  CHECK_THROWS(dup.validate());
  auto bad_ratio = cfg;
  bad_ratio.joints[0].transmission.gear_ratio = -3.0;
  CHECK_THROWS(bad_ratio.validate());
  auto too_many = cfg;
  while (too_many.joints.size() <= config::kMaxJoints) {
    auto j = too_many.joints[0];
    j.name += "_x" + std::to_string(too_many.joints.size());
    j.can_id = static_cast<uint8_t>(10 + too_many.joints.size());
    too_many.joints.push_back(j);
  }
  CHECK_THROWS(too_many.validate());
  auto six = cfg;
  six.joints.resize(3);
  for (int i = 4; i <= 6; ++i) {
    auto j = six.joints[0];
    j.name = "joint_" + std::to_string(i);
    j.can_id = static_cast<uint8_t>(i);
    six.joints.push_back(j);
  }
  six.validate();  // 6 SPARK MAX es válido
}

// ---- Flujo completo -------------------------------------------------------------------

TEST_CASE("Driver: configure → feedback → arm → referencias → heartbeat")
{
  auto rig = makeRig();
  auto & d = *rig.driver;
  d.open();
  d.configure();
  CHECK(d.isConfigured());

  // Secuencia de parámetros del primer SPARK (ID 1).
  const auto writes = rig.bus->sentWithBase(FakeSparkBus::kParameterWrite);
  std::vector<int> ids_dev1;
  for (const auto & p : writes) {
    if ((p.arbitration_id & 0x3F) == 1) {ids_dev1.push_back(p.data[0]);}
  }
  const std::vector<int> expected{9, 112, 113, 149, 158, 160, 13, 14, 15, 16};
  CHECK(ids_dev1 == expected);
  // El tercer eje usa el slot 1: P = 13 + 8 = 21.
  bool slot1 = false;
  for (const auto & p : writes) {
    if ((p.arbitration_id & 0x3F) == 3 && p.data[0] == 21) {slot1 = true;}
  }
  CHECK(slot1);
  CHECK_EQ(rig.bus->device(1).enabled_statuses & 0b111, 0b111);
  CHECK_EQ(rig.bus->heartbeats(), 0);    // desarmado: sin heartbeat

  CHECK(d.waitForFeedback(1s));
  rig.bus->setPosition(1, 25.0f);       // 25 rot del motor = π/2 rad con 100:1
  std::this_thread::sleep_for(40ms);
  CHECK_NEAR(d.state(0).position_rad, kPi / 2, 1e-6);
  CHECK_NEAR(d.state(0).current_a, 55 * 0.0366300366300366, 1e-9);

  d.arm();
  CHECK(d.isArmed());
  CHECK_NEAR(d.state(0).reference_rad, kPi / 2, 1e-6);   // arm mantiene la PV

  CHECK(d.setReference(0, 1.0));
  CHECK(d.setReference(1, 0.5));
  CHECK(d.setReference(2, -0.25));
  CHECK(d.sendReferences());
  CHECK_EQ(rig.bus->heartbeats(), 1);

  const auto sps = rig.bus->sentWithBase(FakeSparkBus::kPositionSetpoint);
  CHECK_EQ(sps.size(), static_cast<std::size_t>(3));
  CHECK_EQ(sps[0].arbitration_id, 0x02050101u);
  CHECK_NEAR(setpointOf(sps[0]), 100.0 / kTwoPi, 1e-5);          // 1 rad · 100 / 2π
  CHECK_NEAR(setpointOf(sps[1]), -0.5 * 50.0 / kTwoPi, 1e-5);    // invertido
  CHECK_NEAR(setpointOf(sps[2]), -0.25 * 20.0 / kTwoPi, 1e-5);
  CHECK_EQ(rig.bus->device(3).last_slot, 1);
  // Heartbeat del ejemplo validado, después de los setpoints.
  const auto all = rig.bus->sent();
  CHECK_EQ(all.back().arbitration_id, 0x01011840u);
  CHECK_EQ(static_cast<int>(all.back().dlc), 8);
  CHECK_EQ(static_cast<int>(all.back().data[7]), 0xFF);

  std::this_thread::sleep_for(40ms);
  CHECK_NEAR(d.state(0).position_rad, 1.0, 1e-5);
  CHECK_NEAR(d.state(1).position_rad, 0.5, 1e-5);

  d.disarm();
  CHECK(d.sendReferences());            // desarmado: no envía nada
  CHECK_EQ(rig.bus->heartbeats(), 1);
  d.close();
}

TEST_CASE("Driver: límites de posición recortan la referencia")
{
  auto rig = makeRig();
  auto & d = *rig.driver;
  d.open();
  d.configure();
  CHECK(d.waitForFeedback(1s));
  d.arm();
  CHECK(!d.setReference(0, 10.0));      // > π → se recorta
  CHECK(!d.setReference(0, std::nan("")));
  CHECK(d.sendReferences());
  const auto sps = rig.bus->sentWithBase(FakeSparkBus::kPositionSetpoint);
  CHECK_NEAR(setpointOf(sps[0]), 50.0, 1e-4);   // π rad · 100 / 2π
  d.close();
}

TEST_CASE("Driver: watchdog de realimentación desarma y enclava")
{
  auto rig = makeRig();
  auto & d = *rig.driver;
  d.open();
  d.configure();
  CHECK(d.waitForFeedback(1s));
  d.arm();
  CHECK(d.sendReferences());
  rig.bus->mute_status = true;
  std::this_thread::sleep_for(400ms);
  CHECK(!d.sendReferences());
  CHECK(!d.isArmed());
  CHECK(d.hasFault());
  CHECK(d.fault().find("Watchdog") != std::string::npos);
  const int hb = rig.bus->heartbeats();
  CHECK(!d.sendReferences());
  CHECK_EQ(rig.bus->heartbeats(), hb);  // sin heartbeat tras el fallo
  CHECK_THROWS(d.arm());
  d.close();
  // Reabrir limpia el fallo.
  rig.bus->mute_status = false;
  d.open();
  CHECK(!d.hasFault());
  d.close();
}

TEST_CASE("Driver: PARAMETER_WRITE rechazado enclava el fallo")
{
  auto rig = makeRig();
  rig.bus->reject_parameter_id = 112;
  auto & d = *rig.driver;
  d.open();
  CHECK_THROWS(d.configure());
  CHECK(d.hasFault());
  CHECK(d.fault().find("position_factor") != std::string::npos);
  CHECK(!d.isConfigured());
  CHECK_THROWS(d.arm());
  d.close();
}

TEST_CASE("Driver: valor confirmado distinto y timeout de respuesta")
{
  {
    auto rig = makeRig();
    rig.bus->wrong_value = true;
    rig.driver->open();
    CHECK_THROWS(rig.driver->configure());
    CHECK(rig.driver->fault().find("no coincide") != std::string::npos);
    rig.driver->close();
  }
  {
    auto rig = makeRig();
    rig.bus->no_ack = true;
    rig.driver->open();
    CHECK_THROWS(rig.driver->configure());
    CHECK(rig.driver->fault().find("Timeout") != std::string::npos);
    rig.driver->close();
  }
}

TEST_CASE("Driver: arm requiere configure y telemetría")
{
  auto rig = makeRig();
  auto & d = *rig.driver;
  CHECK_THROWS(d.arm());                // bus cerrado
  d.open();
  CHECK_THROWS(d.arm());                // sin configure
  d.configure();
  rig.bus->mute_status = true;
  std::this_thread::sleep_for(350ms);
  CHECK_THROWS(d.arm());                // sin STATUS recientes
  CHECK(!d.hasFault());                 // no es un fallo: simplemente no arma
  d.close();
}

TEST_CASE("Driver: trama de error CAN enclava el fallo")
{
  auto rig = makeRig();
  auto & d = *rig.driver;
  d.open();
  d.configure();
  CHECK(d.waitForFeedback(1s));
  d.arm();
  rig.bus->injectErrorFrame();
  std::this_thread::sleep_for(30ms);
  CHECK(d.hasFault());
  CHECK(!d.isArmed());
  CHECK_EQ(d.statistics().error_frames, 1u);
  d.close();
}

TEST_CASE("Driver: persistencia y ajuste de ganancias desarmado")
{
  auto cfg = testConfig();
  cfg.persist_parameters = true;
  auto rig = makeRig(cfg);
  auto & d = *rig.driver;
  d.open();
  d.configure();
  CHECK_EQ(rig.bus->sentWithBase(FakeSparkBus::kPersist).size(), static_cast<std::size_t>(3));

  PidfGains g;
  g.p = 2.0;
  g.output_min = -0.5;
  g.output_max = 0.5;
  d.writeGains(0, g);
  const auto dev = rig.bus->device(1);
  float p = 0.0f;
  std::memcpy(&p, &dev.parameters.at(13), 4);
  CHECK_NEAR(p, 2.0, 0.0);
  CHECK(dev.parameters.count(20) == 1);  // output_max slot 0
  g.p = -1.0;
  CHECK_THROWS(d.writeGains(0, g));

  CHECK(d.waitForFeedback(1s));
  d.arm();
  CHECK_THROWS(d.writeGains(0, PidfGains{}));  // armado: prohibido
  d.close();
}

TEST_CASE("Driver: setJointPosition reescribe el encoder con el reductor")
{
  auto rig = makeRig();
  auto & d = *rig.driver;
  d.open();
  d.configure();
  d.setJointPosition(0, kPi);           // 50 rot del motor con 100:1
  const auto frames = rig.bus->sentWithBase(0x02052800);
  CHECK_EQ(frames.size(), static_cast<std::size_t>(1));
  CHECK_NEAR(setpointOf(frames[0]), 50.0, 1e-4);
  CHECK_EQ(static_cast<int>(frames[0].data[4]), 3);
  d.close();
}

TEST_MAIN()
