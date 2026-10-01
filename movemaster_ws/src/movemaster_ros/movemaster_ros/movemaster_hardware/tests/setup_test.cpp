// joints.json v2 loading and validation, commissioning and bus survey, without CAN.
#include "simulated_spark_bus.hpp"
#define MOVEMASTER_COMMISSION_NO_MAIN
#include "../examples/spark_commission.cpp"
#include <unistd.h>
#include <cctype>
#include <fstream>
#include <sstream>

using namespace movemaster;
using namespace movemaster::testing;
namespace {
std::filesystem::path spec, config_dir, scratch;
const std::vector<std::string> kCommissionOrder{"RESET_SAFE_PARAMETERS", "PARAMETER_WRITE", "PARAMETER_WRITE",
    "PARAMETER_WRITE", "PARAMETER_WRITE", "PERSIST_PARAMETERS"};

Json read_json(const std::filesystem::path &path) {
  std::ifstream file(path);
  Json data;
  file >> data;
  return data;
}
DriverConfig load(const Json &data) {
  const auto path = scratch / "joints.json";
  std::ofstream(path) << data.dump(2);
  auto config = load_driver_config(path, spec);
  validate_driver_config(config);
  return config;
}
void shipped_configurations() {
  for (const auto *name : {"joints.json", "joints.example.json"})
    validate_driver_config(load_driver_config(config_dir / name, spec));
  const auto bench = load_driver_config(config_dir / "joints.json", spec).joints.at(0);
  check(bench.device_id == 1 && bench.spark.idle_mode == IdleMode::kBrake && bench.spark.current_limit_a == 40,
      "joints.json baseline");
  check(bench.mode == ControlMode::kMAXMotionPosition && bench.slot == 0 && !bench.slots.at(0).maxmotion.is_null(),
      "joints.json must keep the MAXMotion behavior it had before v2");
}
// Each mistake names the field, and the previous format explains where things moved.
void loader_errors() {
  const auto base = read_json(config_dir / "joints.json");
  const auto joint = base["joints"].begin().key();
  const auto broken = [&](auto mutate, const std::string &expected) {
    auto data = base;
    mutate(data["joints"][joint]);
    rejects_with([&] { load(data); }, expected);
  };
  broken([](Json &j) { j["pidf"] = j["slots"]["0"]["pidf"]; }, "pidf moved in joints.json v2");
  broken([](Json &j) { j["slot"] = 0; }, "slot moved in joints.json v2");
  broken([](Json &j) { j["spark"]["idle"] = "brake"; }, "spark.idle is not a known key");
  broken([](Json &j) { j["spark"]["motor_type"] = "brushed"; }, "spark.motor_type must be \"brushless\"");
  broken([](Json &j) { j["spark"]["idle_mode"] = "hold"; }, "spark.idle_mode must be one of: coast, brake");
  broken([](Json &j) { j["spark"]["current_limit_a"] = 40.5; }, "spark.current_limit_a must be an integer");
  broken([](Json &j) { j["spark"]["current_limit_a"] = 0; }, "current_limit_a must be in 1..80 A");
  broken([](Json &j) { j.erase("spark"); }, "spark is missing");
  broken([](Json &j) { j["control"].erase("max_following_error_rad"); }, "control.max_following_error_rad is missing");
  broken([](Json &j) { j["control"]["max_following_error_rad"] = 0; }, "max_following_error_rad must be positive");
  broken([](Json &j) { j["control"]["mode"] = "velocity"; }, "control.mode must be one of: position, maxmotion");
  broken([](Json &j) { j["control"]["slot"] = 2; }, "control slot 2 is not configured");
  broken([](Json &j) { j["slots"]["4"] = j["slots"]["0"]; }, "slots are indexed \"0\" to \"3\"");
  broken([](Json &j) { j["slots"]["0"]["output_rang"] = {-1, 1}; }, "slots.0.output_rang is not a known key");
  broken([](Json &j) { j["slots"]["0"]["output_range"] = {0, 1}; }, "output_range must satisfy");
  broken([](Json &j) { j["slots"]["0"]["output_range"] = 0.5; }, "slots.0.output_range must be [min, max]");
  broken([](Json &j) { j["slots"]["0"]["pidf"].erase("d"); }, "PIDF accepts p, i, d, f");
  broken([](Json &j) { j["slots"]["0"].erase("maxmotion"); }, "MAXMotion control needs a maxmotion block");
  broken([](Json &j) { j["max_velocity_rad_s"] = 1.0; }, "cruise_velocity exceeds max_velocity_rad_s");
  broken([](Json &j) { j.erase("max_velocity_rad_s"); }, "max_velocity_rad_s is missing");
  auto data = base;
  data["period"] = 0.02;
  rejects_with([&] { load(data); }, "joints.json: period is not a known key");

  // Position-only slots are valid, MAXMotion just cannot select them.
  data = base;
  auto &j = data["joints"][joint];
  j["slots"]["1"] = {{"pidf", j["slots"]["0"]["pidf"]}, {"output_range", {-0.25, 0.5}}};
  j["control"] = {{"mode", "position"}, {"slot", 1}, {"max_following_error_rad", 0.2}};
  const auto position = load(data).joints.at(0);
  check(position.mode == ControlMode::kPosition && position.slot == 1, "Position control not loaded");
  check(position.slots.at(1).output_min == -0.25 && position.slots.at(1).output_max == 0.5 &&
      position.slots.at(1).maxmotion.is_null(), "Position-only slot not loaded");
  check(position.slots.at(0).output_min == -1 && position.slots.at(0).output_max == 1, "Default output range");
}
bool sent_frame(const SimulatedSparkBus &bus, const char *name) {
  for (const auto &p : bus.sent)
    if ((p.arbitration_id & ~0x3FU) == (bus.frames[name].base_arb_id() & ~0x3FU)) return true;
  return false;
}
// Reset to REV defaults, write the baseline, persist; nothing reaches flash after a failure.
void commissioning() {
  const SparkMAXMotionProtocol spark(spec, 3);
  const auto commission = [&](SimulatedSparkBus &bus, SparkBaseline baseline) {
    SparkSetup(bus, spark, 0.035, 0.2).commission(baseline);
  };
  SimulatedSparkBus bus(spec, {3});
  bus.persist_codes = {255, 255, 0};  // in progress, as the validated backend observed
  commission(bus, {IdleMode::kCoast, 30});
  check(bus.names_sent_to(3) == kCommissionOrder, "Commissioning must reset, write the baseline, then persist");
  check(bus.value(3, 2) == 1 && bus.value(3, 6) == 0 && bus.value(3, 59) == 30 && bus.value(3, 60) == 30,
      "Baseline values");
  check(bus.frames["RESET_SAFE_PARAMETERS"].decode_payload(bus.sent.front().data).at("MAGIC_NUMBER") == 36292 &&
      bus.frames["PERSIST_PARAMETERS"].decode_payload(bus.sent.back().data).at("MAGIC_NUMBER") == 15011,
      "Flash commands must carry the magic numbers of the spec");

  SimulatedSparkBus rejecting(spec, {3});
  rejecting.reject_ack = true;
  rejects_with([&] { commission(rejecting, {IdleMode::kBrake, 40}); }, "Parameter ACK mismatch: CAN 3, motor_type");
  check(!sent_frame(rejecting, "PERSIST_PARAMETERS"), "Persisted after a rejected write");

  SimulatedSparkBus refused(spec, {3});
  refused.persist_codes = {255, 1};
  rejects_with([&] { commission(refused, {IdleMode::kBrake, 40}); }, "PERSIST_PARAMETERS rejected on CAN 3: RESULT_CODE 1");

  SimulatedSparkBus silent(spec, {3});
  silent.silent_flash = true;
  rejects_with([&] { commission(silent, {IdleMode::kBrake, 40}); },
      "Timeout waiting for RESET_SAFE_PARAMETERS_RESPONSE on CAN 3");
  check(!sent_frame(silent, "PARAMETER_WRITE"), "Wrote the baseline without a confirmed reset");
}
// The spark_commission tool on a simulated bus: joint selection, review, refusals and partial failure.
void commission_tool() {
  const auto config = config_for(spec, 3);
  rejects_with([&] { spark_commission::parse(config, {"joint_9"}); }, "Joint desconocido en JOINTS.json: joint_9");
  const auto review = spark_commission::parse(config, {});
  const auto every = spark_commission::parse(config, {"--apply"});
  const auto second = spark_commission::parse(config, {"--apply", "joint_2"});
  check(!review.apply && review.joints.size() == 3 && every.apply && every.joints.size() == 3, "Joint selection");
  check(second.apply && second.joints.size() == 1 && second.joints[0]->name == "joint_2",
      "Naming one joint must select only that joint");
  std::string output;
  const auto run = [&](SimulatedSparkBus &bus, const spark_commission::Request &request) {
    std::ostringstream out;
    const int status = spark_commission::run(config, request, bus, out, 0.03, 0.2);
    output = out.str();
    return status;
  };
  const auto says = [&](const char *text) { return output.find(text) != std::string::npos; };
  SimulatedSparkBus bus(spec, {1, 2, 3});
  check(run(bus, review) == 0 && bus.sent.empty() && says("Revision sin cambios"), "Review must only listen");
  check(run(bus, second) == 0 && says("joint_2: guardado en flash"), "Commissioning joint_2 failed");
  check(bus.names_sent_to(2) == kCommissionOrder && bus.names_sent_to(1).empty() && bus.names_sent_to(3).empty(),
      "Only joint_2 may be commissioned");

  SimulatedSparkBus busy(spec, {1, 2, 3});
  busy.foreign_heartbeat = true;
  check(run(busy, every) == 1 && busy.sent.empty() && says("heartbeat de habilitacion"), "Enable heartbeat ignored");
  SimulatedSparkBus missing(spec, {1, 3});
  check(run(missing, every) == 1 && missing.sent.empty() && says("joint_2  CAN 2  NO RESPONDE"), "Missing SPARK ignored");
  SimulatedSparkBus refused(spec, {1, 2, 3});
  refused.persist_codes = {0, 1};
  check(run(refused, every) == 1 && says("joint_1: guardado en flash") && says("Error en joint_2") &&
      refused.names_sent_to(3).empty(), "A failed SPARK must stop the commissioning");
}
// spec/SparkParameters-v0.1.2.md: parameter rows by ID (name, type) and enum names in order.
struct ParameterTable {
  std::map<int, std::pair<std::string, std::string>> rows;
  std::map<std::string, std::vector<std::string>> enums;
  int value(const std::string &type, const std::string &name) const {
    const auto &names = enums.at(type);
    return static_cast<int>(std::find(names.begin(), names.end(), name) - names.begin());
  }
};
ParameterTable read_parameter_table(const std::filesystem::path &path) {
  std::ifstream file(path);
  check(static_cast<bool>(file), "Cannot open " + path.string());
  const auto trim = [](std::string text) {
    text.erase(0, text.find_first_not_of(" \t\r"));
    text.erase(text.find_last_not_of(" \t\r") + 1);
    return text;
  };
  const auto digits = [](const std::string &text) {
    return !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isdigit(c); });
  };
  ParameterTable table;
  std::string enum_name;
  for (std::string line; std::getline(file, line);) {
    const auto text = trim(line);
    if (text.empty()) enum_name.clear();
    else if (text.back() == ':' && text.find(' ') == std::string::npos) enum_name = text.substr(0, text.size() - 1);
    else if (!enum_name.empty() && text.rfind("* ", 0) == 0) table.enums[enum_name].push_back(trim(text.substr(2)));
    else if (text.front() == '|') {
      std::vector<std::string> cells;
      std::istringstream row(text.substr(1));
      for (std::string cell; std::getline(row, cell, '|');) cells.push_back(trim(cell));
      if (cells.size() >= 3 && digits(cells[1])) table.rows[std::stoi(cells[1])] = {cells[0], cells[2]};
    }
  }
  check(table.rows.size() > 150 && table.enums.count("MotorType"), "Parameter table not parsed: " + path.string());
  return table;
}
// Every parameter ID, type and enum value the code writes agrees with the mapped REV table.
void parameter_table() {
  const auto table = read_parameter_table(spec.parent_path() / "SparkParameters-v0.1.2.md");
  const std::map<std::string, std::string> table_type{{"uint", "UINT32"}, {"float", "FLOAT"}, {"bool", "BOOL"}};
  const auto agrees = [&](const ParameterDefinition &p, const std::string &name) {
    const auto row = table.rows.find(p.parameter_id);
    check(row != table.rows.end() && row->second.first == name && row->second.second == table_type.at(p.value_type),
        "ID " + std::to_string(p.parameter_id) + " (" + p.key + ") is not " + name + " in the parameter table");
  };
  for (const auto *p : {&rev::kMotorType, &rev::kIdleMode, &rev::kFeedbackSensor, &rev::kSmartCurrentStallLimit,
           &rev::kSmartCurrentFreeLimit, &rev::kPositionConversionFactor, &rev::kVelocityConversionFactor,
           &rev::kPositionWrapping, &rev::kStatus0Period, &rev::kStatus2Period})
    agrees(*p, p->description);
  // DEFAULT_PARAMETER_LAYOUT keeps the Python descriptions; these are its names in the table.
  const std::map<std::string, std::string> layout_name{{"p", "P"}, {"i", "I"}, {"d", "D"}, {"f", "F"},
      {"cruise_velocity", "MAXMotion Max Velocity"}, {"max_acceleration", "MAXMotion Max Accel"},
      {"allowed_profile_error", "MAXMotion Allowed Closed Loop Error"}};
  const SparkMAXMotionProtocol spark(spec, 1);
  for (int slot = 0; slot < 4; ++slot) {
    for (const auto &p : {rev::output_min(slot), rev::output_max(slot)}) agrees(p, p.description);
    for (const auto *group : {"pidf", "maxmotion"})
      for (const auto &item : spark[group][slot]) agrees(item.second, layout_name.at(item.first) + " " + std::to_string(slot));
  }
  check(rev::kBrushless == table.value("MotorType", "BRUSHLESS") &&
      rev::kMainEncoder == table.value("Sensor", "MAIN_ENCODER") &&
      static_cast<int>(IdleMode::kCoast) == table.value("IdleMode", "COAST") &&
      static_cast<int>(IdleMode::kBrake) == table.value("IdleMode", "BRAKE"), "Enum values differ from the parameter table");
}
// The survey only listens: which SPARKs answer, and whether something enables motors.
void survey() {
  SimulatedSparkBus bus(spec, {1, 4});
  const auto quiet = survey_bus(bus, bus.frames, 0.03);
  check(quiet.sparks == std::set<int>{1, 4} && !quiet.enable_heartbeat, "Quiet bus survey");
  bus.foreign_heartbeat = true;
  check(survey_bus(bus, bus.frames, 0.03).enable_heartbeat, "Enable heartbeat not detected");
  check(bus.sent.empty(), "The survey transmitted");
}
}  // namespace
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  spec = argv[1];
  config_dir = argv[2];
  scratch = std::filesystem::temp_directory_path() / ("movemaster_setup_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(scratch);
  int status = 0;
  try {
    shipped_configurations();
    parameter_table();
    loader_errors();
    commissioning();
    commission_tool();
    survey();
    std::cout << "PASS: shipped joints.json files, parameter IDs against SparkParameters-v0.1.2.md, "
        "v2 loader errors, Position-only slots, commissioning order, "
        "magic numbers and failures, spark_commission selection and refusals, bus survey.\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    status = 1;
  }
  std::filesystem::remove_all(scratch);
  return status;
}
