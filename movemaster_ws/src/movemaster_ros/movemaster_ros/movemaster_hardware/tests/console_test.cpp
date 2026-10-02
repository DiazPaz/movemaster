#define MOVEMASTER_CONSOLE_NO_MAIN
#include "../examples/spark_console.cpp"
#include <algorithm>
#include <functional>

namespace {
using namespace movemaster;
void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
struct FakeDriver {
  bool enabled = false, fail_write = false;
  int activations = 0, deactivations = 0;
  std::vector<JointState> state = {{0.1, 0, 0.2, false}};
  std::vector<JointControl> control = {{ControlMode::kMAXMotionPosition, 0}};
  std::vector<double> sent;
  void read() {}
  void activate() { enabled = true; ++activations; }
  void deactivate() noexcept { enabled = false; ++deactivations; }
  bool active() const { return enabled; }
  const std::vector<JointState> &states() const { return state; }
  const std::vector<JointControl> &controls() const { return control; }
  // Same rules as MoveMasterDriver: slot 0 has MAXMotion, slot 1 is Position only.
  void set_control(std::size_t, ControlMode mode, int slot) {
    if (slot != 0 && slot != 1) throw std::invalid_argument("slot is not configured");
    if (mode == ControlMode::kMAXMotionPosition && slot == 1) throw std::invalid_argument("slot 1 has no MAXMotion profile");
    control[0] = {mode, slot};
  }
  void write(const std::vector<double> &target) {
    if (fail_write && sent.size() == 3) throw std::runtime_error("Injected driver/watchdog error");
    check(enabled, "Console wrote while inactive");
    check(control[0].mode == ControlMode::kMAXMotionPosition || std::abs(target.at(0) - state[0].position) <= 0.2,
        "Console sent a Position target the driver rejects");
    sent.push_back(target.at(0));
  }
};
DriverConfig config() {
  DriverConfig cfg;
  cfg.period_s = 0.005;
  JointConfig joint;
  joint.name = "joint_1"; joint.device_id = 1;
  joint.gear_ratio = 10; joint.direction = -1; joint.zero_offset_rad = 0.1;
  joint.min_position_rad = -2; joint.max_position_rad = 2;
  joint.max_following_error_rad = 0.2;
  cfg.joints = {joint};
  return cfg;
}
struct Action { const char *text; int wait_ms; };
struct Result { std::string output, error; };
Result session(FakeDriver &driver, const std::vector<Action> &actions) {
  int fds[2];
  if (::pipe(fds) != 0) throw std::runtime_error("pipe failed");
  const int original = ::fcntl(fds[0], F_GETFL);
  bool writer_ok = true;
  std::thread writer([&] {
    for (const auto &action : actions) {
      const std::string text = action.text;
      if (!text.empty() && ::write(fds[1], text.data(), text.size()) != static_cast<ssize_t>(text.size())) {
        writer_ok = false;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(action.wait_ms));
    }
    ::close(fds[1]);
  });
  std::ostringstream output;
  Result result;
  spark_console::stop_requested = 0;
  try { spark_console::run(driver, config(), fds[0], output); }
  catch (const std::exception &e) { result.error = e.what(); }
  writer.join();
  const int restored = ::fcntl(fds[0], F_GETFL);
  ::close(fds[0]);
  check(writer_ok, "Could not feed commands to console");
  check(original == restored, "stdin flags were not restored");
  result.output = output.str();
  return result;
}
}
int main() {
  try {
    FakeDriver driver;
    auto result = session(driver, {{"sp 0.5\n", 30}, {"on\n", 40}, {"sp ", 120},
        {"0.5\n", 50}, {"pv\n", 20}, {"off\n", 20}, {"sp 1\n", 20}, {"q\n", 20}});
    check(result.error.empty(), "Interactive scenario failed");
    check(driver.activations == 1 && !driver.active(), "Enable/disable state incorrect");
    check(driver.sent.size() >= 12, "Control loop stalled while command was incomplete");
    check(std::count(driver.sent.begin(), driver.sent.end(), 0.1) >= 8, "Initial measured hold lost during input pause");
    check(driver.sent.front() == 0.1, "Pre-enable SP was retained instead of measured position");
    const double expected = 0.1 - 3.14159265358979323846 / 10;
    check(std::any_of(driver.sent.begin(), driver.sent.end(), [&](double q) { return std::abs(q - expected) < 1e-12; }),
        "Motor rotations not converted correctly");
    check(result.output.find("Primero escribe on") != std::string::npos, "SP allowed while inactive");
    check(result.output.find("Ultima PV") != std::string::npos, "PV command failed");

    FakeDriver eof;
    result = session(eof, {{"on\n", 70}});
    check(result.error.empty() && eof.activations == 1 && !eof.active() && eof.deactivations > 0,
        "EOF did not disable the axis");

    FakeDriver limit;
    result = session(limit, {{"on\n", 40}, {"sp 1000\n", 30}, {"q\n", 20}});
    check(result.error.empty() && !limit.active(), "Out-of-range command did not disable");
    check(result.output.find("SP fuera de limites") != std::string::npos, "Limit rejection not reported");
    check(std::all_of(limit.sent.begin(), limit.sent.end(), [](double q) { return q == 0.1; }), "Invalid target transmitted");

    FakeDriver failure;
    failure.fail_write = true;
    result = session(failure, {{"on\n", 80}});
    check(!result.error.empty() && !failure.active() && failure.deactivations > 0, "Driver error did not disable");

    FakeDriver invalid;
    result = session(invalid, {{"on\n", 30}, {"sp 1garbage\n", 20}, {"sp nan\n", 20}, {"q\n", 20}});
    check(result.error.empty(), "Invalid numeric syntax unexpectedly aborted console");
    check(std::all_of(invalid.sent.begin(), invalid.sent.end(), [](double q) { return q == 0.1; }), "Invalid syntax changed target");

    // Position steps stay within the following error; mode changes hold the measured position.
    const double tau = 6.283185307179586, small = 0.1 - 0.2 * tau / 10, large = 0.1 - 0.5 * tau / 10,
        long_move = 0.1 - 1.0 * tau / 10;
    FakeDriver modes;
    result = session(modes, {{"on\n", 30}, {"sp 1\n", 30}, {"mode position\n", 30}, {"sp 0.5\n", 30},
        {"sp 0.2\n", 30}, {"slot 1\n", 30}, {"mode maxmotion\n", 30}, {"slot 0\n", 30},
        {"mode maxmotion\n", 30}, {"sp 1\n", 30}, {"pv\n", 20}, {"slot 5\n", 20}, {"mode velocity\n", 20},
        {"q\n", 20}});
    check(result.error.empty() && modes.deactivations > 0 && !modes.active(), "Mode session failed");
    const auto at = [](double q) { return [q](double sent) { return std::abs(sent - q) < 1e-12; }; };
    const auto first_long = std::find_if(modes.sent.begin(), modes.sent.end(), at(long_move));
    check(first_long != modes.sent.end() && std::find_if(first_long, modes.sent.end(), at(0.1)) != modes.sent.end(),
        "Switching to position did not hold the measured position");
    check(std::none_of(modes.sent.begin(), modes.sent.end(), at(large)), "Large Position step sent");
    check(std::any_of(modes.sent.begin(), modes.sent.end(), at(small)), "Small Position step not sent");
    check(at(long_move)(modes.sent.back()) && modes.control[0].mode == ControlMode::kMAXMotionPosition &&
        modes.control[0].slot == 0, "MAXMotion move after the mode changes");
    for (const auto *text : {"menos de 0.31831 rot del PV", "Slot 1 en modo position", "Sin cambios: slot 1 has no MAXMotion",
             "Modo maxmotion, slot 0. Mantiene la posicion medida.", "maxmotion slot 0 | habilitado",
             "Sin cambios: slot is not configured", "Formato: mode position o mode maxmotion."})
      check(result.output.find(text) != std::string::npos, (std::string("Console output lacks: ") + text).c_str());
    std::cout << "PASS: nonblocking partial commands, initial hold, motor rotations, on/off, limits, EOF, driver errors, "
        "modes, slots and Position step limit.\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
