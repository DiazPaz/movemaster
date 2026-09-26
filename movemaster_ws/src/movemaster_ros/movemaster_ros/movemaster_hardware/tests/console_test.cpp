#define MOVEMASTER_CONSOLE_NO_MAIN
#include "../examples/maxmotion_console.cpp"
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
  std::vector<double> sent;
  void read() {}
  void activate() { enabled = true; ++activations; }
  void deactivate() noexcept { enabled = false; ++deactivations; }
  bool active() const { return enabled; }
  const std::vector<JointState> &states() const { return state; }
  void write(const std::vector<double> &target) {
    if (fail_write && sent.size() == 3) throw std::runtime_error("Injected driver/watchdog error");
    check(enabled, "Console wrote while inactive");
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
  maxmotion_console::stop_requested = 0;
  try { maxmotion_console::run(driver, config(), fds[0], output); }
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
    std::cout << "PASS: nonblocking partial commands, initial hold, motor rotations, on/off, limits, EOF and driver errors.\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
