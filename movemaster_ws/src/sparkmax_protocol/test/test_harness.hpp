// Mini arnés de pruebas sin dependencias (no requiere gtest).
#pragma once

#include <cmath>
#include <cstdio>
#include <exception>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace test_harness
{

struct Case
{
  const char * name;
  std::function<void()> body;
};

inline std::vector<Case> & registry()
{
  static std::vector<Case> cases;
  return cases;
}

struct Registrar
{
  Registrar(const char * name, std::function<void()> body)
  {
    registry().push_back({name, std::move(body)});
  }
};

struct Failure : std::exception
{
  explicit Failure(std::string m)
  : message(std::move(m)) {}
  const char * what() const noexcept override {return message.c_str();}
  std::string message;
};

inline int runAll()
{
  int failed = 0;
  for (const auto & c : registry()) {
    try {
      c.body();
      std::printf("[ OK ] %s\n", c.name);
    } catch (const Failure & f) {
      ++failed;
      std::printf("[FAIL] %s\n       %s\n", c.name, f.what());
    } catch (const std::exception & e) {
      ++failed;
      std::printf("[FAIL] %s\n       excepción inesperada: %s\n", c.name, e.what());
    }
  }
  std::printf("%zu pruebas, %d fallidas\n", registry().size(), failed);
  return failed == 0 ? 0 : 1;
}

}  // namespace test_harness

#define TH_CONCAT2(a, b) a ## b
#define TH_CONCAT(a, b) TH_CONCAT2(a, b)

#define TEST_CASE(name) \
  static void TH_CONCAT(test_fn_, __LINE__)(); \
  static test_harness::Registrar TH_CONCAT(test_reg_, __LINE__)(name, TH_CONCAT(test_fn_, __LINE__)); \
  static void TH_CONCAT(test_fn_, __LINE__)()

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::ostringstream th_out; \
      th_out << __FILE__ << ":" << __LINE__ << ": CHECK(" #cond ") falló"; \
      throw test_harness::Failure(th_out.str()); \
    } \
  } while (0)

#define CHECK_EQ(a, b) \
  do { \
    const auto th_a = (a); \
    const auto th_b = (b); \
    if (!(th_a == th_b)) { \
      std::ostringstream th_out; \
      th_out << __FILE__ << ":" << __LINE__ << ": " #a " == " #b " falló (" << th_a << " vs " << \
        th_b << ")"; \
      throw test_harness::Failure(th_out.str()); \
    } \
  } while (0)

#define CHECK_NEAR(a, b, tol) \
  do { \
    const double th_a = (a); \
    const double th_b = (b); \
    if (!(std::fabs(th_a - th_b) <= (tol))) { \
      std::ostringstream th_out; \
      th_out << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b "| <= " #tol " falló (" << \
        th_a << " vs " << th_b << ")"; \
      throw test_harness::Failure(th_out.str()); \
    } \
  } while (0)

#define CHECK_THROWS(expr) \
  do { \
    bool th_thrown = false; \
    try {(void)(expr);} catch (const std::exception &) {th_thrown = true;} \
    if (!th_thrown) { \
      std::ostringstream th_out; \
      th_out << __FILE__ << ":" << __LINE__ << ": se esperaba una excepción de " #expr; \
      throw test_harness::Failure(th_out.str()); \
    } \
  } while (0)

#define TEST_MAIN() \
  int main() {return test_harness::runAll();}
