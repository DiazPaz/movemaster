// Bus CAN simulado con N SPARK MAX para probar MoveMasterDriver sin hardware.
//
// Igual que test_backend.py, el simulador usa IDs y bytes de referencia escritos
// a mano (no el codec de sparkmax_protocol) para detectar errores de orden,
// escala y direccionamiento. No simula la física: la posición sigue al setpoint.

#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "sparkmax_protocol/socketcan.hpp"

class FakeSparkBus : public sparkmax_protocol::CanTransport
{
public:
  using CANPacket = sparkmax_protocol::CANPacket;

  static constexpr uint32_t kParameterWrite = 0x02053800;
  static constexpr uint32_t kParameterWriteResponse = 0x02053840;
  static constexpr uint32_t kStopFollower = 0x02057C80;
  static constexpr uint32_t kStopFollowerResponse = 0x02057CC0;
  static constexpr uint32_t kSetStatuses = 0x02050400;
  static constexpr uint32_t kSetStatusesResponse = 0x02050440;
  static constexpr uint32_t kPersist = 0x0205FFC0;
  static constexpr uint32_t kPersistResponse = 0x02050500;
  static constexpr uint32_t kPositionSetpoint = 0x02050100;
  static constexpr uint32_t kStatus0 = 0x0205B800;
  static constexpr uint32_t kStatus1 = 0x0205B840;
  static constexpr uint32_t kStatus2 = 0x0205B880;
  static constexpr uint32_t kHeartbeat = 0x01011840;

  struct Device
  {
    float position{0.0f};     // rotaciones del motor
    float velocity{0.0f};     // RPM
    uint16_t enabled_statuses{0b011};   // STATUS_0 y STATUS_1 habilitados por defecto
    std::map<int, uint32_t> parameters;
    int setpoints{0};
    int last_slot{-1};
  };

  explicit FakeSparkBus(std::vector<uint8_t> device_ids)
  {
    for (auto id : device_ids) {devices_[id] = Device{};}
  }

  // ---- inyección de fallos -------------------------------------------------
  std::atomic<int> reject_parameter_id{-1};   // responde RESULT_CODE = 1 (Invalid ID)
  std::atomic<bool> no_ack{false};
  std::atomic<bool> wrong_value{false};
  std::atomic<bool> mute_status{false};
  std::atomic<uint8_t> persist_result{0};

  // ---- inspección ------------------------------------------------------------
  std::vector<CANPacket> sent()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent_;
  }
  std::vector<CANPacket> sentWithBase(uint32_t base)
  {
    std::vector<CANPacket> out;
    for (const auto & p : sent()) {
      if ((p.arbitration_id & ~0x3Fu) == base) {out.push_back(p);}
    }
    return out;
  }
  int heartbeats()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    int n = 0;
    for (const auto & p : sent_) {n += p.arbitration_id == kHeartbeat ? 1 : 0;}
    return n;
  }
  Device device(uint8_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return devices_.at(id);
  }
  void setPosition(uint8_t id, float rotations)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    devices_.at(id).position = rotations;
  }
  void injectErrorFrame()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    CANPacket p;
    p.is_error_frame = true;
    p.arbitration_id = 0x40;  // CAN_ERR_BUSOFF
    queue_.push_back(p);
  }

  // ---- CanTransport ------------------------------------------------------------
  void open(const std::vector<sparkmax_protocol::CanFilter> & filters) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    filters_ = filters;
    open_ = true;
  }
  void close() override {open_ = false;}
  bool isOpen() const override {return open_;}
  std::string name() const override {return "fake";}

  bool send(const CANPacket & p, std::chrono::microseconds) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sent_.push_back(p);
    const uint32_t base = p.arbitration_id & ~0x3Fu;
    const uint8_t id = p.arbitration_id & 0x3F;
    if (p.arbitration_id == kHeartbeat) {return true;}
    auto it = devices_.find(id);
    if (it == devices_.end()) {return true;}
    Device & dev = it->second;

    if (base == kParameterWrite && !no_ack) {
      uint32_t value = 0;
      std::memcpy(&value, &p.data[1], 4);
      const int pid = p.data[0];
      dev.parameters[pid] = value;
      reply(kParameterWriteResponse | id, {
          p.data[0], typeOf(pid),
          byteOf(wrong_value ? value + 1 : value, 0), byteOf(value, 1), byteOf(value, 2),
          byteOf(value, 3), static_cast<uint8_t>(pid == reject_parameter_id ? 1 : 0)});
    } else if (base == kStopFollower && !no_ack) {
      reply(kStopFollowerResponse | id, {});
    } else if (base == kSetStatuses && !no_ack) {
      const uint16_t mask = p.data[0] | (p.data[1] << 8);
      const uint16_t bits = p.data[2] | (p.data[3] << 8);
      dev.enabled_statuses = static_cast<uint16_t>((dev.enabled_statuses & ~mask) | (bits & mask));
      reply(kSetStatusesResponse | id, {0, p.data[0], p.data[1],
          static_cast<uint8_t>(dev.enabled_statuses & 0xFF),
          static_cast<uint8_t>(dev.enabled_statuses >> 8)});
    } else if (base == kPersist && !no_ack) {
      reply(kPersistResponse | id, {persist_result.load()});
    } else if (base == kPositionSetpoint) {
      float sp = 0.0f;
      std::memcpy(&sp, &p.data[0], 4);
      dev.position = sp;  // seguimiento ideal
      dev.setpoints++;
      dev.last_slot = p.data[6] & 0x03;
    }
    return true;
  }

  bool receive(CANPacket & packet, std::chrono::microseconds timeout) override
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        generateStatus();
        if (!queue_.empty()) {
          packet = queue_.front();
          queue_.pop_front();
          return true;
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {return false;}
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }

private:
  static uint8_t byteOf(uint32_t v, int i) {return static_cast<uint8_t>((v >> (8 * i)) & 0xFF);}

  static uint8_t typeOf(int pid)
  {
    switch (pid) {
      case 9: case 158: case 160: return 2;   // uint
      case 149: return 4;                     // bool
      default: return 3;                      // float
    }
  }

  void reply(uint32_t id, std::initializer_list<uint8_t> bytes)
  {
    CANPacket p;
    p.arbitration_id = id;
    p.dlc = static_cast<uint8_t>(bytes.size());
    std::size_t i = 0;
    for (auto b : bytes) {p.data[i++] = b;}
    queue_.push_back(p);
  }

  void generateStatus()
  {
    const auto now = std::chrono::steady_clock::now();
    if (mute_status || now - last_status_ < std::chrono::milliseconds(10)) {return;}
    last_status_ = now;
    for (auto & [id, dev] : devices_) {
      if (dev.enabled_statuses & 0b001) {
        // STATUS_0: corriente = 55 * 0.03663 A ≈ 2.01 A (bits 28..39), 12.0 V aprox.
        uint64_t raw = 0;
        raw |= uint64_t{1638} << 16;   // VOLTAGE ≈ 12.0 V
        raw |= uint64_t{55} << 28;     // CURRENT
        raw |= uint64_t{30} << 40;     // MOTOR_TEMPERATURE
        CANPacket p;
        p.arbitration_id = kStatus0 | id;
        p.dlc = 8;
        for (int i = 0; i < 8; ++i) {p.data[i] = static_cast<uint8_t>(raw >> (8 * i));}
        queue_.push_back(p);
      }
      if (dev.enabled_statuses & 0b100) {
        CANPacket p;
        p.arbitration_id = kStatus2 | id;
        p.dlc = 8;
        std::memcpy(&p.data[0], &dev.velocity, 4);
        std::memcpy(&p.data[4], &dev.position, 4);
        queue_.push_back(p);
      }
    }
  }

  std::mutex mutex_;
  std::map<uint8_t, Device> devices_;
  std::deque<CANPacket> queue_;
  std::vector<CANPacket> sent_;
  std::vector<sparkmax_protocol::CanFilter> filters_;
  std::chrono::steady_clock::time_point last_status_{};
  std::atomic<bool> open_{false};
};
