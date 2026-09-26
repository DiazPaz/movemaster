#include "movemaster_hardware/socketcan.hpp"

#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <system_error>

namespace movemaster {
namespace {
[[noreturn]] void fail(const char *what) { throw std::system_error(errno, std::generic_category(), what); }
}
SocketCAN::SocketCAN(const std::string &channel) {
  const auto index = if_nametoindex(channel.c_str());
  if (index == 0) fail("if_nametoindex");
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd_ < 0) fail("socket(PF_CAN)");
  try {
    const int own = 0;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof(own)) < 0)
      fail("CAN_RAW_RECV_OWN_MSGS");
    const can_err_mask_t errors = CAN_ERR_MASK;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0)
      fail("CAN_RAW_ERR_FILTER");
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);
    if (::bind(fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0)
      fail("bind(SocketCAN)");
  } catch (...) { ::close(fd_); fd_ = -1; throw; }
}
SocketCAN::~SocketCAN() { if (fd_ >= 0) ::close(fd_); }
void SocketCAN::set_filters(const std::vector<std::uint32_t> &ids) {
  std::vector<can_filter> filters;
  for (auto id : ids) {
    if (id > CAN_EFF_MASK) throw std::invalid_argument("Invalid CAN filter ID");
    filters.push_back({id | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG});
  }
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      static_cast<socklen_t>(filters.size() * sizeof(can_filter))) < 0) fail("CAN_RAW_FILTER");
}
bool SocketCAN::ready(short events, double timeout) const {
  if (!std::isfinite(timeout) || timeout < 0 || timeout > 3600)
    throw std::invalid_argument("SocketCAN timeout must be in 0..3600 seconds");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  for (;;) {
    const auto left = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    const int ms = static_cast<int>(std::ceil(std::max(0.0, left) * 1000));
    pollfd pfd{fd_, events, 0};
    const int result = ::poll(&pfd, 1, ms);
    if (result < 0) {
      if (errno == EINTR && std::chrono::steady_clock::now() < deadline) continue;
      if (errno == EINTR) return false;
      fail("poll(SocketCAN)");
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
      throw std::runtime_error("SocketCAN unavailable or link error");
    return result > 0 && (pfd.revents & events);
  }
}
void SocketCAN::send(const CANPacket &packet, double timeout) {
  const auto frame = packet.to_socketcan();
  // Runtime callers pass zero: never wait for a congested bus in read()/write().
  if (!ready(POLLOUT, timeout)) throw TimeoutError("SocketCAN TX timeout");
  const auto n = ::write(fd_, &frame, sizeof(frame));
  if (n < 0) fail("write(SocketCAN)");
  if (n != sizeof(frame)) throw std::runtime_error("Short SocketCAN write");
}
std::optional<CANPacket> SocketCAN::recv(double timeout) {
  if (!ready(POLLIN, timeout)) return std::nullopt;
  can_frame frame{};
  const auto n = ::read(fd_, &frame, sizeof(frame));
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return std::nullopt;
    fail("read(SocketCAN)");
  }
  if (n != sizeof(frame)) throw std::runtime_error("Short SocketCAN read");
  return CANPacket::from_socketcan(frame);
}
}  // namespace movemaster
