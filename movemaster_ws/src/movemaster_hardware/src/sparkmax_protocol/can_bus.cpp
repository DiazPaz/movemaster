#include "sparkmax_protocol/can_bus.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace sparkmax
{

namespace
{

using Clock = std::chrono::steady_clock;

[[noreturn]] void throw_errno(const std::string & what)
{
  throw std::system_error(errno, std::generic_category(), what);
}

/// Espera un evento del socket durante timeout_s (resolución sub-milisegundo).
bool wait_for(int fd, short events, double timeout_s)
{
  pollfd pfd{fd, events, 0};
  timespec ts{};
  if (timeout_s > 0.0) {
    ts.tv_sec = static_cast<time_t>(timeout_s);
    ts.tv_nsec = static_cast<long>((timeout_s - std::floor(timeout_s)) * 1e9);
  }
  while (true) {
    const int ready = ppoll(&pfd, 1, &ts, nullptr);
    if (ready >= 0) {return ready > 0;}
    if (errno != EINTR) {throw_errno("SocketCAN poll");}
  }
}

}  // namespace

SocketCanBus::SocketCanBus(const std::string & channel, const std::vector<CanFilter> & filters)
: channel_(channel)
{
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd_ < 0) {throw_errno("SocketCAN socket");}

  try {
    ifreq ifr{};
    if (channel.size() >= IFNAMSIZ) {throw std::invalid_argument("Nombre de interfaz CAN largo");}
    std::strncpy(ifr.ifr_name, channel.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {throw_errno("Interfaz CAN '" + channel + "'");}

    // Igual que python-can: filtros por ID exacto y recepción de error frames.
    if (!filters.empty()) {
      std::vector<can_filter> raw;
      raw.reserve(filters.size());
      for (const auto & f : filters) {
        can_filter cf{f.can_id, f.can_mask};
        if (f.extended) {
          cf.can_id |= CAN_EFF_FLAG;
          cf.can_mask |= CAN_EFF_FLAG;
        }
        raw.push_back(cf);
      }
      if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, raw.data(),
          static_cast<socklen_t>(raw.size() * sizeof(can_filter))) < 0)
      {
        throw_errno("CAN_RAW_FILTER");
      }
    }
    const can_err_mask_t err_mask = CAN_ERR_MASK;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
      throw_errno("CAN_RAW_ERR_FILTER");
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw_errno("SocketCAN bind '" + channel + "'");
    }
  } catch (...) {
    shutdown();
    throw;
  }
}

SocketCanBus::~SocketCanBus() {shutdown();}

void SocketCanBus::shutdown()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SocketCanBus::send(const CANPacket & packet, double timeout_s)
{
  if (fd_ < 0) {throw std::runtime_error("SocketCAN cerrado");}
  const can_frame frame = packet.to_can_frame();
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(timeout_s));

  while (true) {
    const auto written = ::write(fd_, &frame, sizeof(frame));
    if (written == static_cast<ssize_t>(sizeof(frame))) {return;}
    if (written < 0 && errno != EAGAIN && errno != ENOBUFS && errno != EINTR) {
      throw_errno("SocketCAN send");
    }
    const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
    if (remaining <= 0.0) {
      throw std::runtime_error("SocketCAN send: buffer de transmisión lleno (timeout)");
    }
    // ENOBUFS no despierta POLLOUT de forma fiable: espera acotada y reintenta.
    wait_for(fd_, POLLOUT, std::min(remaining, 0.0005));
  }
}

std::optional<CANPacket> SocketCanBus::recv(double timeout_s)
{
  if (fd_ < 0) {throw std::runtime_error("SocketCAN cerrado");}
  while (true) {
    can_frame frame{};
    const auto n = ::read(fd_, &frame, sizeof(frame));
    if (n == static_cast<ssize_t>(sizeof(frame))) {return CANPacket::from_can_frame(frame);}
    if (n < 0 && errno != EAGAIN && errno != EINTR) {throw_errno("SocketCAN recv");}
    if (n >= 0) {throw std::runtime_error("SocketCAN recv: trama incompleta");}
    if (timeout_s <= 0.0 || !wait_for(fd_, POLLIN, timeout_s)) {return std::nullopt;}
    timeout_s = 0.0;  // ya esperamos; una sola lectura más
  }
}

}  // namespace sparkmax
