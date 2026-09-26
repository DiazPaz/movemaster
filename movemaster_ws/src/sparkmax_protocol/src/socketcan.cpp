#include "sparkmax_protocol/socketcan.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace sparkmax_protocol
{

namespace
{
std::string systemError(const std::string & what)
{
  return what + ": " + std::strerror(errno);
}

int toPollTimeout(std::chrono::microseconds timeout)
{
  if (timeout.count() <= 0) {return 0;}
  // Redondeo hacia arriba a milisegundos (resolución de poll()).
  return static_cast<int>((timeout.count() + 999) / 1000);
}
}  // namespace

SocketCanTransport::SocketCanTransport(std::string interface_name, bool receive_error_frames)
: interface_name_(std::move(interface_name)), receive_error_frames_(receive_error_frames)
{
  if (interface_name_.empty() || interface_name_.size() >= IFNAMSIZ) {
    throw std::invalid_argument("Nombre de interfaz CAN inválido: '" + interface_name_ + "'");
  }
}

SocketCanTransport::~SocketCanTransport()
{
  close();
}

void SocketCanTransport::open(const std::vector<CanFilter> & filters)
{
  if (fd_ >= 0) {return;}
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {throw CanTransportError(systemError("socket(PF_CAN)"));}

  auto fail = [fd](const std::string & what) {
      const std::string message = systemError(what);
      ::close(fd);
      throw CanTransportError(message);
    };

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    fail("Interfaz CAN '" + interface_name_ + "' no encontrada (SIOCGIFINDEX)");
  }

  // Filtros de recepción (equivalente a can_filters de python-can).
  if (!filters.empty()) {
    std::vector<struct can_filter> native;
    native.reserve(filters.size());
    for (const auto & filter : filters) {
      struct can_filter f {};
      if (filter.extended) {
        f.can_id = (filter.id & CAN_EFF_MASK) | CAN_EFF_FLAG;
        f.can_mask = (filter.mask & CAN_EFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;
      } else {
        f.can_id = filter.id & CAN_SFF_MASK;
        f.can_mask = (filter.mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;
      }
      native.push_back(f);
    }
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, native.data(),
      static_cast<socklen_t>(native.size() * sizeof(struct can_filter))) < 0)
    {
      fail("setsockopt(CAN_RAW_FILTER)");
    }
  }

  if (receive_error_frames_) {
    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_CRTL | CAN_ERR_BUSOFF |
      CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
      fail("setsockopt(CAN_RAW_ERR_FILTER)");
    }
  }

  // Las tramas propias no se reciben (receive_own_messages=False en python-can).
  const int recv_own = 0;
  ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    fail("fcntl(O_NONBLOCK)");
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    fail("bind(" + interface_name_ + ")");
  }
  fd_ = fd;
}

void SocketCanTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanTransport::send(const CANPacket & packet, std::chrono::microseconds timeout)
{
  if (fd_ < 0) {throw CanTransportError("SocketCAN: el bus no está abierto");}
  if (packet.dlc > CAN_MAX_DLEN) {throw CanTransportError("SocketCAN: DLC > 8");}

  struct can_frame frame {};
  if (packet.is_extended_id) {
    frame.can_id = (packet.arbitration_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  } else {
    frame.can_id = packet.arbitration_id & CAN_SFF_MASK;
  }
  if (packet.is_remote_frame) {frame.can_id |= CAN_RTR_FLAG;}
  frame.can_dlc = packet.dlc;
  if (!packet.is_remote_frame) {std::memcpy(frame.data, packet.data.data(), packet.dlc);}

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const ssize_t written = ::write(fd_, &frame, sizeof(frame));
    if (written == static_cast<ssize_t>(sizeof(frame))) {return true;}
    if (written >= 0) {throw CanTransportError("SocketCAN: escritura incompleta");}
    if (errno == EINTR) {continue;}
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOBUFS) {
      throw CanTransportError(systemError("SocketCAN write"));
    }
    // Buffer de TX lleno: esperar hasta el timeout.
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {return false;}
    struct pollfd pfd {fd_, POLLOUT, 0};
    const int ready = ::poll(&pfd, 1, toPollTimeout(remaining));
    if (ready < 0 && errno != EINTR) {throw CanTransportError(systemError("SocketCAN poll"));}
    if (ready == 0) {return false;}
  }
}

bool SocketCanTransport::receive(CANPacket & packet, std::chrono::microseconds timeout)
{
  if (fd_ < 0) {throw CanTransportError("SocketCAN: el bus no está abierto");}
  struct can_frame frame {};
  ssize_t count = ::read(fd_, &frame, sizeof(frame));
  if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
    struct pollfd pfd {fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, toPollTimeout(timeout));
    if (ready < 0) {
      if (errno == EINTR) {return false;}
      throw CanTransportError(systemError("SocketCAN poll"));
    }
    if (ready == 0) {return false;}
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (pfd.revents & POLLIN) == 0) {
      throw CanTransportError("SocketCAN: la interfaz reportó un error (¿link down?)");
    }
    count = ::read(fd_, &frame, sizeof(frame));
  }
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {return false;}
    throw CanTransportError(systemError("SocketCAN read"));
  }
  if (count != static_cast<ssize_t>(sizeof(frame))) {
    throw CanTransportError("SocketCAN: trama incompleta");
  }

  packet = CANPacket{};
  packet.is_error_frame = (frame.can_id & CAN_ERR_FLAG) != 0;
  packet.is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
  packet.is_remote_frame = (frame.can_id & CAN_RTR_FLAG) != 0;
  packet.arbitration_id = packet.is_error_frame ? (frame.can_id & CAN_ERR_MASK) :
    packet.is_extended_id ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
  packet.dlc = frame.can_dlc > CAN_MAX_DLEN ? CAN_MAX_DLEN : frame.can_dlc;
  std::memcpy(packet.data.data(), frame.data, packet.dlc);
  return true;
}

}  // namespace sparkmax_protocol
