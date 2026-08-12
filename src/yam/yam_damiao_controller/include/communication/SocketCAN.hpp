#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <cstring>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <array>
#include <stdexcept>
#include <chrono>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <net/if.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>


inline void print_data(const uint8_t* data, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    printf("%02x ", data[i]);
  }
  printf("\n");
}

/**
 * SerialPort now acts as a SocketCAN adapter.
 * - Construct with interface name, e.g. "can0".
 * - send_can(...) sends a CAN frame.
 * - recv_raw(...) receives a struct can_frame.
 */
class SerialPort
{
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(const std::string &ifname, speed_t /*baudrate*/ = 0, int timeout_ms = 200)
    : sockfd_(-1)
  {
    set_timeout(timeout_ms);
    init_can(ifname);
  }

  ~SerialPort()
  {
    if (sockfd_ >= 0) close(sockfd_);
  }

  // Send a raw CAN frame by CAN ID (standard or extended).
  // returns bytes written or -1 on error
  ssize_t send_can(uint32_t can_id, const uint8_t* data, uint8_t len, bool extended = false)
  {
    if (len > 8) return -1;
    if (sockfd_ < 0) return -1;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_dlc = len;
    if (extended) frame.can_id = (can_id | CAN_EFF_FLAG);
    else frame.can_id = can_id & CAN_SFF_MASK;
    std::memcpy(frame.data, data, len);

    ssize_t n = ::write(sockfd_, &frame, sizeof(frame));
    if (n != (ssize_t)sizeof(frame)) {
      if (n < 0) perror("write(can)");
    }
    return n;
  }

  // Receive a raw CAN frame (blocking with timeout). Returns:
  // >0 : number of bytes read (sizeof(can_frame))
  //  0 : timeout
  // -1 : error
  ssize_t recv_raw(struct can_frame &frame)
  {
    if (sockfd_ < 0) return -1;

    FD_ZERO(&rSet_);
    FD_SET(sockfd_, &rSet_);
    int ret = select(sockfd_ + 1, &rSet_, nullptr, nullptr, &timeout_);
    if (ret < 0) {
      perror("select");
      return -1;
    } else if (ret == 0) {
      return 0; // timeout
    } else {
      ssize_t n = ::read(sockfd_, &frame, sizeof(frame));
      if (n < 0) perror("read(can)");
      return n;
    }
  }

  // Optional helpers kept from the old API (not used by damiao after conversion)
  ssize_t send(const uint8_t* data, size_t len)
  {
    // Not a general-purpose raw-write to socket; prefer send_can.
    // Keep this for backward compatibility: attempt to write bytes directly.
    if (sockfd_ < 0) return -1;
    ssize_t ret = ::write(sockfd_, data, len);
    return ret;
  }

  // Not used in new flow; kept as stub to avoid build errors
  ssize_t recv(uint8_t* data, size_t len)
  {
    // try to read a single can_frame and copy its bytes into data if possible
    struct can_frame frame;
    ssize_t n = recv_raw(frame);
    if (n <= 0) return n;
    size_t copy_len = std::min(len, (size_t)sizeof(frame));
    std::memcpy(data, &frame, copy_len);
    return (ssize_t)copy_len;
  }

  void set_timeout(int timeout_ms)
  {
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
  }

private:
  int sockfd_;
  fd_set rSet_;
  timeval timeout_;

  void init_can(const std::string &ifname)
  {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ < 0) {
      perror("socket(PF_CAN)");
      throw std::runtime_error("Failed to create CAN socket");
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ-1);
    if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
      perror("ioctl(SIOCGIFINDEX)");
      close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error("ioctl SIOCGIFINDEX failed");
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("bind(can)");
      close(sockfd_);
      sockfd_ = -1;
      throw std::runtime_error("bind failed");
    }

    // socket is ready
  }
};

#endif // SERIAL_PORT_H
