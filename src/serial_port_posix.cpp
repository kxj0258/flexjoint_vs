#include "serial_port.hpp"

#ifndef _WIN32

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstdio>
#include <cstring>

SerialPort::SerialPort(const std::string& device, int baud_rate)
    : device_(device), baud_rate_(baud_rate), fd_(-1)
{}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open()
{
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        fprintf(stderr, "SerialPort: cannot open %s: %s\n",
                device_.c_str(), strerror(errno));
        return false;
    }
    if (!configure_termios()) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    flush_input();
    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::write(const uint8_t* data, size_t len)
{
    ssize_t n = ::write(fd_, data, len);
    return static_cast<int>(n);
}

int SerialPort::read(uint8_t* buf, size_t max_len, int timeout_ms)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret < 0) {
        fprintf(stderr, "SerialPort: select error: %s\n", strerror(errno));
        return -1;
    }
    if (ret == 0)
        return 0; // timeout

    ssize_t n = ::read(fd_, buf, max_len);
    return static_cast<int>(n);
}

bool SerialPort::flush_input()
{
    if (fd_ < 0)
        return false;
    if (tcflush(fd_, TCIFLUSH) != 0) {
        fprintf(stderr, "SerialPort: tcflush input failed: %s\n", strerror(errno));
        return false;
    }
    return true;
}

bool SerialPort::flush_io()
{
    if (fd_ < 0)
        return false;
    if (tcflush(fd_, TCIOFLUSH) != 0) {
        fprintf(stderr, "SerialPort: tcflush io failed: %s\n", strerror(errno));
        return false;
    }
    return true;
}

bool SerialPort::configure_termios()
{
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        fprintf(stderr, "SerialPort: tcgetattr failed: %s\n", strerror(errno));
        return false;
    }

    int speed = baud_constant(baud_rate_);
    if (speed < 0) {
        fprintf(stderr, "SerialPort: unsupported baud rate %d\n", baud_rate_);
        return false;
    }
    cfsetispeed(&tty, static_cast<speed_t>(speed));
    cfsetospeed(&tty, static_cast<speed_t>(speed));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 data bits
    tty.c_cflag &= ~PARENB;                     // no parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag |= CREAD | CLOCAL;              // enable receiver, ignore modem lines
    tty.c_cflag &= ~CRTSCTS;                    // no hardware flow control

    tty.c_lflag = 0; // raw input (no echo, no canonical, no signals)
    tty.c_iflag = 0; // no software flow control, no special byte handling
    tty.c_oflag = 0; // raw output

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        fprintf(stderr, "SerialPort: tcsetattr failed: %s\n", strerror(errno));
        return false;
    }
    return true;
}

int SerialPort::baud_constant(int baud_rate)
{
    switch (baud_rate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return -1;
    }
}

#endif
