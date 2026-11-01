#include "leap_bridge/serial_port.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace leap_bridge {

namespace {

bool BaudToSpeed(int baud, speed_t *speed) {
    switch (baud) {
        case 115200: *speed = B115200; return true;
        case 230400: *speed = B230400; return true;
        case 460800: *speed = B460800; return true;
        case 921600: *speed = B921600; return true;
        default: return false;
    }
}

}  // namespace

SerialPort::~SerialPort() { Close(); }

bool SerialPort::Fail(const std::string &what) {
    error_ = what + ": " + std::strerror(errno);
    Close();
    return false;
}

bool SerialPort::Open(const std::string &path, int baud) {
    Close();

    speed_t speed;
    if (!BaudToSpeed(baud, &speed)) {
        error_ = "unsupported baud rate " + std::to_string(baud);
        return false;
    }

    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        return Fail("open " + path);
    }

    termios tio = {};
    if (tcgetattr(fd_, &tio) != 0) {
        return Fail("tcgetattr");
    }
    // Binary stream: no line editing, no echo, no CR/LF translation.
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(CRTSCTS | CSTOPB);
    // Opening the port must not reset the board when this program exits.
    tio.c_cflag &= ~HUPCL;
    // Reads return whatever is buffered; epoll decides when to read.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        return Fail("tcsetattr");
    }

    // ESP32 boards wire DTR/RTS to EN/IO0 for auto-programming. Release both
    // in one call so the chip keeps running instead of resetting.
    int lines = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd_, TIOCMBIC, &lines);

    // Drop bytes from before we opened (boot log, a half frame).
    tcflush(fd_, TCIOFLUSH);
    error_.clear();
    return true;
}

void SerialPort::Close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

ssize_t SerialPort::Read(uint8_t *buf, size_t len) {
    const ssize_t n = ::read(fd_, buf, len);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        return 0;
    }
    return n;
}

bool SerialPort::WriteAll(const uint8_t *data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        const ssize_t n = ::write(fd_, data + sent, len - sent);
        if (n > 0) {
            sent += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // The kernel TX buffer is full; a short wait is fine at our rates.
            pollfd pfd = {fd_, POLLOUT, 0};
            if (::poll(&pfd, 1, 50) > 0) {
                continue;
            }
        }
        error_ = std::string("write: ") + std::strerror(errno);
        return false;
    }
    return true;
}

}  // namespace leap_bridge
