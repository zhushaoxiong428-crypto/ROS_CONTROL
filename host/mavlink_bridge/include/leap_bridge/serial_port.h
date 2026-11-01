#pragma once

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace leap_bridge {

// Raw 8N1 serial port, opened non-blocking so it can sit in an epoll set.
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    // On failure returns false and leaves the reason in error().
    bool Open(const std::string &path, int baud);
    void Close();

    int fd() const { return fd_; }
    const std::string &error() const { return error_; }

    // Returns the byte count, 0 when nothing is pending, or -1 on a real error.
    ssize_t Read(uint8_t *buf, size_t len);
    // Writes everything or fails; a frame cut in half is worse than a dropped one.
    bool WriteAll(const uint8_t *data, size_t len);

private:
    bool Fail(const std::string &what);

    int fd_ = -1;
    std::string error_;
};

}  // namespace leap_bridge
