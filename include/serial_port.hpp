#pragma once
#include <string>
#include <cstdint>

// POSIX serial port wrapper (8N1, configurable baud rate).
// Replaces the Windows CreateFile/DCB/ReadFile/WriteFile pattern.
class SerialPort {
public:
    SerialPort(const std::string& device, int baud_rate);
    ~SerialPort();

    // Returns true on success. Prints error to stderr on failure.
    bool open();
    void close();

    // Returns number of bytes written, or -1 on error.
    int write(const uint8_t* data, size_t len);

    // Blocking read with timeout_ms. Returns bytes read, 0 on timeout, -1 on error.
    int read(uint8_t* buf, size_t max_len, int timeout_ms);

private:
    std::string device_;
    int baud_rate_;
    int fd_;

    bool configure_termios();
    static int baud_constant(int baud_rate);
};
