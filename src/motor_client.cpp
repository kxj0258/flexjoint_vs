#include "motor_client.hpp"

#include "modbus_crc.hpp"
#include "serial_port.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

constexpr double kPi = 3.14159265358979323846;

void append_i16_le(std::vector<uint8_t>& out, int value)
{
    const int16_t v = static_cast<int16_t>(value);
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void append_i32_le(std::vector<uint8_t>& out, int value)
{
    const int32_t v = static_cast<int32_t>(value);
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

bool valid_command(int command)
{
    return command >= 0 && command <= 0xFF;
}

} // namespace

std::vector<uint8_t> build_motor_frame(uint8_t command,
                                       const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> frame;
    frame.reserve(7 + payload.size());
    frame.push_back(0x3E);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(command);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());

    uint16_t crc = crc16_modbus(frame.data(), static_cast<uint32_t>(frame.size()));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

MotorClient::MotorClient(SerialPort& port, const MotorProtocolConfig& cfg)
    : port_(port), cfg_(cfg)
{}

bool MotorClient::send_velocity_rad_s(double velocity_rad_s)
{
    if (!valid_command(cfg_.velocity_command)) {
        fprintf(stderr, "MotorClient: invalid velocity command 0x%X\n",
                cfg_.velocity_command);
        return false;
    }

    const double vel_rpm10 = velocity_rad_s / (2.0 * kPi) * 60.0 * 10.0;
    uint8_t data_low = 0;
    uint8_t data_high = 0;
    velocity_to_bytes(static_cast<float>(vel_rpm10), data_low, data_high);

    return send_frame(static_cast<uint8_t>(cfg_.velocity_command),
                      {data_low, data_high});
}

bool MotorClient::send_position_rad(double position_rad)
{
    if (!valid_command(cfg_.position_command)) {
        fprintf(stderr,
                "MotorClient: position command is not configured. "
                "Set motor_protocol.position_command or use 'frame/raw'.\n");
        return false;
    }

    const double counts = position_rad / (2.0 * kPi) * cfg_.position_counts_per_rev;
    std::vector<uint8_t> payload;
    append_i32_le(payload, static_cast<int>(std::lround(counts)));
    return send_frame(static_cast<uint8_t>(cfg_.position_command), payload);
}

bool MotorClient::send_pid(double kp, double ki, double kd)
{
    if (!valid_command(cfg_.pid_command)) {
        fprintf(stderr,
                "MotorClient: PID command is not configured. "
                "Set motor_protocol.pid_command or use 'frame/raw'.\n");
        return false;
    }

    std::vector<uint8_t> payload;
    append_i16_le(payload, static_cast<int>(std::lround(kp * cfg_.pid_scale)));
    append_i16_le(payload, static_cast<int>(std::lround(ki * cfg_.pid_scale)));
    append_i16_le(payload, static_cast<int>(std::lround(kd * cfg_.pid_scale)));
    return send_frame(static_cast<uint8_t>(cfg_.pid_command), payload);
}

bool MotorClient::send_frame(uint8_t command, const std::vector<uint8_t>& payload)
{
    const std::vector<uint8_t> frame = build_motor_frame(command, payload);
    return send_raw(frame);
}

bool MotorClient::send_raw(const std::vector<uint8_t>& bytes)
{
    if (bytes.empty())
        return false;
    const int n = port_.write(bytes.data(), bytes.size());
    if (n != static_cast<int>(bytes.size())) {
        fprintf(stderr, "MotorClient: write failed (%d/%zu bytes)\n",
                n, bytes.size());
        return false;
    }
    return true;
}

bool MotorClient::read_feedback(double encoder_zero_deg, MotorFeedback& feedback,
                                int timeout_ms, bool quiet)
{
    uint8_t buf[15] = {};
    int total = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (total < 15) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
            break;
        const int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        const int n = port_.read(buf + total, static_cast<size_t>(15 - total),
                                 std::max(1, remain_ms));
        if (n < 0)
            return false;
        if (n == 0)
            continue;
        total += n;
    }

    if (total < 15) {
        if (quiet)
            return false;
        fprintf(stderr, "MotorClient: feedback short read (%d bytes)\n", total);
        return false;
    }

    const int raw_angle = 256 * static_cast<int>(buf[6]) + static_cast<int>(buf[5]);
    const int raw_vel_unsigned = 256 * static_cast<int>(buf[12]) + static_cast<int>(buf[11]);
    const int raw_vel = (raw_vel_unsigned <= 32767)
                        ? raw_vel_unsigned
                        : -(65536 - raw_vel_unsigned);

    const double absolute_deg = static_cast<double>(raw_angle) * 360.0 / 16384.0;
    feedback.raw_angle_count       = raw_angle;
    feedback.raw_velocity_count    = raw_vel;
    feedback.absolute_position_deg = absolute_deg;
    feedback.absolute_position_rad = absolute_deg * kPi / 180.0;
    feedback.joint_position_rad    = (absolute_deg - encoder_zero_deg) * kPi / 180.0;
    feedback.velocity_rad_s        = static_cast<double>(raw_vel) * 0.1 * 2.0 * kPi / 60.0;
    return true;
}
