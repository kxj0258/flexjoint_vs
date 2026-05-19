#include "motor_client.hpp"

#include "modbus_crc.hpp"
#include "serial_port.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint8_t kFrameHeaderHost  = 0x3E;
constexpr uint8_t kFrameHeaderMotor = 0x3C;
constexpr uint8_t kDefaultSequence  = 0x00;
constexpr uint8_t kDefaultAddress   = 0x01;
constexpr uint8_t kCmdReadSystemStatus = 0x0B;
constexpr int kLegacyFeedbackLength = 15;
constexpr int kSystemStatusLength   = 20;

void append_i16_le(std::vector<uint8_t>& out, int value)
{
    const int16_t v = static_cast<int16_t>(value);
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void append_u32_le(std::vector<uint8_t>& out, uint32_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
}

uint16_t read_u16_le(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

int16_t read_i16_le(const uint8_t* data)
{
    return static_cast<int16_t>(read_u16_le(data));
}

uint32_t read_u32_le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

int32_t read_i32_le(const uint8_t* data)
{
    return static_cast<int32_t>(read_u32_le(data));
}

bool valid_command(int command)
{
    return command >= 0 && command <= 0xFF;
}

bool read_exact(SerialPort& port, uint8_t* buf, int expected_len, int timeout_ms)
{
    int total = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (total < expected_len) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
            break;
        const int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        const int n = port.read(buf + total, static_cast<size_t>(expected_len - total),
                                std::max(1, remain_ms));
        if (n < 0)
            return false;
        if (n == 0)
            continue;
        total += n;
    }

    return total == expected_len;
}

bool read_frame_synced(SerialPort& port, uint8_t* buf, int expected_len, int timeout_ms)
{
    if (expected_len <= 0)
        return false;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
            return false;
        const int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        const int n = port.read(buf, 1, std::max(1, remain_ms));
        if (n < 0)
            return false;
        if (n == 0)
            continue;
        if (buf[0] == kFrameHeaderMotor)
            break;
    }

    return read_exact(port, buf + 1, expected_len - 1, timeout_ms);
}

void fill_common_feedback(double encoder_zero_deg, uint16_t single_turn_count,
                          int32_t multi_turn_count, int16_t speed_rpm10,
                          MotorFeedback& feedback)
{
    const double single_turn_deg =
        static_cast<double>(single_turn_count) * 360.0 / 16384.0;
    const double multi_turn_deg =
        static_cast<double>(multi_turn_count) * 360.0 / 16384.0;

    feedback.raw_angle_count         = static_cast<int>(single_turn_count);
    feedback.raw_multi_turn_count    = multi_turn_count;
    feedback.raw_velocity_count      = static_cast<int>(speed_rpm10);
    feedback.absolute_position_deg   = single_turn_deg;
    feedback.absolute_position_rad   = single_turn_deg * kPi / 180.0;
    feedback.multi_turn_position_deg = multi_turn_deg;
    feedback.multi_turn_position_rad = multi_turn_deg * kPi / 180.0;
    feedback.joint_position_rad      = (single_turn_deg - encoder_zero_deg) * kPi / 180.0;
    feedback.velocity_rad_s =
        static_cast<double>(speed_rpm10) * 0.1 * 2.0 * kPi / 60.0;
}

bool validate_response_frame(const uint8_t* buf, int frame_len, uint8_t expected_command,
                             uint8_t expected_payload_len, bool quiet, const char* tag)
{
    if (buf[0] != kFrameHeaderMotor) {
        if (!quiet)
            fprintf(stderr, "MotorClient: %s invalid header 0x%02X\n", tag, buf[0]);
        return false;
    }
    if (buf[3] != expected_command) {
        if (!quiet)
            fprintf(stderr, "MotorClient: %s invalid command 0x%02X\n", tag, buf[3]);
        return false;
    }
    if (buf[4] != expected_payload_len) {
        if (!quiet)
            fprintf(stderr, "MotorClient: %s invalid payload length %u\n",
                    tag, static_cast<unsigned>(buf[4]));
        return false;
    }

    const uint16_t crc_calc = crc16_modbus(buf, static_cast<uint32_t>(frame_len - 2));
    const uint16_t crc_recv = read_u16_le(buf + frame_len - 2);
    if (crc_calc != crc_recv) {
        if (!quiet) {
            fprintf(stderr,
                    "MotorClient: %s CRC mismatch calc=0x%04X recv=0x%04X\n",
                    tag, crc_calc, crc_recv);
        }
        return false;
    }
    return true;
}

} // namespace

std::vector<uint8_t> build_motor_frame(uint8_t command,
                                       const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> frame;
    frame.reserve(7 + payload.size());
    frame.push_back(kFrameHeaderHost);
    frame.push_back(kDefaultSequence);
    frame.push_back(kDefaultAddress);
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

bool MotorClient::send_position_counts(uint32_t position_counts)
{
    if (!valid_command(cfg_.position_command)) {
        fprintf(stderr,
                "MotorClient: position command is not configured. "
                "Set motor_protocol.position_command or use 'frame/raw'.\n");
        return false;
    }

    std::vector<uint8_t> payload;
    payload.reserve(4);
    append_u32_le(payload, position_counts);
    return send_frame(static_cast<uint8_t>(cfg_.position_command), payload);
}

bool MotorClient::send_position_rad(double position_rad)
{
    const double counts = position_rad / (2.0 * kPi) * cfg_.position_counts_per_rev;
    if (counts < 0.0) {
        fprintf(stderr,
                "MotorClient: absolute position count must be non-negative, got %.3f\n",
                counts);
        return false;
    }
    return send_position_counts(static_cast<uint32_t>(std::llround(counts)));
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

bool MotorClient::read_system_status(double encoder_zero_deg, MotorFeedback& feedback,
                                     int timeout_ms, bool quiet)
{
    const std::vector<uint8_t> request = build_motor_frame(kCmdReadSystemStatus, {});
    if (!send_raw(request))
        return false;

    uint8_t buf[kSystemStatusLength] = {};
    if (!read_frame_synced(port_, buf, kSystemStatusLength, timeout_ms)) {
        if (!quiet)
            fprintf(stderr, "MotorClient: system status short read\n");
        return false;
    }

    if (!validate_response_frame(buf, kSystemStatusLength, kCmdReadSystemStatus, 0x0D,
                                 quiet, "system status"))
        return false;

    fill_common_feedback(encoder_zero_deg,
                         read_u16_le(buf + 5),
                         read_i32_le(buf + 7),
                         read_i16_le(buf + 11),
                         feedback);
    feedback.bus_voltage_v       = static_cast<double>(buf[13]) * 0.2;
    feedback.phase_current_a     = static_cast<double>(buf[14]) * 0.03;
    feedback.temperature_c       = static_cast<double>(buf[15]) * 0.4;
    feedback.fault_code          = buf[16];
    feedback.run_state           = buf[17];
    feedback.system_status_valid = true;
    return true;
}

bool MotorClient::read_feedback(double encoder_zero_deg, MotorFeedback& feedback,
                                int timeout_ms, bool quiet)
{
    uint8_t buf[kLegacyFeedbackLength] = {};
    if (!read_frame_synced(port_, buf, kLegacyFeedbackLength, timeout_ms)) {
        if (quiet)
            return false;
        fprintf(stderr, "MotorClient: feedback short read\n");
        return false;
    }

    if (!validate_response_frame(buf, kLegacyFeedbackLength, 0x2F, 0x08,
                                 quiet, "legacy feedback")) {
        return false;
    }

    fill_common_feedback(encoder_zero_deg,
                         read_u16_le(buf + 5),
                         read_i32_le(buf + 7),
                         read_i16_le(buf + 11),
                         feedback);
    feedback.bus_voltage_v       = 0.0;
    feedback.phase_current_a     = 0.0;
    feedback.temperature_c       = 0.0;
    feedback.fault_code          = 0;
    feedback.run_state           = 0;
    feedback.system_status_valid = false;
    return true;
}
