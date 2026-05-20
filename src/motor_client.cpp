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
constexpr uint8_t kCmdReadLegacyFeedback = 0x2F;
constexpr uint8_t kSystemStatusPayloadLen = 0x0D;
constexpr uint8_t kLegacyFeedbackPayloadLen = 0x08;
constexpr int kMaxPayloadLen = 60;

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

double wrap_degrees_180(double degrees)
{
    double wrapped = std::fmod(degrees + 180.0, 360.0);
    if (wrapped < 0.0)
        wrapped += 360.0;
    return wrapped - 180.0;
}

int remaining_ms(const std::chrono::steady_clock::time_point& deadline)
{
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
        return 0;
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
}

bool read_exact_until(SerialPort& port, uint8_t* buf, int expected_len,
                      const std::chrono::steady_clock::time_point& deadline)
{
    int total = 0;

    while (total < expected_len) {
        const int remain_ms = remaining_ms(deadline);
        if (remain_ms <= 0)
            break;
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

bool read_motor_frame_until(SerialPort& port, MotorFrame& frame,
                            const std::chrono::steady_clock::time_point& deadline)
{
    frame = MotorFrame{};

    while (true) {
        const int remain_ms = remaining_ms(deadline);
        if (remain_ms <= 0)
            return false;
        uint8_t header = 0;
        const int n = port.read(&header, 1, std::max(1, remain_ms));
        if (n < 0)
            return false;
        if (n == 0)
            continue;
        if (header == kFrameHeaderMotor) {
            frame.header = header;
            break;
        }
    }

    uint8_t fixed[4] = {};
    if (!read_exact_until(port, fixed, 4, deadline))
        return false;

    frame.sequence = fixed[0];
    frame.address = fixed[1];
    frame.command = fixed[2];
    frame.payload_len = fixed[3];
    if (frame.payload_len > kMaxPayloadLen)
        return true;

    frame.payload.resize(frame.payload_len);
    if (frame.payload_len > 0 &&
        !read_exact_until(port, frame.payload.data(), frame.payload_len, deadline)) {
        return false;
    }

    uint8_t crc_buf[2] = {};
    if (!read_exact_until(port, crc_buf, 2, deadline))
        return false;

    frame.crc_received = read_u16_le(crc_buf);

    std::vector<uint8_t> crc_data;
    crc_data.reserve(5 + frame.payload.size());
    crc_data.push_back(frame.header);
    crc_data.push_back(frame.sequence);
    crc_data.push_back(frame.address);
    crc_data.push_back(frame.command);
    crc_data.push_back(frame.payload_len);
    crc_data.insert(crc_data.end(), frame.payload.begin(), frame.payload.end());
    frame.crc_calculated =
        crc16_modbus(crc_data.data(), static_cast<uint32_t>(crc_data.size()));
    frame.crc_ok = (frame.crc_calculated == frame.crc_received);
    return true;
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
    feedback.joint_position_rad      =
        wrap_degrees_180(single_turn_deg - encoder_zero_deg) * kPi / 180.0;
    feedback.velocity_rad_s =
        static_cast<double>(speed_rpm10) * 0.1 * 2.0 * kPi / 60.0;
}

bool validate_payload_len(const MotorFrame& frame, uint8_t expected_payload_len,
                          bool quiet, const char* tag)
{
    if (frame.payload_len != expected_payload_len) {
        if (!quiet)
            fprintf(stderr, "MotorClient: %s invalid payload length %u\n",
                    tag, static_cast<unsigned>(frame.payload_len));
        return false;
    }
    return true;
}

void parse_encoder_feedback(double encoder_zero_deg, const MotorFrame& frame,
                            MotorFeedback& feedback)
{
    const uint8_t* data = frame.payload.data();
    fill_common_feedback(encoder_zero_deg,
                         read_u16_le(data),
                         read_i32_le(data + 2),
                         read_i16_le(data + 6),
                         feedback);
}

} // namespace

std::vector<uint8_t> build_motor_frame(uint8_t command,
                                       const std::vector<uint8_t>& payload)
{
    return build_motor_frame(command, payload, kDefaultSequence, kDefaultAddress);
}

std::vector<uint8_t> build_motor_frame(uint8_t command,
                                       const std::vector<uint8_t>& payload,
                                       uint8_t sequence,
                                       uint8_t address)
{
    std::vector<uint8_t> frame;
    frame.reserve(7 + payload.size());
    frame.push_back(kFrameHeaderHost);
    frame.push_back(sequence);
    frame.push_back(address);
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

    const uint8_t command = static_cast<uint8_t>(cfg_.velocity_command);
    const bool ok = send_frame(command, {data_low, data_high});
    if (ok && cfg_.consume_command_response)
        consume_response(command, 5, true);
    return ok;
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
    const uint8_t command = static_cast<uint8_t>(cfg_.position_command);
    const bool ok = send_frame(command, payload);
    if (ok && cfg_.consume_command_response)
        consume_response(command, 5, true);
    return ok;
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
    const std::vector<uint8_t> frame =
        build_motor_frame(command, payload, cfg_.sequence, cfg_.address);
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
    const std::vector<uint8_t> request =
        build_motor_frame(kCmdReadSystemStatus, {}, cfg_.sequence, cfg_.address);
    if (!send_raw(request))
        return false;

    MotorFrame frame;
    if (!read_frame(kCmdReadSystemStatus, frame, timeout_ms, quiet))
        return false;
    if (!validate_payload_len(frame, kSystemStatusPayloadLen, quiet, "system status"))
        return false;

    parse_encoder_feedback(encoder_zero_deg, frame, feedback);
    feedback.bus_voltage_v       = static_cast<double>(frame.payload[8]) * 0.2;
    feedback.phase_current_a     = static_cast<double>(frame.payload[9]) * 0.03;
    feedback.temperature_c       = static_cast<double>(frame.payload[10]) * 0.4;
    feedback.fault_code          = frame.payload[11];
    feedback.run_state           = frame.payload[12];
    feedback.system_status_valid = true;
    return true;
}

bool MotorClient::read_feedback(double encoder_zero_deg, MotorFeedback& feedback,
                                int timeout_ms, bool quiet)
{
    const std::vector<uint8_t> request =
        build_motor_frame(kCmdReadLegacyFeedback, {}, cfg_.sequence, cfg_.address);
    if (!send_raw(request))
        return false;

    return read_feedback_command(kCmdReadLegacyFeedback, encoder_zero_deg, feedback,
                                 timeout_ms, quiet);
}

bool MotorClient::read_feedback_command(uint8_t command, double encoder_zero_deg,
                                        MotorFeedback& feedback, int timeout_ms,
                                        bool quiet)
{
    MotorFrame frame;
    if (!read_frame(command, frame, timeout_ms, quiet))
        return false;
    if (!validate_payload_len(frame, kLegacyFeedbackPayloadLen, quiet,
                              "encoder feedback")) {
        return false;
    }

    parse_encoder_feedback(encoder_zero_deg, frame, feedback);
    feedback.bus_voltage_v       = 0.0;
    feedback.phase_current_a     = 0.0;
    feedback.temperature_c       = 0.0;
    feedback.fault_code          = 0;
    feedback.run_state           = 0;
    feedback.system_status_valid = false;
    return true;
}

bool MotorClient::read_frame(uint8_t expected_command, MotorFrame& frame,
                             int timeout_ms, bool quiet, MotorReadStats* stats)
{
    MotorReadStats local_stats;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(0, timeout_ms));

    while (remaining_ms(deadline) > 0) {
        MotorFrame candidate;
        if (!read_motor_frame_until(port_, candidate, deadline)) {
            if (!quiet && !local_stats.saw_frame) {
                fprintf(stderr, "MotorClient: command 0x%02X read timeout\n",
                        static_cast<unsigned>(expected_command));
            }
            break;
        }

        local_stats.saw_frame = true;
        if (candidate.payload_len > kMaxPayloadLen) {
            ++local_stats.malformed_frames;
            if (!quiet || cfg_.debug_frames) {
                fprintf(stderr,
                        "MotorClient: skip malformed frame cmd=0x%02X payload_len=%u\n",
                        static_cast<unsigned>(candidate.command),
                        static_cast<unsigned>(candidate.payload_len));
            }
            continue;
        }
        if (!candidate.crc_ok) {
            ++local_stats.crc_errors;
            if (!quiet || cfg_.debug_frames) {
                fprintf(stderr,
                        "MotorClient: skip CRC-bad frame cmd=0x%02X len=%u calc=0x%04X recv=0x%04X\n",
                        static_cast<unsigned>(candidate.command),
                        static_cast<unsigned>(candidate.payload_len),
                        candidate.crc_calculated, candidate.crc_received);
            }
            continue;
        }
        if (cfg_.strict_address && candidate.address != cfg_.address) {
            ++local_stats.address_mismatches;
            if (!quiet || cfg_.debug_frames) {
                fprintf(stderr,
                        "MotorClient: skip address-mismatch frame cmd=0x%02X addr=0x%02X expected=0x%02X\n",
                        static_cast<unsigned>(candidate.command),
                        static_cast<unsigned>(candidate.address),
                        static_cast<unsigned>(cfg_.address));
            }
            continue;
        }
        if (candidate.command != expected_command) {
            ++local_stats.skipped_frames;
            local_stats.last_skipped_command = candidate.command;
            local_stats.last_skipped_payload_len = candidate.payload_len;
            if (!quiet || cfg_.debug_frames) {
                fprintf(stderr,
                        "MotorClient: skip frame cmd=0x%02X len=%u while waiting for 0x%02X\n",
                        static_cast<unsigned>(candidate.command),
                        static_cast<unsigned>(candidate.payload_len),
                        static_cast<unsigned>(expected_command));
            }
            continue;
        }

        frame = candidate;
        last_read_stats_ = local_stats;
        if (stats)
            *stats = local_stats;
        return true;
    }

    last_read_stats_ = local_stats;
    if (stats)
        *stats = local_stats;
    if ((!quiet || cfg_.debug_frames) &&
        (local_stats.skipped_frames > 0 || local_stats.crc_errors > 0 ||
         local_stats.address_mismatches > 0 || local_stats.malformed_frames > 0)) {
        fprintf(stderr,
                "MotorClient: no 0x%02X before timeout; skipped=%d crc_bad=%d addr_bad=%d malformed=%d last_skip=0x%02X len=%u\n",
                static_cast<unsigned>(expected_command),
                local_stats.skipped_frames, local_stats.crc_errors,
                local_stats.address_mismatches, local_stats.malformed_frames,
                static_cast<unsigned>(local_stats.last_skipped_command),
                static_cast<unsigned>(local_stats.last_skipped_payload_len));
    }
    return false;
}

bool MotorClient::consume_response(uint8_t command, int timeout_ms, bool quiet)
{
    MotorFeedback unused;
    if (command == static_cast<uint8_t>(cfg_.velocity_command) ||
        command == static_cast<uint8_t>(cfg_.position_command) ||
        command == 0x50 || command == 0x51 || command == 0x52 ||
        command == 0x53 || command == 0x55 || command == 0x56) {
        return read_feedback_command(command, 0.0, unused, timeout_ms, quiet);
    }

    MotorFrame frame;
    return read_frame(command, frame, timeout_ms, quiet);
}
