#pragma once

#include <cstdint>
#include <string>
#include <vector>

class SerialPort;

struct MotorProtocolConfig {
    int    velocity_command        = 0x54;
    int    position_command        = 0x55;
    int    pid_command             = -1;
    int    feedback_command        = 0x0B;
    uint8_t sequence               = 0x00;
    uint8_t address                = 0x01;
    bool   strict_address          = false;
    bool   consume_command_response = true;
    bool   debug_frames            = false;
    double position_counts_per_rev = 16384.0;
    double pid_scale               = 1000.0;
};

struct MotorFrame {
    uint8_t header = 0;
    uint8_t sequence = 0;
    uint8_t address = 0;
    uint8_t command = 0;
    uint8_t payload_len = 0;
    std::vector<uint8_t> payload;
    uint16_t crc_received = 0;
    uint16_t crc_calculated = 0;
    bool crc_ok = false;
};

struct MotorReadStats {
    int skipped_frames = 0;
    int crc_errors = 0;
    int address_mismatches = 0;
    int malformed_frames = 0;
    uint8_t last_skipped_command = 0;
    uint8_t last_skipped_payload_len = 0;
    bool saw_frame = false;
};

struct MotorFeedback {
    int    raw_angle_count       = 0;
    int32_t raw_multi_turn_count = 0;
    int    raw_velocity_count    = 0;
    double absolute_position_deg = 0.0;
    double absolute_position_rad = 0.0;
    double multi_turn_position_deg = 0.0;
    double multi_turn_position_rad = 0.0;
    double joint_position_rad    = 0.0;
    double velocity_rad_s        = 0.0;
    double bus_voltage_v         = 0.0;
    double phase_current_a       = 0.0;
    double temperature_c         = 0.0;
    uint8_t fault_code           = 0;
    uint8_t run_state            = 0;
    bool   system_status_valid   = false;
};

struct MotorSystemParams {
    uint8_t device_address = 0;
    uint8_t current_threshold_raw = 0;
    uint8_t max_voltage_threshold_raw = 0;
    uint8_t baud_config_raw = 0;
    float position_kp = 0.0f;
    float position_target_speed_rpm10 = 0.0f;
    float velocity_kp = 0.0f;
    float velocity_ki = 0.0f;
    float reserved = 0.0f;
    uint8_t velocity_filter_raw = 0;
    uint8_t power_percent = 0;
};

std::vector<uint8_t> build_motor_frame(uint8_t command,
                                       const std::vector<uint8_t>& payload);
std::vector<uint8_t> build_motor_frame(uint8_t command,
                                       const std::vector<uint8_t>& payload,
                                       uint8_t sequence,
                                       uint8_t address);

class MotorClient {
public:
    MotorClient(SerialPort& port, const MotorProtocolConfig& cfg);

    bool send_velocity_rad_s(double velocity_rad_s);
    bool send_position_counts(uint32_t position_counts);
    bool send_position_rad(double position_rad);
    bool send_pid(double kp, double ki, double kd);
    bool send_frame(uint8_t command, const std::vector<uint8_t>& payload);
    bool send_raw(const std::vector<uint8_t>& bytes);

    bool read_system_status(double encoder_zero_deg, MotorFeedback& feedback,
                            int timeout_ms, bool quiet = false);
    bool read_feedback(double encoder_zero_deg, MotorFeedback& feedback,
                       int timeout_ms, bool quiet = false);
    bool read_feedback_command(uint8_t command, double encoder_zero_deg,
                               MotorFeedback& feedback, int timeout_ms,
                               bool quiet = false);
    bool read_system_params(MotorSystemParams& params, int timeout_ms,
                            bool quiet = false);
    bool write_system_params(const MotorSystemParams& params, bool save_to_flash,
                             int timeout_ms, bool quiet = false,
                             MotorSystemParams* echoed_params = nullptr);
    bool read_frame(uint8_t expected_command, MotorFrame& frame,
                    int timeout_ms, bool quiet = false,
                    MotorReadStats* stats = nullptr);
    bool consume_response(uint8_t command, int timeout_ms = 5,
                          bool quiet = true);

    const MotorProtocolConfig& protocol() const { return cfg_; }
    MotorProtocolConfig& protocol() { return cfg_; }
    const MotorReadStats& last_read_stats() const { return last_read_stats_; }

private:
    SerialPort&          port_;
    MotorProtocolConfig  cfg_;
    MotorReadStats       last_read_stats_;
};
