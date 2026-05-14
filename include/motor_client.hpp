#pragma once

#include <cstdint>
#include <string>
#include <vector>

class SerialPort;

struct MotorProtocolConfig {
    int    velocity_command        = 0x54;
    int    position_command        = -1;
    int    pid_command             = -1;
    double position_counts_per_rev = 16384.0;
    double pid_scale               = 1000.0;
};

struct MotorFeedback {
    int    raw_angle_count       = 0;
    int    raw_velocity_count    = 0;
    double absolute_position_deg = 0.0;
    double absolute_position_rad = 0.0;
    double joint_position_rad    = 0.0;
    double velocity_rad_s        = 0.0;
};

std::vector<uint8_t> build_motor_frame(uint8_t command,
                                       const std::vector<uint8_t>& payload);

class MotorClient {
public:
    MotorClient(SerialPort& port, const MotorProtocolConfig& cfg);

    bool send_velocity_rad_s(double velocity_rad_s);
    bool send_position_rad(double position_rad);
    bool send_pid(double kp, double ki, double kd);
    bool send_frame(uint8_t command, const std::vector<uint8_t>& payload);
    bool send_raw(const std::vector<uint8_t>& bytes);

    bool read_feedback(double encoder_zero_deg, MotorFeedback& feedback,
                       int timeout_ms, bool quiet = false);

    const MotorProtocolConfig& protocol() const { return cfg_; }
    MotorProtocolConfig& protocol() { return cfg_; }

private:
    SerialPort&          port_;
    MotorProtocolConfig  cfg_;
};
