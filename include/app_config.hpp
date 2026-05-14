#pragma once

#include <string>

#include "controller.hpp"
#include "motor_client.hpp"
#include "vision.hpp"

struct AppConfig {
    std::string serial_port;
    int         baud_rate = 115200;
    float       initial_angle_rad = 0.0f;
    float       encoder_zero_offset_deg = 0.0f;
    ControlParams ctrl;
    FeatureExtractor::Config vision;
    MotorProtocolConfig motor;
};

AppConfig load_app_config(const std::string& path);
