#pragma once

#include <string>

#include "controller.hpp"
#include "motor_client.hpp"
#include "vision.hpp"

struct TaskConfig {
    bool   enable_completion_check = true;
    double image_error_tolerance_px = 1.0;
    double velocity_tolerance_rad_s = 0.03;
    int    stable_frames_required = 300;
    double max_runtime_s = 120.0;
    int    max_control_cycles = 0;
    bool   return_to_zero_on_exit = true;
    std::string return_zero_mode = "motor_pos_rad";
    double return_zero_joint_rad = 0.0;
    double return_zero_motor_pos_rad = 0.0;
    double return_to_zero_timeout_s = 8.0;
    int    max_vision_failures = 30;
    int    max_feedback_failures = 30;
    double velocity_saturation_rad_s = 1.5;
    double min_safe_angle_rad = -1.0;
    double max_safe_angle_rad = 1.0;
    double vision_boundary_margin_px = 25.0;
};

struct LogConfig {
    std::string root_dir = "data/log";
};

struct VideoConfig {
    bool        save_raw_video = true;
    bool        save_annotated_video = true;
    std::string raw_filename = "raw_video.mp4";
    std::string annotated_filename = "annotated_video.mp4";
    std::string codec = "mp4v";
};

struct ExperimentConfig {
    std::string controller_mode = "proposed";
};

struct AppConfig {
    std::string serial_port;
    int         baud_rate = 115200;
    float       initial_angle_rad = 0.0f;
    float       encoder_zero_offset_deg = 0.0f;
    ControlParams ctrl;
    FeatureExtractor::Config vision;
    MotorProtocolConfig motor;
    TaskConfig task;
    LogConfig log;
    VideoConfig video;
    ExperimentConfig experiment;
};

AppConfig load_app_config(const std::string& path);
