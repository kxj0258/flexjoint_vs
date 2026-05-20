#include "app_config.hpp"

#include <cstdlib>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

int scalar_to_int(const YAML::Node& node, int fallback)
{
    if (!node)
        return fallback;
    try {
        const std::string text = node.as<std::string>();
        char* end = nullptr;
        const long value = std::strtol(text.c_str(), &end, 0);
        if (end && *end == '\0')
            return static_cast<int>(value);
    } catch (...) {
    }
    return node.as<int>();
}

double scalar_to_double(const YAML::Node& node, double fallback)
{
    if (!node)
        return fallback;
    return node.as<double>();
}

bool scalar_to_bool(const YAML::Node& node, bool fallback)
{
    if (!node)
        return fallback;
    return node.as<bool>();
}

std::string scalar_to_string(const YAML::Node& node, const std::string& fallback)
{
    if (!node)
        return fallback;
    return node.as<std::string>();
}

} // namespace

AppConfig load_app_config(const std::string& path)
{
    YAML::Node y = YAML::LoadFile(path);
    AppConfig c;

    const auto serial = y["serial"];
    c.serial_port = serial["motor_port"].as<std::string>();
#ifdef _WIN32
    if (serial["windows_port"])
        c.serial_port = serial["windows_port"].as<std::string>();
#else
    if (serial["linux_port"])
        c.serial_port = serial["linux_port"].as<std::string>();
#endif
    c.baud_rate = serial["baud_rate"].as<int>();

    c.initial_angle_rad       = y["robot"]["initial_angle_rad"].as<float>();
    c.encoder_zero_offset_deg = y["robot"]["encoder_zero_offset_deg"].as<float>();

    auto intr = y["robot"]["camera_intrinsics"].as<std::vector<float>>();
    c.ctrl.fx = intr[0];
    c.ctrl.fy = intr[1];
    c.ctrl.cx = intr[2];
    c.ctrl.cy = intr[3];

    auto ex = y["robot"]["camera_extrinsics"].as<std::vector<float>>();
    for (int i = 0; i < 12; i++)
        c.ctrl.cam_ex[i] = ex[i];

    c.ctrl.L = y["robot"]["L"].as<float>();
    auto e1 = y["robot"]["rt_e1"].as<std::vector<float>>();
    auto e2 = y["robot"]["rt_e2"].as<std::vector<float>>();
    for (int i = 0; i < 3; i++) {
        c.ctrl.rt_e1[i] = e1[i];
        c.ctrl.rt_e2[i] = e2[i];
    }

    const auto ctrl = y["control"];
    c.ctrl.K1        = ctrl["K1"].as<float>();
    c.ctrl.B         = ctrl["B"].as<float>();
    c.ctrl.K2        = ctrl["K2"].as<float>();
    c.ctrl.Gamma     = ctrl["Gamma"].as<float>();
    c.ctrl.K4        = ctrl["K4"].as<float>();
    c.ctrl.eps       = ctrl["eps"].as<float>();
    c.ctrl.Kq        = ctrl["Kq"].as<float>();
    c.ctrl.cmd_c1    = ctrl["cmd_c1"].as<float>();
    c.ctrl.cmd_c2    = ctrl["cmd_c2"].as<float>();
    c.ctrl.K_dtk_I   = ctrl["K_dtk_I"].as<float>();
    c.ctrl.omega_dtk = ctrl["omega_dtk"].as<float>();
    c.ctrl.h         = ctrl["h"].as<float>();
    c.ctrl.Mr        = ctrl["Mr"].as<float>();
    c.ctrl.Jm        = ctrl["Jm"].as<float>();
    c.ctrl.Kp        = ctrl["Kp"].as<float>();
    c.ctrl.Kd        = ctrl["Kd"].as<float>();

    auto ps = ctrl["Ps"].as<std::vector<float>>();
    for (int i = 0; i < 4; i++)
        c.ctrl.Ps[i] = ps[i];

    auto mu = ctrl["mu_rho"].as<std::vector<float>>();
    for (int i = 0; i < 5; i++)
        c.ctrl.mu_rho[i] = mu[i];

    auto yd = y["vision"]["desired_coords"].as<std::vector<float>>();
    for (int i = 0; i < 6; i++)
        c.ctrl.yd[i] = yd[i];

    const auto vis = y["vision"];
    c.vision.camera_index  = y["camera"]["index"].as<int>();
    c.vision.fps           = y["camera"]["fps"].as<int>();
    c.vision.width         = y["camera"]["width"].as<int>();
    c.vision.height        = y["camera"]["height"].as<int>();
    c.vision.hough_param1  = vis["hough_param1"].as<int>();
    c.vision.hough_param2  = vis["hough_param2"].as<int>();
    c.vision.hough_min_rad = vis["hough_min_radius"].as<int>();
    c.vision.hough_max_rad = vis["hough_max_radius"].as<int>();
    c.vision.save_path     = vis["save_path"].as<std::string>();
    if (vis["hough_dp"])
        c.vision.hough_dp = vis["hough_dp"].as<double>();
    if (vis["hough_min_dist"])
        c.vision.hough_min_dist = vis["hough_min_dist"].as<double>();
    if (vis["blur_kernel"])
        c.vision.blur_kernel = vis["blur_kernel"].as<int>();
    if (vis["blur_sigma"])
        c.vision.blur_sigma = vis["blur_sigma"].as<double>();
    if (vis["equalize_hist"])
        c.vision.equalize_hist = vis["equalize_hist"].as<bool>();
    if (const auto controls = y["camera_controls"]) {
        for (const auto& entry : controls) {
            c.vision.camera_controls.emplace_back(entry.first.as<std::string>(),
                                                  scalar_to_int(entry.second, 0));
        }
    }
    for (int i = 0; i < 6; i++)
        c.vision.desired[i] = yd[i];

    const auto proto = y["motor_protocol"];
    if (proto) {
        c.motor.velocity_command = scalar_to_int(proto["velocity_command"],
                                                 c.motor.velocity_command);
        c.motor.position_command = scalar_to_int(proto["position_command"],
                                                 c.motor.position_command);
        c.motor.pid_command = scalar_to_int(proto["pid_command"],
                                            c.motor.pid_command);
        c.motor.feedback_command = scalar_to_int(proto["feedback_command"],
                                                 c.motor.feedback_command);
        c.motor.sequence = static_cast<uint8_t>(
            scalar_to_int(proto["sequence"], c.motor.sequence) & 0xFF);
        c.motor.address = static_cast<uint8_t>(
            scalar_to_int(proto["address"], c.motor.address) & 0xFF);
        if (proto["strict_address"])
            c.motor.strict_address = proto["strict_address"].as<bool>();
        if (proto["consume_command_response"]) {
            c.motor.consume_command_response =
                proto["consume_command_response"].as<bool>();
        }
        if (proto["debug_frames"])
            c.motor.debug_frames = proto["debug_frames"].as<bool>();
        c.motor.position_counts_per_rev =
            scalar_to_double(proto["position_counts_per_rev"],
                             c.motor.position_counts_per_rev);
        c.motor.pid_scale = scalar_to_double(proto["pid_scale"],
                                             c.motor.pid_scale);
    }

    const auto task = y["task"];
    if (task) {
        c.task.enable_completion_check =
            scalar_to_bool(task["enable_completion_check"],
                           c.task.enable_completion_check);
        c.task.image_error_tolerance_px =
            scalar_to_double(task["image_error_tolerance_px"],
                             c.task.image_error_tolerance_px);
        c.task.velocity_tolerance_rad_s =
            scalar_to_double(task["velocity_tolerance_rad_s"],
                             c.task.velocity_tolerance_rad_s);
        c.task.stable_frames_required =
            scalar_to_int(task["stable_frames_required"],
                          c.task.stable_frames_required);
        c.task.max_runtime_s =
            scalar_to_double(task["max_runtime_s"], c.task.max_runtime_s);
        c.task.max_control_cycles =
            scalar_to_int(task["max_control_cycles"],
                          c.task.max_control_cycles);
        c.task.return_to_zero_on_exit =
            scalar_to_bool(task["return_to_zero_on_exit"],
                           c.task.return_to_zero_on_exit);
        c.task.return_zero_mode =
            scalar_to_string(task["return_zero_mode"],
                             c.task.return_zero_mode);
        c.task.return_zero_joint_rad =
            scalar_to_double(task["return_zero_joint_rad"],
                             c.task.return_zero_joint_rad);
        c.task.return_zero_motor_pos_rad =
            scalar_to_double(task["return_zero_motor_pos_rad"],
                             c.task.return_zero_motor_pos_rad);
        c.task.return_to_zero_timeout_s =
            scalar_to_double(task["return_to_zero_timeout_s"],
                             c.task.return_to_zero_timeout_s);
        c.task.max_vision_failures =
            scalar_to_int(task["max_vision_failures"],
                          c.task.max_vision_failures);
        c.task.max_feedback_failures =
            scalar_to_int(task["max_feedback_failures"],
                          c.task.max_feedback_failures);
        c.task.velocity_saturation_rad_s =
            scalar_to_double(task["velocity_saturation_rad_s"],
                             c.task.velocity_saturation_rad_s);
        c.task.min_safe_angle_rad =
            scalar_to_double(task["min_safe_angle_rad"],
                             c.task.min_safe_angle_rad);
        c.task.max_safe_angle_rad =
            scalar_to_double(task["max_safe_angle_rad"],
                             c.task.max_safe_angle_rad);
        c.task.vision_boundary_margin_px =
            scalar_to_double(task["vision_boundary_margin_px"],
                             c.task.vision_boundary_margin_px);
    }

    const auto log = y["log"];
    if (log) {
        c.log.root_dir = scalar_to_string(log["root_dir"], c.log.root_dir);
    }

    const auto video = y["video"];
    if (video) {
        c.video.save_raw_video =
            scalar_to_bool(video["save_raw_video"], c.video.save_raw_video);
        c.video.save_annotated_video =
            scalar_to_bool(video["save_annotated_video"],
                           c.video.save_annotated_video);
        c.video.raw_filename =
            scalar_to_string(video["raw_filename"], c.video.raw_filename);
        c.video.annotated_filename =
            scalar_to_string(video["annotated_filename"],
                             c.video.annotated_filename);
        c.video.codec = scalar_to_string(video["codec"], c.video.codec);
    }

    const auto experiment = y["experiment"];
    if (experiment) {
        c.experiment.controller_mode =
            scalar_to_string(experiment["controller_mode"],
                             c.experiment.controller_mode);
    }

    return c;
}
