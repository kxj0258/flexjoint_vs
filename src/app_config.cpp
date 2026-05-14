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
        c.motor.position_counts_per_rev =
            scalar_to_double(proto["position_counts_per_rev"],
                             c.motor.position_counts_per_rev);
        c.motor.pid_scale = scalar_to_double(proto["pid_scale"],
                                             c.motor.pid_scale);
    }

    return c;
}
