#include <yaml-cpp/yaml.h>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

#include "serial_port.hpp"
#include "modbus_crc.hpp"
#include "controller.hpp"
#include "vision.hpp"

static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running = false; }

// Load all parameters from YAML into ControlParams and supporting structs.
struct AppConfig {
    std::string serial_port;
    int         baud_rate;
    float       initial_angle_rad;
    float       encoder_zero_offset_deg;
    ControlParams ctrl;
    FeatureExtractor::Config vision;
};

static AppConfig load_config(const std::string& path)
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
    c.baud_rate    = y["serial"]["baud_rate"].as<int>();

    c.initial_angle_rad         = y["robot"]["initial_angle_rad"].as<float>();
    c.encoder_zero_offset_deg   = y["robot"]["encoder_zero_offset_deg"].as<float>();

    auto intr = y["robot"]["camera_intrinsics"].as<std::vector<float>>();
    c.ctrl.fx = intr[0]; c.ctrl.fy = intr[1];
    c.ctrl.cx = intr[2]; c.ctrl.cy = intr[3];

    auto ex = y["robot"]["camera_extrinsics"].as<std::vector<float>>();
    for (int i = 0; i < 12; i++) c.ctrl.cam_ex[i] = ex[i];

    c.ctrl.L = y["robot"]["L"].as<float>();
    auto e1 = y["robot"]["rt_e1"].as<std::vector<float>>();
    auto e2 = y["robot"]["rt_e2"].as<std::vector<float>>();
    for (int i = 0; i < 3; i++) { c.ctrl.rt_e1[i] = e1[i]; c.ctrl.rt_e2[i] = e2[i]; }

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
    for (int i = 0; i < 4; i++) c.ctrl.Ps[i] = ps[i];

    auto mu = ctrl["mu_rho"].as<std::vector<float>>();
    for (int i = 0; i < 5; i++) c.ctrl.mu_rho[i] = mu[i];

    auto yd = y["vision"]["desired_coords"].as<std::vector<float>>();
    for (int i = 0; i < 6; i++) c.ctrl.yd[i] = yd[i];

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
    for (int i = 0; i < 6; i++) c.vision.desired[i] = yd[i];

    return c;
}

// Build and send a Modbus RTU velocity command frame.
// velocity_rad_s: desired joint velocity in rad/s
static void send_velocity(SerialPort& port, float velocity_rad_s)
{
    // Convert rad/s -> RPM * 10 (motor controller unit)
    float vel_rpm10 = velocity_rad_s / (2.0f * 3.1415926f) * 60.0f * 10.0f;

    uint8_t data_low, data_high;
    velocity_to_bytes(vel_rpm10, data_low, data_high);

    uint8_t frame[7] = { 0x3E, 0x00, 0x01, 0x54, 0x02, data_low, data_high };
    uint16_t crc = crc16_modbus(frame, 7);

    uint8_t crc_low  = static_cast<uint8_t>(crc & 0xFF);
    uint8_t crc_high = static_cast<uint8_t>((crc >> 8) & 0xFF);

    uint8_t packet[9] = { 0x3E, 0x00, 0x01, 0x54, 0x02,
                          data_low, data_high, crc_low, crc_high };
    if (port.write(packet, 9) < 0)
        fprintf(stderr, "send_velocity: write failed\n");
}

// Read encoder response (15 bytes) and decode angle + velocity.
// Returns false on read failure.
static bool read_encoder(SerialPort& port,
                         float encoder_zero_deg,
                         float& angle_rad, float& vel_rad_s)
{
    uint8_t buf[20] = {};
    int n = port.read(buf, 15, 5000);
    if (n < 15) {
        fprintf(stderr, "read_encoder: short read (%d bytes)\n", n);
        return false;
    }

    int angpos = 256 * static_cast<int>(buf[6]) + static_cast<int>(buf[5]);
    int angvel_raw = 256 * static_cast<int>(buf[12]) + static_cast<int>(buf[11]);
    int angvel = (angvel_raw <= 32767) ? angvel_raw : -(65536 - angvel_raw);

    float ang_deg = static_cast<float>(angpos) * 360.0f / 16384.0f;
    angle_rad  = (ang_deg - encoder_zero_deg) * 3.1415926f / 180.0f;
    vel_rad_s  = static_cast<float>(angvel) * 0.1f * 2.0f * 3.1415926f / 60.0f;
    return true;
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <config.yaml>\n", argv[0]);
        return 1;
    }

    AppConfig cfg;
    try {
        cfg = load_config(argv[1]);
    } catch (const std::exception& e) {
        fprintf(stderr, "Config error: %s\n", e.what());
        return 1;
    }

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // Open serial port
    SerialPort port(cfg.serial_port, cfg.baud_rate);
    if (!port.open()) return 1;

    // Open camera
    FeatureExtractor extractor(cfg.vision);
    if (!extractor.open()) return 1;

    // Warm up camera: discard first 5 frames
    float img_coord_va[6] = {};
    int   rad_va[3]       = {};
    for (int i = 0; i < 5; i++)
        extractor.extract(img_coord_va, rad_va);

    // Initial state
    const float joint_va     = cfg.initial_angle_rad;
    const float joint_vel_va = 0.0f;

    float state[26] = {};
    state[0]  = joint_va;
    state[1]  = joint_vel_va;
    state[2]  = img_coord_va[0]; state[3]  = img_coord_va[1];
    state[4]  = img_coord_va[2]; state[5]  = img_coord_va[3];
    state[6]  = img_coord_va[4]; state[7]  = img_coord_va[5];
    // Adaptive params: theta[4], rho[5]
    state[8]  = 400.0f; state[9]  = 420.0f;
    state[10] = 400.0f; state[11] = 300.0f;
    state[12] = 0.0001f; state[13] = 0.0001f;
    state[14] = 0.0001f; state[15] = 0.0001f; state[16] = 0.0001f;
    // Observer
    state[17] = joint_va; state[18] = joint_va;
    state[19] = 0.0f;     state[20] = 0.0f;
    // Integrator
    state[21] = joint_va;

    std::ofstream dataFile("dataFile.txt");
    if (!dataFile.is_open()) {
        fprintf(stderr, "Cannot open dataFile.txt\n");
        return 1;
    }

    while (g_running) {
        // Pack inputs from state
        float joint_state[2] = { state[0], state[1] };
        float para_slow[9];
        for (int i = 0; i < 9; i++) para_slow[i] = state[8 + i];
        float obs[4] = { state[17], state[18], state[19], state[20] };
        float q_c    = state[21];

        // Log current state
        for (int i = 0; i < 26; i++)
            dataFile << std::setw(15) << state[i];
        dataFile << '\n';

        // Vision
        float img_pos[6];
        int   rad_new[3];
        if (!extractor.extract(img_pos, rad_new)) {
            fprintf(stderr, "Feature extraction failed, skipping\n");
            continue;
        }
        printf("circles: (%.1f,%.1f,r=%d) (%.1f,%.1f,r=%d) (%.1f,%.1f,r=%d)\n",
               img_pos[0], img_pos[1], rad_new[0],
               img_pos[2], img_pos[3], rad_new[1],
               img_pos[4], img_pos[5], rad_new[2]);

        // Control
        float joint_cal   = 0.0f;
        float para_update[17] = {};
        cal_joint_vel(joint_state, img_pos, obs, para_slow, q_c,
                      &joint_cal, para_update, cfg.ctrl);

        // Saturate velocity command
        const float sat = 1.5f;
        float velocity = joint_cal;
        if (velocity >  sat) velocity =  sat;
        if (velocity < -sat) velocity = -sat;

        printf("vel_cmd=%.4f rad/s\n", velocity);

        // Send to motor
        send_velocity(port, velocity);

        // Read encoder feedback
        float angle_rad = state[0], vel_rad_s = state[1];
        read_encoder(port, cfg.encoder_zero_offset_deg, angle_rad, vel_rad_s);
        printf("angle=%.4f rad  vel=%.4f rad/s\n", angle_rad, vel_rad_s);

        // Update state
        state[0] = angle_rad;
        state[1] = vel_rad_s;
        for (int i = 0; i < 6; i++) state[2 + i] = img_pos[i];
        for (int i = 0; i < 9; i++) state[8 + i] = para_update[i];
        state[17] = para_update[9];  state[18] = para_update[10];
        state[19] = para_update[11]; state[20] = para_update[12];
        state[21] = para_update[13];
        state[22] = joint_cal;
        state[23] = para_update[14];
        state[24] = para_update[15];
        state[25] = para_update[16];
    }

    dataFile.flush();
    printf("Shutting down.\n");
    return 0;
}
