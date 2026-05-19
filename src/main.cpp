#include <atomic>
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iomanip>
#include <string>

#include "app_config.hpp"
#include "serial_port.hpp"
#include "controller.hpp"
#include "motor_client.hpp"
#include "vision.hpp"

static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running = false; }

namespace {

constexpr float kVelocitySaturation = 1.5f;
constexpr float kMaxSafeAngleRad = 1.0f;
constexpr float kMinSafeAngleRad = -1.0f;
constexpr float kVisionMarginPx = 25.0f;

bool feature_points_near_boundary(const float img_pos[6], float width, float height,
                                  float margin)
{
    for (int i = 0; i < 3; ++i) {
        const float u = img_pos[2 * i];
        const float v = img_pos[2 * i + 1];
        if (u < margin || u > width - margin ||
            v < margin || v > height - margin) {
            return true;
        }
    }
    return false;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <config.yaml>\n", argv[0]);
        return 1;
    }

    AppConfig cfg;
    try {
        cfg = load_app_config(argv[1]);
    } catch (const std::exception& e) {
        fprintf(stderr, "Config error: %s\n", e.what());
        return 1;
    }

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // Open serial port
    SerialPort port(cfg.serial_port, cfg.baud_rate);
    if (!port.open()) return 1;
    MotorClient motor(port, cfg.motor);

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
        // 先读当前关节状态，再参与本周期控制计算，避免安全判断滞后一拍。
        float angle_rad = state[0];
        float vel_rad_s = state[1];
        MotorFeedback feedback;
        if (motor.read_system_status(cfg.encoder_zero_offset_deg, feedback, 200, true) ||
            motor.read_feedback(cfg.encoder_zero_offset_deg, feedback, 200, true)) {
            angle_rad = static_cast<float>(feedback.joint_position_rad);
            vel_rad_s = static_cast<float>(feedback.velocity_rad_s);
        }

        state[0] = angle_rad;
        state[1] = vel_rad_s;

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

        // 第一层保护：先做速度限幅，防止控制器输出瞬时过大。
        float velocity = joint_cal;
        velocity = std::clamp(velocity, -kVelocitySaturation, kVelocitySaturation);

        // 第二层保护：位置软限位。
        // 如果当前位置已经越界，并且速度指令还在朝危险方向继续运动，则强制清零。
        if ((angle_rad > kMaxSafeAngleRad && velocity > 0.0f) ||
            (angle_rad < kMinSafeAngleRad && velocity < 0.0f)) {
            velocity = 0.0f;
            printf("Position Limit Triggered\n");
        }

        // 第三层保护：视觉特征点边界限制。
        // 任意一个特征点接近图像边缘时，立即停止，避免目标飞出视野导致伺服失稳。
        const bool vision_limit =
            feature_points_near_boundary(img_pos,
                                         static_cast<float>(cfg.vision.width),
                                         static_cast<float>(cfg.vision.height),
                                         kVisionMarginPx);
        if (vision_limit) {
            velocity = 0.0f;
            printf("Vision Boundary Limit Triggered\n");
        }

        printf("vel_cmd=%.4f rad/s (joint_cal=%.4f)\n", velocity, joint_cal);

        // Send to motor
        motor.send_velocity_rad_s(velocity);
        printf("angle=%.4f rad  vel=%.4f rad/s\n", angle_rad, vel_rad_s);

        // Update state
        for (int i = 0; i < 6; i++) state[2 + i] = img_pos[i];
        for (int i = 0; i < 9; i++) state[8 + i] = para_update[i];
        state[17] = para_update[9];  state[18] = para_update[10];
        state[19] = para_update[11]; state[20] = para_update[12];
        state[21] = para_update[13];
        state[22] = velocity;
        state[23] = para_update[14];
        state[24] = para_update[15];
        state[25] = para_update[16];
    }

    dataFile.flush();
    printf("Shutting down.\n");
    return 0;
}
