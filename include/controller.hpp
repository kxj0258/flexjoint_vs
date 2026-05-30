#pragma once

#include "feature_points.hpp"

// Control algorithms for the soft robot arm visual servoing system.
//
// main.cpp stores control state in a typed ControlState struct. The controller
// API receives only the slices it needs, so image coordinates can use N=2/3/4.

// ControlParams holds all tunable gains loaded from the YAML config.
// Keeping them in a struct avoids re-reading the config on every call.
struct ControlParams {
    int feature_count = kLegacyFeaturePoints;
    // Slow layer
    float K1 = 1.42f;
    float B = 0.000049f;
    float K2 = 0.001f;
    float Gamma = 0.0001f;
    // Fast layer
    float K4 = 2.7f;
    float eps = 0.1f;
    float Kq = 50.0f;
    // Command mixing
    float cmd_c1 = 1.0f;
    float cmd_c2 = 0.325f;
    // Integrator
    float K_dtk_I = 0.8f;
    float omega_dtk = 0.6f;
    // Integration step
    float h = 0.04f;
    // Arm geometry
    float L = 0.3f;
    float rt_e1[3] = {0.06f, -0.055f, 0.0f};
    float rt_e2[3] = {0.075f, 0.075f, 0.0f};
    float feature_offsets[kMaxFeaturePoints][3] = {
        {0.0f, 0.0f, 0.0f},
        {0.06f, -0.055f, 0.0f},
        {0.075f, 0.075f, 0.0f},
        {0.0f, 0.0f, 0.0f}
    };
    // Camera intrinsics (fx, fy, cx, cy)
    float fx = 487.05f, fy = 487.05f, cx = 338.23f, cy = 231.89f;
    // Camera extrinsics (row-major 3x4, last row implicit [0,0,0,1])
    float cam_ex[12] = {
        -0.0561f, -0.9962f, -0.0673f, 0.0015f,
        0.9981f, -0.0577f, 0.0208f, -0.4200f,
        -0.0246f, -0.0660f, 0.9975f, 0.4523f
    };
    // Desired image coordinates (u1,v1, ... uN,vN)
    float yd[kMaxImageCoords] = {
        264.5f, 96.5f, 298.5f, 166.5f, 174.5f, 144.5f, 0.0f, 0.0f
    };
    // Observer gains
    float Ps[4] = {10.0f, 15.0f, 10.0f, 6.0f};
    float Mr = 1.0f, Jm = 1.0f;
    // Adaptive learning rates
    float mu_rho[5] = {0.01f, 0.0001f, 0.01f, 0.001f, 0.001f};
    // PD gains for CalControl (Luca method)
    float Kp = 5.0f, Kd = 0.35f;
};

// Primary controller: dual-layer visual servoing with online parameter adaptation.
//
// joint_state[2]  : [joint_angle, joint_vel] - current encoder feedback
// img_coord[6]    : [u1,v1, u2,v2, u3,v3]   - detected feature point pixels
// obs[4]          : observer state [theta_obs, theta_obs_dot, vel_obs, vel_obs_dot]
// para_slow[9]    : adaptive parameters [theta[4], rho[5]]
// q_c             : integrator state
// joint_vel_cmd   : output velocity command (rad/s)
// para_update[17] : updated parameters written back to state vector
//   [0..3]  theta_update
//   [4..8]  rho_update
//   [9..12] observer state update
//   [13]    qc_update
//   [14]    tau
//   [15]    tau_s
//   [16]    tau_f_c
void cal_joint_vel(const float joint_state[2], const float img_coord[6],
                   const float obs[4], const float para_slow[9], float q_c,
                   float* joint_vel_cmd, float para_update[17],
                   const ControlParams& p);

void cal_joint_vel_features(const float joint_state[2], const float* img_coord,
                            int feature_count,
                            const float obs[4], const float para_slow[9],
                            float q_c, float* joint_vel_cmd,
                            float para_update[17],
                            const ControlParams& p);

// Backup PD controller (Luca method) - simpler, fewer adaptive parameters.
//
// joint_state[2]  : [joint_angle, joint_vel]
// img_coord[6]    : [u1,v1, u2,v2, u3,v3]
// obs[4]          : observer state
// q_c             : integrator state
// ydif[6]         : finite-difference image velocity (dy/dt)
// joint_vel_cmd   : output velocity command
// para_update[8]  : updated state
//   [0..3] observer state update
//   [4]    qc_update
//   [5]    tau
//   [6]    tau_s
//   [7]    tau_f_c
void cal_control(const float joint_state[2], const float img_coord[6],
                 const float obs[4], float q_c, const float ydif[6],
                 float* joint_vel_cmd, float para_update[8],
                 const ControlParams& p);

void cal_control_features(const float joint_state[2], const float* img_coord,
                          int feature_count,
                          const float obs[4], float q_c,
                          const float* ydif,
                          float* joint_vel_cmd, float para_update[8],
                          const ControlParams& p);
