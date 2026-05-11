#include "controller.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

static float clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void cal_joint_vel(const float joint_state[2], const float img_coord[6],
                   const float obs[4], const float para_slow[9], float q_c,
                   float* joint_vel_cmd, float para_update[17],
                   const ControlParams& p)
{
    const float L       = p.L;
    const float K1      = p.K1;
    const float B       = p.B;
    const float K2      = p.K2;
    const float Gamma   = p.Gamma;
    const float K4      = p.K4;
    const float eps     = p.eps;
    const float Kq      = p.Kq;
    const float cmd_c1  = p.cmd_c1;
    const float cmd_c2  = p.cmd_c2;
    const float K_dtk_I = p.K_dtk_I;
    const float omega_dtk = p.omega_dtk;
    const float h       = p.h;
    const float Mr      = p.Mr;
    const float Jm      = p.Jm;

    const float* rt_e1 = p.rt_e1;
    const float* rt_e2 = p.rt_e2;

    Matrix<float, 3, 4> cam_K;
    cam_K << p.fx, 0,    p.cx, 0,
             0,    p.fy, p.cy, 0,
             0,    0,    1,    0;

    Matrix<float, 4, 4> cam_Ex;
    cam_Ex << p.cam_ex[0],  p.cam_ex[1],  p.cam_ex[2],  p.cam_ex[3],
              p.cam_ex[4],  p.cam_ex[5],  p.cam_ex[6],  p.cam_ex[7],
              p.cam_ex[8],  p.cam_ex[9],  p.cam_ex[10], p.cam_ex[11],
              0, 0, 0, 1;

    Vector4f T1, T2, T3;
    T1 << cam_Ex(0,0), cam_Ex(0,1), cam_Ex(0,2), cam_Ex(0,3);
    T2 << cam_Ex(1,0), cam_Ex(1,1), cam_Ex(1,2), cam_Ex(1,3);
    T3 << cam_Ex(2,0), cam_Ex(2,1), cam_Ex(2,2), cam_Ex(2,3);

    const float joint_ang = obs[0];
    const float joint_vel = obs[2];

    Matrix<float, 4, 4> Q, Qinv;
    Q << 0, 1, 0, 0,
         0, 0, 0, 1,
         (1.0f/Jm)*Kq, -(1.0f/Jm)*Kq, 0, 0,
         0, 0, (1.0f/Jm)*Kq, -(1.0f/Jm)*Kq;
    Qinv = Q.inverse();

    Vector4f Ps;
    Ps << p.Ps[0], p.Ps[1], p.Ps[2], p.Ps[3];

    Matrix<float, 1, 4> Q1, Q2, Q3, Q4;
    Q1 << Qinv(0,0), Qinv(0,1), Qinv(0,2), Qinv(0,3);
    Q2 << Qinv(1,0), Qinv(1,1), Qinv(1,2), Qinv(1,3);
    Q3 << Qinv(2,0), Qinv(2,1), Qinv(2,2), Qinv(2,3);
    Q4 << Qinv(3,0), Qinv(3,1), Qinv(3,2), Qinv(3,3);

    Vector4f pos_end, pos_e1, pos_e2;
    pos_end << L*cosf(joint_ang), L*sinf(joint_ang), 0, 1;
    pos_e1  << L*cosf(joint_ang) + rt_e1[0]*cosf(joint_ang) - rt_e1[1]*sinf(joint_ang),
               L*sinf(joint_ang) + rt_e1[0]*sinf(joint_ang) + rt_e1[1]*cosf(joint_ang), 0, 1;
    pos_e2  << L*cosf(joint_ang) + rt_e2[0]*cosf(joint_ang) - rt_e2[1]*sinf(joint_ang),
               L*sinf(joint_ang) + rt_e2[0]*sinf(joint_ang) + rt_e2[1]*cosf(joint_ang), 0, 1;

    Vector4f Js1, Js2, Js3;
    Js1 << -L*sinf(joint_ang), L*cosf(joint_ang), 0, 0;
    Js2 << -L*sinf(joint_ang) - rt_e1[0]*sinf(joint_ang) - rt_e1[1]*cosf(joint_ang),
            L*cosf(joint_ang) + rt_e1[0]*cosf(joint_ang) - rt_e1[1]*sinf(joint_ang), 0, 0;
    Js3 << -L*sinf(joint_ang) - rt_e2[0]*sinf(joint_ang) - rt_e2[1]*cosf(joint_ang),
            L*cosf(joint_ang) + rt_e2[0]*cosf(joint_ang) - rt_e2[1]*sinf(joint_ang), 0, 0;

    const float u1 = img_coord[0], v1 = img_coord[1];
    const float u2 = img_coord[2], v2 = img_coord[3];
    const float u3 = img_coord[4], v3 = img_coord[5];

    const float* yd = p.yd;

    Vector2f delta_y1, delta_y2, delta_y3;
    delta_y1 << u1 - yd[0], v1 - yd[1];
    delta_y2 << u2 - yd[2], v2 - yd[3];
    delta_y3 << u3 - yd[4], v3 - yd[5];

    Matrix<float, 2, 4> A1, A2, A3;
    A1 << para_slow[0]*T1(0)+(para_slow[1]-u1)*T3(0), para_slow[0]*T1(1)+(para_slow[1]-u1)*T3(1),
          para_slow[0]*T1(2)+(para_slow[1]-u1)*T3(2), para_slow[0]*T1(3)+(para_slow[1]-u1)*T3(3),
          para_slow[2]*T2(0)+(para_slow[3]-v1)*T3(0), para_slow[2]*T2(1)+(para_slow[3]-v1)*T3(1),
          para_slow[2]*T2(2)+(para_slow[3]-v1)*T3(2), para_slow[2]*T2(3)+(para_slow[3]-v1)*T3(3);
    A2 << para_slow[0]*T1(0)+(para_slow[1]-u2)*T3(0), para_slow[0]*T1(1)+(para_slow[1]-u2)*T3(1),
          para_slow[0]*T1(2)+(para_slow[1]-u2)*T3(2), para_slow[0]*T1(3)+(para_slow[1]-u2)*T3(3),
          para_slow[2]*T2(0)+(para_slow[3]-v2)*T3(0), para_slow[2]*T2(1)+(para_slow[3]-v2)*T3(1),
          para_slow[2]*T2(2)+(para_slow[3]-v2)*T3(2), para_slow[2]*T2(3)+(para_slow[3]-v2)*T3(3);
    A3 << para_slow[0]*T1(0)+(para_slow[1]-u3)*T3(0), para_slow[0]*T1(1)+(para_slow[1]-u3)*T3(1),
          para_slow[0]*T1(2)+(para_slow[1]-u3)*T3(2), para_slow[0]*T1(3)+(para_slow[1]-u3)*T3(3),
          para_slow[2]*T2(0)+(para_slow[3]-v3)*T3(0), para_slow[2]*T2(1)+(para_slow[3]-v3)*T3(1),
          para_slow[2]*T2(2)+(para_slow[3]-v3)*T3(2), para_slow[2]*T2(3)+(para_slow[3]-v3)*T3(3);

    Matrix<float, 1, 4> c_slow_Q = Mr*Q3 + Jm*Q4 - Jm*(Q2 - Q1)*Ps*Q2;
    float c_slow = (c_slow_Q(0)*Ps(0) + c_slow_Q(1)*Ps(1) +
                    c_slow_Q(2)*Ps(2) + c_slow_Q(3)*Ps(3)) * (joint_state[0] - obs[1]);

    float tau_s = (Mr + Jm) * (
        -K1 * joint_vel
        - Js1.transpose() * (A1.transpose() + 0.5f*T3*delta_y1.transpose()) * B * delta_y1
        - Js2.transpose() * (A2.transpose() + 0.5f*T3*delta_y2.transpose()) * B * delta_y2
        - Js3.transpose() * (A3.transpose() + 0.5f*T3*delta_y3.transpose()) * B * delta_y3
    ) - c_slow;
    tau_s = clamp(tau_s, -2.0f, 2.0f);

    float tau_f_c = -(1.0f/eps) * K4 * (obs[3] - obs[2]);
    float tau_f   = clamp(tau_f_c, -1.0f, 1.0f);

    float tau_sob = tau_s + tau_f;

    // --- Observer RK4 ---
    Matrix<float, 4, 4> Fs;
    Matrix<float, 4, 1> Gs;
    Fs << 0, 0, 1, 0,
          0, 0, 0, 1,
          -(1.0f/Mr)*Kq, (1.0f/Mr)*Kq, 0, 0,
           (1.0f/Jm)*Kq,-(1.0f/Jm)*Kq, 0, 0;
    Gs << 0, 0, 0, 1.0f/Jm;

    Vector4f st_ini; st_ini << obs[0], obs[1], obs[2], obs[3];
    auto obs_dot = [&](const Vector4f& s) -> Vector4f {
        return Fs*s + Gs*tau_sob + Qinv*Ps*(joint_state[0] - s(1));
    };
    Vector4f k1 = obs_dot(st_ini);
    Vector4f k2 = obs_dot(st_ini + 0.5f*h*k1);
    Vector4f k3 = obs_dot(st_ini + 0.5f*h*k2);
    Vector4f k4 = obs_dot(st_ini + h*k3);
    Vector4f st_update = st_ini + (h/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);

    // --- Theta (camera param) RK4 ---
    const float z1 = T3.dot(pos_end);
    const float z2 = T3.dot(pos_e1);
    const float z3 = T3.dot(pos_e2);

    Vector2f y1v; y1v << u1, v1;
    Vector2f y2v; y2v << u2, v2;
    Vector2f y3v; y3v << u3, v3;

    Matrix<float, 4, 4> D1, D2, D3;
    D1 << T1(0)*B*delta_y1(0), T3(0)*B*delta_y1(0), T2(0)*B*delta_y1(1), T3(0)*B*delta_y1(1),
          T1(1)*B*delta_y1(0), T3(1)*B*delta_y1(0), T2(1)*B*delta_y1(1), T3(1)*B*delta_y1(1),
          T1(2)*B*delta_y1(0), T3(2)*B*delta_y1(0), T2(2)*B*delta_y1(1), T3(2)*B*delta_y1(1),
          T1(3)*B*delta_y1(0), T3(3)*B*delta_y1(0), T2(3)*B*delta_y1(1), T3(3)*B*delta_y1(1);
    D2 << T1(0)*B*delta_y2(0), T3(0)*B*delta_y2(0), T2(0)*B*delta_y2(1), T3(0)*B*delta_y2(1),
          T1(1)*B*delta_y2(0), T3(1)*B*delta_y2(0), T2(1)*B*delta_y2(1), T3(1)*B*delta_y2(1),
          T1(2)*B*delta_y2(0), T3(2)*B*delta_y2(0), T2(2)*B*delta_y2(1), T3(2)*B*delta_y2(1),
          T1(3)*B*delta_y2(0), T3(3)*B*delta_y2(0), T2(3)*B*delta_y2(1), T3(3)*B*delta_y2(1);
    D3 << T1(0)*B*delta_y3(0), T3(0)*B*delta_y3(0), T2(0)*B*delta_y3(1), T3(0)*B*delta_y3(1),
          T1(1)*B*delta_y3(0), T3(1)*B*delta_y3(0), T2(1)*B*delta_y3(1), T3(1)*B*delta_y3(1),
          T1(2)*B*delta_y3(0), T3(2)*B*delta_y3(0), T2(2)*B*delta_y3(1), T3(2)*B*delta_y3(1),
          T1(3)*B*delta_y3(0), T3(3)*B*delta_y3(0), T2(3)*B*delta_y3(1), T3(3)*B*delta_y3(1);

    Matrix<float, 2, 4> W1, W2, W3;
    W1 << T1.dot(pos_end), T3.dot(pos_end), 0, 0,
          0, 0, T2.dot(pos_end), T3.dot(pos_end);
    W2 << T1.dot(pos_e1), T3.dot(pos_e1), 0, 0,
          0, 0, T2.dot(pos_e1), T3.dot(pos_e1);
    W3 << T1.dot(pos_e2), T3.dot(pos_e2), 0, 0,
          0, 0, T2.dot(pos_e2), T3.dot(pos_e2);

    auto theta_dot = [&](const Vector4f& th) -> Vector4f {
        Matrix<float, 2, 4> OB;
        OB << th(0), 0, th(1), 0,
              0, th(2), th(3), 0;
        return -(1.0f/Gamma) * (
            -D1.transpose()*Js1*joint_vel - D2.transpose()*Js2*joint_vel - D3.transpose()*Js3*joint_vel
            + W1.transpose()*K2*(OB*cam_Ex*pos_end - z1*y1v)
            + W2.transpose()*K2*(OB*cam_Ex*pos_e1  - z2*y2v)
            + W3.transpose()*K2*(OB*cam_Ex*pos_e2  - z3*y3v)
        );
    };

    Vector4f th_ini; th_ini << para_slow[0], para_slow[1], para_slow[2], para_slow[3];
    Vector4f t1 = theta_dot(th_ini);
    Vector4f t2 = theta_dot(th_ini + 0.5f*h*t1);
    Vector4f t3 = theta_dot(th_ini + 0.5f*h*t2);
    Vector4f t4 = theta_dot(th_ini + h*t3);
    Vector4f th_update = th_ini + (h/6.0f)*(t1 + 2.0f*t2 + 2.0f*t3 + t4);

    // --- Rho updates ---
    float tD1 = sqrtf(delta_y1.squaredNorm());
    float tD2 = sqrtf(delta_y2.squaredNorm());
    float tD3 = sqrtf(delta_y3.squaredNorm());

    float tE1 = 0.5f*B*fabsf((u1+yd[0])*delta_y1(0) + (v1+yd[1])*delta_y1(1));
    float tE2 = 0.5f*B*fabsf((u2+yd[2])*delta_y2(0) + (v2+yd[3])*delta_y2(1));
    float tE3 = 0.5f*B*fabsf((u3+yd[4])*delta_y3(0) + (v3+yd[5])*delta_y3(1));

    float rho1 = para_slow[4] + h*p.mu_rho[0]*B*(tD1+tD2+tD3);
    float rho2 = para_slow[5] + h*p.mu_rho[1]*(tE1+tE2+tE3);
    float rho3 = para_slow[6] + h*p.mu_rho[2]*fabsf(joint_vel)*B*(tD1+tD2+tD3);
    float rho4 = para_slow[7] + h*p.mu_rho[3]*fabsf(joint_vel)*(tE1+tE2+tE3);
    float rho5 = para_slow[8] + h*p.mu_rho[4]*fabsf(joint_vel);

    // --- Integrator RK4 ---
    float tau = tau_s + tau_f;
    float qc_const = joint_vel + K_dtk_I*joint_ang + omega_dtk*tau;
    auto qc_dot = [&](float qc) { return -K_dtk_I*qc + qc_const; };
    float qk1 = qc_dot(q_c);
    float qk2 = qc_dot(q_c + 0.5f*h*qk1);
    float qk3 = qc_dot(q_c + 0.5f*h*qk2);
    float qk4 = qc_dot(q_c + h*qk3);
    float qc_update = q_c + (h/6.0f)*(qk1 + 2.0f*qk2 + 2.0f*qk3 + qk4);

    // --- Outputs ---
    para_update[0]  = th_update(0);
    para_update[1]  = th_update(1);
    para_update[2]  = th_update(2);
    para_update[3]  = th_update(3);
    para_update[4]  = rho1;
    para_update[5]  = rho2;
    para_update[6]  = rho3;
    para_update[7]  = rho4;
    para_update[8]  = rho5;
    para_update[9]  = st_update(0);
    para_update[10] = st_update(1);
    para_update[11] = st_update(2);
    para_update[12] = st_update(3);
    para_update[13] = qc_update;
    para_update[14] = tau;
    para_update[15] = tau_s;
    para_update[16] = tau_f_c;

    *joint_vel_cmd = cmd_c1 * (tau_s + cmd_c2 * tau_f);
}

void cal_control(const float joint_state[2], const float img_coord[6],
                 const float obs[4], float q_c, const float ydif[6],
                 float* joint_vel_cmd, float para_update[8],
                 const ControlParams& p)
{
    const float L         = p.L;
    const float Kp        = p.Kp;
    const float Kd        = p.Kd;
    const float K4        = p.K4;
    const float eps       = p.eps;
    const float Kq        = p.Kq;
    const float cmd_c1    = p.cmd_c1;
    const float cmd_c2    = p.cmd_c2;
    const float K_dtk_I   = p.K_dtk_I;
    const float omega_dtk = p.omega_dtk;
    const float h         = p.h;
    const float Mr        = p.Mr;
    const float Jm        = p.Jm;
    const float* rt_e1    = p.rt_e1;
    const float* rt_e2    = p.rt_e2;

    // Fixed camera intrinsics (not adaptive in this controller)
    float ps[4] = { p.fx, p.cx, p.fy, p.cy };

    Matrix<float, 4, 4> cam_Ex;
    cam_Ex << p.cam_ex[0],  p.cam_ex[1],  p.cam_ex[2],  p.cam_ex[3],
              p.cam_ex[4],  p.cam_ex[5],  p.cam_ex[6],  p.cam_ex[7],
              p.cam_ex[8],  p.cam_ex[9],  p.cam_ex[10], p.cam_ex[11],
              0, 0, 0, 1;

    Vector4f T1, T2, T3;
    T1 << cam_Ex(0,0), cam_Ex(0,1), cam_Ex(0,2), cam_Ex(0,3);
    T2 << cam_Ex(1,0), cam_Ex(1,1), cam_Ex(1,2), cam_Ex(1,3);
    T3 << cam_Ex(2,0), cam_Ex(2,1), cam_Ex(2,2), cam_Ex(2,3);

    const float joint_ang = obs[0];
    const float joint_vel = obs[2];

    Matrix<float, 4, 4> Q, Qinv;
    Q << 0, 1, 0, 0,
         0, 0, 0, 1,
         (1.0f/Jm)*Kq, -(1.0f/Jm)*Kq, 0, 0,
         0, 0, (1.0f/Jm)*Kq, -(1.0f/Jm)*Kq;
    Qinv = Q.inverse();

    Vector4f Ps;
    Ps << p.Ps[0], p.Ps[1], p.Ps[2], p.Ps[3];

    Matrix<float, 1, 4> Q1, Q2, Q3, Q4;
    Q1 << Qinv(0,0), Qinv(0,1), Qinv(0,2), Qinv(0,3);
    Q2 << Qinv(1,0), Qinv(1,1), Qinv(1,2), Qinv(1,3);
    Q3 << Qinv(2,0), Qinv(2,1), Qinv(2,2), Qinv(2,3);
    Q4 << Qinv(3,0), Qinv(3,1), Qinv(3,2), Qinv(3,3);

    Vector4f pos_end, pos_e1, pos_e2;
    pos_end << L*cosf(joint_ang), L*sinf(joint_ang), 0, 1;
    pos_e1  << L*cosf(joint_ang) + rt_e1[0]*cosf(joint_ang) - rt_e1[1]*sinf(joint_ang),
               L*sinf(joint_ang) + rt_e1[0]*sinf(joint_ang) + rt_e1[1]*cosf(joint_ang), 0, 1;
    pos_e2  << L*cosf(joint_ang) + rt_e2[0]*cosf(joint_ang) - rt_e2[1]*sinf(joint_ang),
               L*sinf(joint_ang) + rt_e2[0]*sinf(joint_ang) + rt_e2[1]*cosf(joint_ang), 0, 1;

    Vector4f Js1, Js2, Js3;
    Js1 << -L*sinf(joint_ang), L*cosf(joint_ang), 0, 0;
    Js2 << -L*sinf(joint_ang) - rt_e1[0]*sinf(joint_ang) - rt_e1[1]*cosf(joint_ang),
            L*cosf(joint_ang) + rt_e1[0]*cosf(joint_ang) - rt_e1[1]*sinf(joint_ang), 0, 0;
    Js3 << -L*sinf(joint_ang) - rt_e2[0]*sinf(joint_ang) - rt_e2[1]*cosf(joint_ang),
            L*cosf(joint_ang) + rt_e2[0]*cosf(joint_ang) - rt_e2[1]*sinf(joint_ang), 0, 0;

    const float u1 = img_coord[0], v1 = img_coord[1];
    const float u2 = img_coord[2], v2 = img_coord[3];
    const float u3 = img_coord[4], v3 = img_coord[5];
    const float* yd = p.yd;

    Vector2f delta_y1, delta_y2, delta_y3;
    delta_y1 << u1 - yd[0], v1 - yd[1];
    delta_y2 << u2 - yd[2], v2 - yd[3];
    delta_y3 << u3 - yd[4], v3 - yd[5];

    Matrix<float, 2, 4> A1, A2, A3;
    A1 << ps[0]*T1(0)+(ps[1]-u1)*T3(0), ps[0]*T1(1)+(ps[1]-u1)*T3(1),
          ps[0]*T1(2)+(ps[1]-u1)*T3(2), ps[0]*T1(3)+(ps[1]-u1)*T3(3),
          ps[2]*T2(0)+(ps[3]-v1)*T3(0), ps[2]*T2(1)+(ps[3]-v1)*T3(1),
          ps[2]*T2(2)+(ps[3]-v1)*T3(2), ps[2]*T2(3)+(ps[3]-v1)*T3(3);
    A2 << ps[0]*T1(0)+(ps[1]-u2)*T3(0), ps[0]*T1(1)+(ps[1]-u2)*T3(1),
          ps[0]*T1(2)+(ps[1]-u2)*T3(2), ps[0]*T1(3)+(ps[1]-u2)*T3(3),
          ps[2]*T2(0)+(ps[3]-v2)*T3(0), ps[2]*T2(1)+(ps[3]-v2)*T3(1),
          ps[2]*T2(2)+(ps[3]-v2)*T3(2), ps[2]*T2(3)+(ps[3]-v2)*T3(3);
    A3 << ps[0]*T1(0)+(ps[1]-u3)*T3(0), ps[0]*T1(1)+(ps[1]-u3)*T3(1),
          ps[0]*T1(2)+(ps[1]-u3)*T3(2), ps[0]*T1(3)+(ps[1]-u3)*T3(3),
          ps[2]*T2(0)+(ps[3]-v3)*T3(0), ps[2]*T2(1)+(ps[3]-v3)*T3(1),
          ps[2]*T2(2)+(ps[3]-v3)*T3(2), ps[2]*T2(3)+(ps[3]-v3)*T3(3);

    const float z1 = T3.dot(pos_end);
    const float z2 = T3.dot(pos_e1);
    const float z3 = T3.dot(pos_e2);

    Vector2f AJ1 = A1 * Js1;
    Vector2f AJ2 = A2 * Js2;
    Vector2f AJ3 = A3 * Js3;

    Matrix<float, 6, 1> JR;
    JR << AJ1(0)/z1, AJ1(1)/z1, AJ2(0)/z2, AJ2(1)/z2, AJ3(0)/z3, AJ3(1)/z3;

    float detJR = JR.squaredNorm();
    Matrix<float, 1, 6> JR_p = (1.0f/detJR) * JR.transpose();

    Matrix<float, 6, 1> y_err;
    y_err << delta_y1(0), delta_y1(1), delta_y2(0), delta_y2(1), delta_y3(0), delta_y3(1);

    Matrix<float, 6, 1> dif_y;
    dif_y << ydif[0], ydif[1], ydif[2], ydif[3], ydif[4], ydif[5];

    float tau_s = (Mr + Jm) * (JR_p * (-Kp*y_err - Kd*dif_y))(0);
    tau_s = clamp(tau_s, -2.0f, 2.0f);

    float tau_f_c = -(1.0f/eps) * K4 * (obs[3] - obs[2]);
    float tau_f   = clamp(tau_f_c, -1.0f, 1.0f);

    float tau_sob = tau_s + tau_f;

    // Observer RK4
    Matrix<float, 4, 4> Fs;
    Matrix<float, 4, 1> Gs;
    Fs << 0, 0, 1, 0,
          0, 0, 0, 1,
          -(1.0f/Mr)*Kq, (1.0f/Mr)*Kq, 0, 0,
           (1.0f/Jm)*Kq,-(1.0f/Jm)*Kq, 0, 0;
    Gs << 0, 0, 0, 1.0f/Jm;

    Vector4f st_ini; st_ini << obs[0], obs[1], obs[2], obs[3];
    auto obs_dot = [&](const Vector4f& s) -> Vector4f {
        return Fs*s + Gs*tau_sob + Qinv*Ps*(joint_state[0] - s(1));
    };
    Vector4f k1 = obs_dot(st_ini);
    Vector4f k2 = obs_dot(st_ini + 0.5f*h*k1);
    Vector4f k3 = obs_dot(st_ini + 0.5f*h*k2);
    Vector4f k4 = obs_dot(st_ini + h*k3);
    Vector4f st_update = st_ini + (h/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);

    // Integrator RK4
    float tau = tau_s + tau_f;
    float qc_const = joint_vel + K_dtk_I*joint_ang + omega_dtk*tau;
    auto qc_dot = [&](float qc) { return -K_dtk_I*qc + qc_const; };
    float qk1 = qc_dot(q_c);
    float qk2 = qc_dot(q_c + 0.5f*h*qk1);
    float qk3 = qc_dot(q_c + 0.5f*h*qk2);
    float qk4 = qc_dot(q_c + h*qk3);
    float qc_update = q_c + (h/6.0f)*(qk1 + 2.0f*qk2 + 2.0f*qk3 + qk4);

    para_update[0] = st_update(0);
    para_update[1] = st_update(1);
    para_update[2] = st_update(2);
    para_update[3] = st_update(3);
    para_update[4] = qc_update;
    para_update[5] = tau;
    para_update[6] = tau_s;
    para_update[7] = tau_f_c;

    *joint_vel_cmd = cmd_c1 * (tau_s + cmd_c2 * tau_f);
}
