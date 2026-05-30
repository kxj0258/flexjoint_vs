#include "controller.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include <cmath>

using namespace Eigen;

namespace {

float clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

int normalized_feature_count(int feature_count)
{
    if (feature_count < kMinFeaturePoints)
        return kMinFeaturePoints;
    if (feature_count > kMaxFeaturePoints)
        return kMaxFeaturePoints;
    return feature_count;
}

struct FeatureKinematics {
    Vector4f pos;
    Vector4f Js;
};

FeatureKinematics feature_kinematics(float L, float joint_ang,
                                     const float offset[3])
{
    const float c = cosf(joint_ang);
    const float s = sinf(joint_ang);
    const float ox = offset[0];
    const float oy = offset[1];
    const float oz = offset[2];

    FeatureKinematics out;
    out.pos << L*c + ox*c - oy*s,
               L*s + ox*s + oy*c,
               oz,
               1.0f;
    out.Js << -L*s - ox*s - oy*c,
               L*c + ox*c - oy*s,
               0.0f,
               0.0f;
    return out;
}

Matrix<float, 2, 4> projection_matrix(const Vector4f& T1,
                                      const Vector4f& T2,
                                      const Vector4f& T3,
                                      const float theta[4],
                                      float u, float v)
{
    Matrix<float, 2, 4> A;
    A << theta[0]*T1(0) + (theta[1] - u)*T3(0),
         theta[0]*T1(1) + (theta[1] - u)*T3(1),
         theta[0]*T1(2) + (theta[1] - u)*T3(2),
         theta[0]*T1(3) + (theta[1] - u)*T3(3),
         theta[2]*T2(0) + (theta[3] - v)*T3(0),
         theta[2]*T2(1) + (theta[3] - v)*T3(1),
         theta[2]*T2(2) + (theta[3] - v)*T3(2),
         theta[2]*T2(3) + (theta[3] - v)*T3(3);
    return A;
}

Matrix<float, 4, 4> theta_regressor_D(const Vector4f& T1,
                                      const Vector4f& T2,
                                      const Vector4f& T3,
                                      float B,
                                      const Vector2f& delta_y)
{
    Matrix<float, 4, 4> D;
    D << T1(0)*B*delta_y(0), T3(0)*B*delta_y(0),
         T2(0)*B*delta_y(1), T3(0)*B*delta_y(1),
         T1(1)*B*delta_y(0), T3(1)*B*delta_y(0),
         T2(1)*B*delta_y(1), T3(1)*B*delta_y(1),
         T1(2)*B*delta_y(0), T3(2)*B*delta_y(0),
         T2(2)*B*delta_y(1), T3(2)*B*delta_y(1),
         T1(3)*B*delta_y(0), T3(3)*B*delta_y(0),
         T2(3)*B*delta_y(1), T3(3)*B*delta_y(1);
    return D;
}

Matrix<float, 2, 4> theta_regressor_W(const Vector4f& T1,
                                      const Vector4f& T2,
                                      const Vector4f& T3,
                                      const Vector4f& pos)
{
    Matrix<float, 2, 4> W;
    W << T1.dot(pos), T3.dot(pos), 0.0f, 0.0f,
         0.0f, 0.0f, T2.dot(pos), T3.dot(pos);
    return W;
}

void observer_matrices(float Kq, float Mr, float Jm,
                       Matrix<float, 4, 4>& Qinv,
                       Matrix<float, 4, 4>& Fs,
                       Matrix<float, 4, 1>& Gs)
{
    Matrix<float, 4, 4> Q;
    Q << 0, 1, 0, 0,
         0, 0, 0, 1,
         (1.0f/Jm)*Kq, -(1.0f/Jm)*Kq, 0, 0,
         0, 0, (1.0f/Jm)*Kq, -(1.0f/Jm)*Kq;
    Qinv = Q.inverse();

    Fs << 0, 0, 1, 0,
          0, 0, 0, 1,
          -(1.0f/Mr)*Kq, (1.0f/Mr)*Kq, 0, 0,
           (1.0f/Jm)*Kq,-(1.0f/Jm)*Kq, 0, 0;
    Gs << 0, 0, 0, 1.0f/Jm;
}

Vector4f integrate_observer(const float obs[4], const float joint_state[2],
                            const Matrix<float, 4, 4>& Qinv,
                            const Matrix<float, 4, 4>& Fs,
                            const Matrix<float, 4, 1>& Gs,
                            const Vector4f& Ps,
                            float tau_sob, float h)
{
    Vector4f st_ini;
    st_ini << obs[0], obs[1], obs[2], obs[3];
    auto obs_dot = [&](const Vector4f& s) -> Vector4f {
        return Fs*s + Gs*tau_sob + Qinv*Ps*(joint_state[0] - s(1));
    };
    Vector4f k1 = obs_dot(st_ini);
    Vector4f k2 = obs_dot(st_ini + 0.5f*h*k1);
    Vector4f k3 = obs_dot(st_ini + 0.5f*h*k2);
    Vector4f k4 = obs_dot(st_ini + h*k3);
    return st_ini + (h/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);
}

float integrate_qc(float q_c, float joint_vel, float joint_ang,
                   float tau, float K_dtk_I, float omega_dtk, float h)
{
    const float qc_const = joint_vel + K_dtk_I*joint_ang + omega_dtk*tau;
    auto qc_dot = [&](float qc) { return -K_dtk_I*qc + qc_const; };
    float qk1 = qc_dot(q_c);
    float qk2 = qc_dot(q_c + 0.5f*h*qk1);
    float qk3 = qc_dot(q_c + 0.5f*h*qk2);
    float qk4 = qc_dot(q_c + h*qk3);
    return q_c + (h/6.0f)*(qk1 + 2.0f*qk2 + 2.0f*qk3 + qk4);
}

} // namespace

void cal_joint_vel(const float joint_state[2], const float img_coord[6],
                   const float obs[4], const float para_slow[9], float q_c,
                   float* joint_vel_cmd, float para_update[17],
                   const ControlParams& p)
{
    cal_joint_vel_features(joint_state, img_coord, kLegacyFeaturePoints,
                           obs, para_slow, q_c, joint_vel_cmd,
                           para_update, p);
}

void cal_joint_vel_features(const float joint_state[2], const float* img_coord,
                            int feature_count,
                            const float obs[4], const float para_slow[9],
                            float q_c, float* joint_vel_cmd,
                            float para_update[17],
                            const ControlParams& p)
{
    feature_count = normalized_feature_count(feature_count);

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

    Matrix<float, 4, 4> Qinv, Fs;
    Matrix<float, 4, 1> Gs;
    observer_matrices(Kq, Mr, Jm, Qinv, Fs, Gs);

    Vector4f Ps;
    Ps << p.Ps[0], p.Ps[1], p.Ps[2], p.Ps[3];

    Matrix<float, 1, 4> Q1, Q2, Q3, Q4;
    Q1 << Qinv(0,0), Qinv(0,1), Qinv(0,2), Qinv(0,3);
    Q2 << Qinv(1,0), Qinv(1,1), Qinv(1,2), Qinv(1,3);
    Q3 << Qinv(2,0), Qinv(2,1), Qinv(2,2), Qinv(2,3);
    Q4 << Qinv(3,0), Qinv(3,1), Qinv(3,2), Qinv(3,3);

    const float theta[4] = {
        para_slow[0], para_slow[1], para_slow[2], para_slow[3]
    };

    std::array<Vector4f, kMaxFeaturePoints> pos;
    std::array<Vector4f, kMaxFeaturePoints> Js;
    std::array<Vector2f, kMaxFeaturePoints> delta_y;
    std::array<Vector2f, kMaxFeaturePoints> yv;
    std::array<Matrix<float, 2, 4>, kMaxFeaturePoints> A;
    std::array<Matrix<float, 4, 4>, kMaxFeaturePoints> D;
    std::array<Matrix<float, 2, 4>, kMaxFeaturePoints> W;
    std::array<float, kMaxFeaturePoints> z;

    float image_term_sum = 0.0f;
    float tD_sum = 0.0f;
    float tE_sum = 0.0f;
    for (int i = 0; i < feature_count; ++i) {
        const FeatureKinematics kin =
            feature_kinematics(L, joint_ang, p.feature_offsets[i]);
        pos[i] = kin.pos;
        Js[i] = kin.Js;

        const float u = img_coord[2 * i];
        const float v = img_coord[2 * i + 1];
        yv[i] << u, v;
        delta_y[i] << u - p.yd[2 * i], v - p.yd[2 * i + 1];

        A[i] = projection_matrix(T1, T2, T3, theta, u, v);
        D[i] = theta_regressor_D(T1, T2, T3, B, delta_y[i]);
        W[i] = theta_regressor_W(T1, T2, T3, pos[i]);
        z[i] = T3.dot(pos[i]);

        Matrix<float, 1, 1> image_term =
            Js[i].transpose() *
            (A[i].transpose() + 0.5f*T3*delta_y[i].transpose()) *
            B * delta_y[i];
        image_term_sum += image_term(0);

        tD_sum += sqrtf(delta_y[i].squaredNorm());
        tE_sum += 0.5f*B*fabsf(
            (u + p.yd[2 * i]) * delta_y[i](0) +
            (v + p.yd[2 * i + 1]) * delta_y[i](1));
    }

    Matrix<float, 1, 4> c_slow_Q = Mr*Q3 + Jm*Q4 - Jm*(Q2 - Q1)*Ps*Q2;
    float c_slow = (c_slow_Q(0)*Ps(0) + c_slow_Q(1)*Ps(1) +
                    c_slow_Q(2)*Ps(2) + c_slow_Q(3)*Ps(3)) *
                   (joint_state[0] - obs[1]);

    float tau_s = (Mr + Jm) * (-K1 * joint_vel - image_term_sum) - c_slow;
    tau_s = clamp(tau_s, -2.0f, 2.0f);

    float tau_f_c = -(1.0f/eps) * K4 * (obs[3] - obs[2]);
    float tau_f   = clamp(tau_f_c, -1.0f, 1.0f);

    float tau_sob = tau_s + tau_f;
    Vector4f st_update = integrate_observer(obs, joint_state, Qinv, Fs, Gs,
                                            Ps, tau_sob, h);

    auto theta_dot = [&](const Vector4f& th) -> Vector4f {
        Matrix<float, 2, 4> OB;
        OB << th(0), 0, th(1), 0,
              0, th(2), th(3), 0;
        Vector4f sum = Vector4f::Zero();
        for (int i = 0; i < feature_count; ++i) {
            sum += -D[i].transpose()*Js[i]*joint_vel;
            sum += W[i].transpose()*K2*
                (OB*cam_Ex*pos[i] - z[i]*yv[i]);
        }
        return -(1.0f/Gamma) * sum;
    };

    Vector4f th_ini;
    th_ini << para_slow[0], para_slow[1], para_slow[2], para_slow[3];
    Vector4f t1 = theta_dot(th_ini);
    Vector4f t2 = theta_dot(th_ini + 0.5f*h*t1);
    Vector4f t3 = theta_dot(th_ini + 0.5f*h*t2);
    Vector4f t4 = theta_dot(th_ini + h*t3);
    Vector4f th_update = th_ini + (h/6.0f)*(t1 + 2.0f*t2 + 2.0f*t3 + t4);

    float rho1 = para_slow[4] + h*p.mu_rho[0]*B*tD_sum;
    float rho2 = para_slow[5] + h*p.mu_rho[1]*tE_sum;
    float rho3 = para_slow[6] + h*p.mu_rho[2]*fabsf(joint_vel)*B*tD_sum;
    float rho4 = para_slow[7] + h*p.mu_rho[3]*fabsf(joint_vel)*tE_sum;
    float rho5 = para_slow[8] + h*p.mu_rho[4]*fabsf(joint_vel);

    float tau = tau_s + tau_f;
    float qc_update = integrate_qc(q_c, joint_vel, joint_ang, tau,
                                   K_dtk_I, omega_dtk, h);

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
    cal_control_features(joint_state, img_coord, kLegacyFeaturePoints,
                         obs, q_c, ydif, joint_vel_cmd, para_update, p);
}

void cal_control_features(const float joint_state[2], const float* img_coord,
                          int feature_count,
                          const float obs[4], float q_c,
                          const float* ydif,
                          float* joint_vel_cmd, float para_update[8],
                          const ControlParams& p)
{
    feature_count = normalized_feature_count(feature_count);

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

    Matrix<float, 4, 4> Qinv, Fs;
    Matrix<float, 4, 1> Gs;
    observer_matrices(Kq, Mr, Jm, Qinv, Fs, Gs);

    Vector4f Ps;
    Ps << p.Ps[0], p.Ps[1], p.Ps[2], p.Ps[3];

    Matrix<float, Dynamic, 1> JR(2 * feature_count);
    Matrix<float, Dynamic, 1> y_err(2 * feature_count);
    Matrix<float, Dynamic, 1> dif_y(2 * feature_count);

    for (int i = 0; i < feature_count; ++i) {
        const FeatureKinematics kin =
            feature_kinematics(L, joint_ang, p.feature_offsets[i]);
        const float u = img_coord[2 * i];
        const float v = img_coord[2 * i + 1];
        Vector2f delta_y;
        delta_y << u - p.yd[2 * i], v - p.yd[2 * i + 1];

        Matrix<float, 2, 4> A = projection_matrix(T1, T2, T3, ps, u, v);
        const float z = T3.dot(kin.pos);
        Vector2f AJ = A * kin.Js;

        JR(2 * i) = AJ(0) / z;
        JR(2 * i + 1) = AJ(1) / z;
        y_err(2 * i) = delta_y(0);
        y_err(2 * i + 1) = delta_y(1);
        dif_y(2 * i) = ydif[2 * i];
        dif_y(2 * i + 1) = ydif[2 * i + 1];
    }

    float detJR = JR.squaredNorm();
    if (detJR < 1e-8f) {
        detJR = 1e-8f;
    }
    Matrix<float, 1, Dynamic> JR_p = (1.0f/detJR) * JR.transpose();

    float tau_s = (Mr + Jm) * (JR_p * (-Kp*y_err - Kd*dif_y))(0);
    tau_s = clamp(tau_s, -2.0f, 2.0f);

    float tau_f_c = -(1.0f/eps) * K4 * (obs[3] - obs[2]);
    float tau_f   = clamp(tau_f_c, -1.0f, 1.0f);

    float tau_sob = tau_s + tau_f;
    Vector4f st_update = integrate_observer(obs, joint_state, Qinv, Fs, Gs,
                                            Ps, tau_sob, h);

    float tau = tau_s + tau_f;
    float qc_update = integrate_qc(q_c, joint_vel, joint_ang, tau,
                                   K_dtk_I, omega_dtk, h);

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
