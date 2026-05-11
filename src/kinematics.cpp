#include "kinematics.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix;
using Eigen::Vector4f;

void cal_jacobian_fast(const float known[23], const float qk[2],
                       const float p_est[6], float Qm[41])
{
    const float L    = known[0];
    const float PHI  = known[1];
    const float WR   = known[2];
    const float eps  = known[9];
    const float Kq   = known[10];

    float rt_e1[3] = { known[3], known[4], known[5] };
    float rt_e2[3] = { known[6], known[7], known[8] };

    Vector4f T1, T2, T3;
    T1 << known[11], known[12], known[13], known[14];
    T2 << known[15], known[16], known[17], known[18];
    T3 << known[19], known[20], known[21], known[22];

    const float theta = qk[0];
    const float ksi   = qk[1];
    const float x1    = p_est[0];
    const float x2    = p_est[1];
    const float c1    = p_est[2];
    const float c2    = p_est[3];
    const float c3    = p_est[4];
    const float c4    = p_est[5];

    const float delta = (x1 + ksi) / Kq;

    Vector4f X_F1, X_F2, X_F3;
    X_F1 << L * cosf(theta) - sinf(theta) * PHI * delta,
            L * sinf(theta) + cosf(theta) * PHI * delta,
            0, 1;
    X_F2 << L * cosf(theta) - sinf(theta) * PHI * delta
            + rt_e1[0] * cosf(theta + WR * delta) - rt_e1[1] * sinf(theta + WR * delta),
            L * sinf(theta) + cosf(theta) * PHI * delta
            + rt_e1[0] * sinf(theta + WR * delta) + rt_e1[1] * cosf(theta + WR * delta),
            0, 1;
    X_F3 << L * cosf(theta) - sinf(theta) * PHI * delta
            + rt_e2[0] * cosf(theta + WR * delta) - rt_e2[1] * sinf(theta + WR * delta),
            L * sinf(theta) + cosf(theta) * PHI * delta
            + rt_e2[0] * sinf(theta + WR * delta) + rt_e2[1] * cosf(theta + WR * delta),
            0, 1;

    Vector4f m1 = c1 * T1 + c2 * T3;
    Vector4f m2 = c3 * T2 + c4 * T3;
    Vector4f m3 = T3;

    const float z1 = m3.dot(X_F1);
    const float z2 = m3.dot(X_F2);
    const float z3 = m3.dot(X_F3);

    Matrix<float, 4, 4> cam_Ext;
    cam_Ext << known[11], known[12], known[13], known[14],
               known[15], known[16], known[17], known[18],
               known[19], known[20], known[21], known[22],
               0, 0, 0, 1;

    Vector4f R1 = cam_Ext * X_F1;
    Vector4f R2 = cam_Ext * X_F2;
    Vector4f R3 = cam_Ext * X_F3;

    Vector4f Jf1, Jf2, Jf3;
    Jf1 << -sinf(theta) * PHI, cosf(theta) * PHI, 0, 0;
    Jf2 << -sinf(theta) * PHI + (-rt_e1[0] * sinf(theta + WR * delta) - rt_e1[1] * cosf(theta + WR * delta)) * WR,
            cosf(theta) * PHI + ( rt_e1[0] * cosf(theta + WR * delta) - rt_e1[1] * sinf(theta + WR * delta)) * WR,
            0, 0;
    Jf3 << -sinf(theta) * PHI + (-rt_e2[0] * sinf(theta + WR * delta) - rt_e2[1] * cosf(theta + WR * delta)) * WR,
            cosf(theta) * PHI + ( rt_e2[0] * cosf(theta + WR * delta) - rt_e2[1] * sinf(theta + WR * delta)) * WR,
            0, 0;

    const float y1h = (1.0f / z1) * m1.dot(X_F1);
    const float y2h = (1.0f / z1) * m2.dot(X_F1);
    const float y3h = (1.0f / z2) * m1.dot(X_F2);
    const float y4h = (1.0f / z2) * m2.dot(X_F2);
    const float y5h = (1.0f / z3) * m1.dot(X_F3);

    Qm[0]  = (1.0f / (z1 * eps * Kq)) * (m1 - y1h * m3).dot(Jf1);
    Qm[1]  = 0;
    Qm[2]  = (1.0f / z1) * R1(0);
    Qm[3]  = 1;
    Qm[4]  = 0;
    Qm[5]  = 0;

    float Qm6_1 = (m1 - y1h * m3).dot(Jf1) * m3.dot(Jf1);
    Qm[6]  = -2.0f * (1.0f / z1) * (1.0f / z1) * Qm6_1
             * (1.0f / (eps * Kq)) * (1.0f / (eps * Kq)) * x2;
    Qm[7]  = (1.0f / (z1 * eps * Kq)) * (m1 - y1h * m3).dot(Jf1);

    Qm[8]  = (1.0f / (z1 * eps * Kq)) * x2 * T1.dot(Jf1)
             - (1.0f / (z1 * z1)) * R1(0) * (1.0f / (eps * Kq)) * (1.0f / (eps * Kq)) * x2 * T3.dot(Jf1);
    Qm[9]  = 0;
    Qm[10] = 0;
    Qm[11] = 0;

    Qm[12] = (1.0f / (z1 * eps * Kq)) * (m2 - y2h * m3).dot(Jf1);
    Qm[13] = 0;
    Qm[14] = 0;
    Qm[15] = 0;
    Qm[16] = (1.0f / z1) * R1(1);
    Qm[17] = 1;

    Qm[18] = (1.0f / (z2 * eps * Kq)) * (m1 - y3h * m3).dot(Jf2);
    Qm[19] = 0;
    Qm[20] = (1.0f / z2) * R2(0);
    Qm[21] = 1;
    Qm[22] = 0;
    Qm[23] = 0;

    Qm[24] = (1.0f / (z2 * eps * Kq)) * (m2 - y4h * m3).dot(Jf2);
    Qm[25] = 0;
    Qm[26] = 0;
    Qm[27] = 0;
    Qm[28] = (1.0f / z2) * R2(1);
    Qm[29] = 1;

    Qm[30] = (1.0f / (z3 * eps * Kq)) * (m1 - y5h * m3).dot(Jf3);
    Qm[31] = 0;
    Qm[32] = (1.0f / z3) * R3(0);
    Qm[33] = 1;
    Qm[34] = 0;
    Qm[35] = 0;

    Qm[36] = y1h;
    Qm[37] = y2h;
    Qm[38] = y3h;
    Qm[39] = y4h;
    Qm[40] = y5h;
}


void cal_jacobian_slow(const float slow_known[19], const float slow_est[6],
                       float Qs[41])
{
    const float L = slow_known[0];
    float rt_e1[3] = { slow_known[1], slow_known[2], slow_known[3] };
    float rt_e2[3] = { slow_known[4], slow_known[5], slow_known[6] };

    Vector4f T1, T2, T3;
    T1 << slow_known[7],  slow_known[8],  slow_known[9],  slow_known[10];
    T2 << slow_known[11], slow_known[12], slow_known[13], slow_known[14];
    T3 << slow_known[15], slow_known[16], slow_known[17], slow_known[18];

    const float theta = slow_est[0];
    const float x2    = slow_est[1];
    const float c1    = slow_est[2];
    const float c2    = slow_est[3];
    const float c3    = slow_est[4];
    const float c4    = slow_est[5];

    Vector4f X_F1, X_F2, X_F3;
    X_F1 << L * cosf(theta), L * sinf(theta), 0, 1;
    X_F2 << L * cosf(theta) + rt_e1[0] * cosf(theta) - rt_e1[1] * sinf(theta),
            L * sinf(theta) + rt_e1[0] * sinf(theta) + rt_e1[1] * cosf(theta),
            0, 1;
    X_F3 << L * cosf(theta) + rt_e2[0] * cosf(theta) - rt_e2[1] * sinf(theta),
            L * sinf(theta) + rt_e2[0] * sinf(theta) + rt_e2[1] * cosf(theta),
            0, 1;

    Vector4f m1 = c1 * T1 + c2 * T3;
    Vector4f m2 = c3 * T2 + c4 * T3;
    Vector4f m3 = T3;

    const float z1 = m3.dot(X_F1);
    const float z2 = m3.dot(X_F2);
    const float z3 = m3.dot(X_F3);

    Matrix<float, 4, 4> cam_Ext;
    cam_Ext << slow_known[7],  slow_known[8],  slow_known[9],  slow_known[10],
               slow_known[11], slow_known[12], slow_known[13], slow_known[14],
               slow_known[15], slow_known[16], slow_known[17], slow_known[18],
               0, 0, 0, 1;

    Vector4f R1 = cam_Ext * X_F1;
    Vector4f R2 = cam_Ext * X_F2;
    Vector4f R3 = cam_Ext * X_F3;

    Vector4f Js1, Js2, Js3;
    Js1 << -L * sinf(theta), L * cosf(theta), 0, 0;
    Js2 << -L * sinf(theta) + (-rt_e1[0] * sinf(theta) - rt_e1[1] * cosf(theta)),
            L * cosf(theta) + ( rt_e1[0] * cosf(theta) - rt_e1[1] * sinf(theta)),
            0, 0;
    Js3 << -L * sinf(theta) + (-rt_e2[0] * sinf(theta) - rt_e2[1] * cosf(theta)),
            L * cosf(theta) + ( rt_e2[0] * cosf(theta) - rt_e2[1] * sinf(theta)),
            0, 0;

    const float y1h = (1.0f / z1) * m1.dot(X_F1);
    const float y2h = (1.0f / z1) * m2.dot(X_F1);
    const float y3h = (1.0f / z2) * m1.dot(X_F2);
    const float y4h = (1.0f / z2) * m2.dot(X_F2);
    const float y5h = (1.0f / z3) * m1.dot(X_F3);

    Qs[0]  = (1.0f / z1) * (m1 - y1h * m3).dot(Js1);
    Qs[1]  = 0;
    Qs[2]  = (1.0f / z1) * R1(0);
    Qs[3]  = 1;
    Qs[4]  = 0;
    Qs[5]  = 0;

    float Qs6_1 = (m1 - y1h * m3).dot(Js1) * m3.dot(Js1);
    Vector4f Jd1;
    Jd1 << -L * cosf(theta), -L * sinf(theta), 0, 0;
    float Qs6_2 = (m1 - y1h * m3).dot(Jd1);
    Qs[6]  = -2.0f * (1.0f / (z1 * z1)) * Qs6_1 * x2 + (1.0f / z1) * Qs6_2 * x2;
    Qs[7]  = (1.0f / z1) * (m1 - y1h * m3).dot(Js1);

    Qs[8]  = (1.0f / z1) * x2 * T1.dot(Js1)
             - (1.0f / (z1 * z1)) * R1(0) * x2 * T3.dot(Js1);
    Qs[9]  = 0;
    Qs[10] = 0;
    Qs[11] = 0;

    Qs[12] = (1.0f / z1) * (m2 - y2h * m3).dot(Js1);
    Qs[13] = 0;
    Qs[14] = 0;
    Qs[15] = 0;
    Qs[16] = (1.0f / z1) * R1(1);
    Qs[17] = 1;

    Qs[18] = (1.0f / z2) * (m1 - y3h * m3).dot(Js2);
    Qs[19] = 0;
    Qs[20] = (1.0f / z2) * R2(0);
    Qs[21] = 1;
    Qs[22] = 0;
    Qs[23] = 0;

    Qs[24] = (1.0f / z2) * (m2 - y4h * m3).dot(Js2);
    Qs[25] = 0;
    Qs[26] = 0;
    Qs[27] = 0;
    Qs[28] = (1.0f / z2) * R2(1);
    Qs[29] = 1;

    Qs[30] = (1.0f / z3) * (m1 - y5h * m3).dot(Js3);
    Qs[31] = 0;
    Qs[32] = (1.0f / z3) * R3(0);
    Qs[33] = 1;
    Qs[34] = 0;
    Qs[35] = 0;

    Qs[36] = y1h;
    Qs[37] = y2h;
    Qs[38] = y3h;
    Qs[39] = y4h;
    Qs[40] = y5h;
}
