#pragma once

// Jacobian matrix calculations for the soft robot arm camera model.
//
// Both functions compute a 41-element output array encoding the image-space
// Jacobian for three feature points observed by the camera.
//
// Output layout (Qm/Qs, 41 elements):
//   [0..11]  : row 1 of Jacobian (feature point 1, u-coordinate)
//   [12..17] : row 2 (feature point 1, v-coordinate)
//   [18..23] : row 3 (feature point 2, u-coordinate)
//   [24..29] : row 4 (feature point 2, v-coordinate)
//   [30..35] : row 5 (feature point 3, u-coordinate)
//   [36..40] : predicted image coordinates y1h..y5h

// Fast-dynamics Jacobian.
// known[23]: L, PHI, WR, rt_e1[3], rt_e2[3], eps, Kq, T1[4], T2[4], T3[4]
// qk[2]    : theta (joint angle), ksi (joint velocity)
// p_est[6] : x1, x2, c1, c2, c3, c4 (estimated parameters)
// Qm[41]   : output Jacobian array
void cal_jacobian_fast(const float known[23], const float qk[2],
                       const float p_est[6], float Qm[41]);

// Slow-dynamics (quasi-static) Jacobian.
// slow_known[19]: L, rt_e1[3], rt_e2[3], T1[4], T2[4], T3[4]
// slow_est[6]   : theta, x2, c1, c2, c3, c4
// Qs[41]        : output Jacobian array
void cal_jacobian_slow(const float slow_known[19], const float slow_est[6],
                       float Qs[41]);
