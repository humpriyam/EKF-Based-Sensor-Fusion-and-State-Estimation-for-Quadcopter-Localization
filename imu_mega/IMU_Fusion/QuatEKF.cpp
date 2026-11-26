#include "QuatEKF.h"

bool QuatEKF::init(const Vec3& a, const Vec3& m, const EKFParams& p) {
  if (!vfinite(m) || vnorm(m) < 1e-6f) {
    Serial.println(F("ekf init: need a valid magnetometer sample."));
    return false;
  }
  if (fabsf(vnorm(a) - 1.0f) > 0.2f) Serial.println(F("ekf init: |accel| far from 1 g, sensor may not be still."));

  float r, pt, y;
  accelMagToEuler(a, m, true, r, pt, y);
  Quat q0 = eul2quat_zyx(r, pt, y);

  // reference field in world frame: [horizontal; 0; vertical]
  float mn = vnorm(m);
  float R[9];
  quatToRotm(q0, R);
  float mx = m.x / mn, my = m.y / mn, mz = m.z / mn;
  float h0 = R[0] * mx + R[1] * my + R[2] * mz;
  float h1 = R[3] * mx + R[4] * my + R[5] * mz;
  float h2 = R[6] * mx + R[7] * my + R[8] * mz;
  float hor = sqrtf(h0 * h0 + h1 * h1);
  float rn = sqrtf(hor * hor + h2 * h2);
  ref = Vec3(hor / rn, 0.0f, h2 / rn);

  x[0] = q0.w; x[1] = q0.x; x[2] = q0.y; x[3] = q0.z;
  x[4] = x[5] = x[6] = 0.0f;
  P.zero();
  for (uint8_t i = 0; i < 4; i++) P(i, i) = 1e-2f;
  for (uint8_t i = 4; i < 7; i++) P(i, i) = 1e-3f;
  prm = p;
  return true;
}

void QuatEKF::outputRPY(float rpy_deg[3]) const {
  float r, p, y;
  quat2eul_zyx(quat(), r, p, y);
  rpy_deg[0] = r * RAD2DEG_F; rpy_deg[1] = p * RAD2DEG_F; rpy_deg[2] = y * RAD2DEG_F;
}

// h(q): predicted unit accel (R'*[0;0;1]) and unit mag (R'*ref). Same as hMeas in ekf_step.m
void QuatEKF::hMeas(const float q[4], const Vec3& r, bool useMag, float h[6]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  h[0] = 2 * (x * z - w * y);
  h[1] = 2 * (y * z + w * x);
  h[2] = 1 - 2 * (x * x + y * y);
  if (useMag) {
    float R00 = 1 - 2 * (y * y + z * z), R01 = 2 * (x * y - w * z), R02 = 2 * (x * z + w * y);
    float R10 = 2 * (x * y + w * z), R11 = 1 - 2 * (x * x + z * z), R12 = 2 * (y * z - w * x);
    float R20 = 2 * (x * z - w * y), R21 = 2 * (y * z + w * x), R22 = 1 - 2 * (x * x + y * y);
    h[3] = R00 * r.x + R10 * r.y + R20 * r.z;     // R' * ref
    h[4] = R01 * r.x + R11 * r.y + R21 * r.z;
    h[5] = R02 * r.x + R12 * r.y + R22 * r.z;
  }
}

// Analytic Jacobian of h(q) wrt q = [w x y z] (same h as hMeas)
void QuatEKF::jacobian(const float q[4], const Vec3& r, bool useMag, float H[6][4]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  H[0][0] = -2 * y;  H[0][1] =  2 * z;  H[0][2] = -2 * w;  H[0][3] = 2 * x;
  H[1][0] =  2 * x;  H[1][1] =  2 * w;  H[1][2] =  2 * z;  H[1][3] = 2 * y;
  H[2][0] =  0;      H[2][1] = -4 * x;  H[2][2] = -4 * y;  H[2][3] = 0;
  if (!useMag) return;
  float r0 = r.x, r1 = r.y, r2 = r.z;
  H[3][0] = 2 * (r1 * z - r2 * y);
  H[3][1] = 2 * (r1 * y + r2 * z);
  H[3][2] = -4 * r0 * y + 2 * r1 * x - 2 * r2 * w;
  H[3][3] = -4 * r0 * z + 2 * r1 * w + 2 * r2 * x;
  H[4][0] = 2 * (-r0 * z + r2 * x);
  H[4][1] = 2 * r0 * y - 4 * r1 * x + 2 * r2 * w;
  H[4][2] = 2 * (r0 * x + r2 * z);
  H[4][3] = -2 * r0 * w - 4 * r1 * z + 2 * r2 * y;
  H[5][0] = 2 * (r0 * y - r1 * x);
  H[5][1] = 2 * r0 * z - 2 * r1 * w - 4 * r2 * x;
  H[5][2] = 2 * r0 * w + 2 * r1 * z - 4 * r2 * y;
  H[5][3] = 2 * (r0 * x + r1 * y);
}

float QuatEKF::jacobianCheck() const {
  float Ha[6][4];
  jacobian(x, ref, true, Ha);
  float err = 0;
  const float e = 1e-3f;
  for (uint8_t k = 0; k < 4; k++) {
    float qp[4] = {x[0], x[1], x[2], x[3]};
    float qm[4] = {x[0], x[1], x[2], x[3]};
    qp[k] += e; qm[k] -= e;
    float hp[6], hm[6];
    hMeas(qp, ref, true, hp);
    hMeas(qm, ref, true, hm);
    for (uint8_t i = 0; i < 6; i++) {
      float d = fabsf((hp[i] - hm[i]) / (2 * e) - Ha[i][k]);
      if (d > err) err = d;
    }
  }
  return err;
}

void QuatEKF::step(const Vec3& g, const Vec3& a, const Vec3& m, float dt, float rpy_deg[3]) {
  if (!isfinite(dt) || dt <= 0 || !vfinite(g)) { outputRPY(rpy_deg); return; }   // bad input: no step
  if (dt > 0.2f) dt = 0.2f;

  const float qw = x[0], qx = x[1], qy = x[2], qz = x[3];
  const float w0 = g.x - x[4], w1 = g.y - x[5], w2 = g.z - x[6];
  const float h2 = 0.5f * dt;

  // ---------------- predict state: exact rotation step ----------------
  float n = sqrtf(w0 * w0 + w1 * w1 + w2 * w2);
  Quat dq;
  if (n * dt > 1e-9f) {
    float ang = 0.5f * n * dt, s = sinf(ang) / n;
    dq = Quat(cosf(ang), s * w0, s * w1, s * w2);
  } else {
    dq = Quat(1, h2 * w0, h2 * w1, h2 * w2);
  }
  Quat qn = qnormalize(qmul(Quat(qw, qx, qy, qz), dq));

  // ---------------- predict covariance: F = [A B; 0 I] ----------------
  float A[4][4] = {{1,       -h2 * w0, -h2 * w1, -h2 * w2},
                   {h2 * w0,  1,        h2 * w2, -h2 * w1},
                   {h2 * w1, -h2 * w2,  1,        h2 * w0},
                   {h2 * w2,  h2 * w1, -h2 * w0,  1}};
  float Xi[4][3] = {{-qx, -qy, -qz},
                    { qw, -qz,  qy},
                    { qz,  qw, -qx},
                    {-qy,  qx,  qw}};
  float B[4][3];
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 3; j++) B[i][j] = -h2 * Xi[i][j];

  float M11[4][4], M12[4][3];                       // M = F*P (top rows)
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      float s = 0;
      for (uint8_t k = 0; k < 4; k++) s += A[i][k] * P.m[k][j];
      for (uint8_t k = 0; k < 3; k++) s += B[i][k] * P.m[4 + k][j];
      M11[i][j] = s;
    }
    for (uint8_t j = 0; j < 3; j++) {
      float s = 0;
      for (uint8_t k = 0; k < 4; k++) s += A[i][k] * P.m[k][4 + j];
      for (uint8_t k = 0; k < 3; k++) s += B[i][k] * P.m[4 + k][4 + j];
      M12[i][j] = s;
    }
  }
  const float cq = h2 * h2 * prm.sigG * prm.sigG;   // Qq = (0.5dt)^2 sigG^2 Xi Xi'
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = i; j < 4; j++) {
      float s = 0, qq = 0;
      for (uint8_t k = 0; k < 4; k++) s += M11[i][k] * A[j][k];
      for (uint8_t k = 0; k < 3; k++) { s += M12[i][k] * B[j][k]; qq += Xi[i][k] * Xi[j][k]; }
      s += cq * qq;
      P.m[i][j] = s; P.m[j][i] = s;
    }
    for (uint8_t j = 0; j < 3; j++) { P.m[i][4 + j] = M12[i][j]; P.m[4 + j][i] = M12[i][j]; }
  }
  const float qb = prm.sigB * prm.sigB * dt;        // bias random walk (P22 otherwise unchanged)
  for (uint8_t i = 4; i < 7; i++) P.m[i][i] += qb;

  x[0] = qn.w; x[1] = qn.x; x[2] = qn.y; x[3] = qn.z;

  // ---------------- update ----------------
  float na = vnorm(a);
  bool useA = vfinite(a) && fabsf(na - 1.0f) < 0.5f;
  float mn = vfinite(m) ? vnorm(m) : 0.0f;
  bool useMag = useA && vfinite(m) && mn > 20.0f && mn < 80.0f;

  if (useA) {
    uint8_t nz = useMag ? 6 : 3;
    float z[6], Rd[6];
    z[0] = a.x / na; z[1] = a.y / na; z[2] = a.z / na;
    float sA = prm.sigA * (1.0f + 20.0f * fabsf(na - 1.0f));   // distrust accel when |a| != 1 g
    Rd[0] = Rd[1] = Rd[2] = sA * sA;
    if (useMag) {
      z[3] = m.x / mn; z[4] = m.y / mn; z[5] = m.z / mn;
      Rd[3] = Rd[4] = Rd[5] = prm.sigM * prm.sigM;
    }

    float h0[6], H[6][4];
    hMeas(x, ref, useMag, h0);
    jacobian(x, ref, useMag, H);

    const float xp0 = x[0], xp1 = x[1], xp2 = x[2], xp3 = x[3];
    for (uint8_t i = 0; i < nz; i++) {                 // sequential scalar updates (R diagonal)
      float PH[7];
      for (uint8_t r = 0; r < 7; r++)
        PH[r] = P.m[r][0] * H[i][0] + P.m[r][1] * H[i][1] + P.m[r][2] * H[i][2] + P.m[r][3] * H[i][3];
      float S = Rd[i] + H[i][0] * PH[0] + H[i][1] * PH[1] + H[i][2] * PH[2] + H[i][3] * PH[3];
      if (S < 1e-12f) continue;
      float y = z[i] - h0[i]
              - (H[i][0] * (x[0] - xp0) + H[i][1] * (x[1] - xp1) + H[i][2] * (x[2] - xp2) + H[i][3] * (x[3] - xp3));
      float invS = 1.0f / S;
      for (uint8_t r = 0; r < 7; r++) {
        float K = PH[r] * invS;
        x[r] += K * y;
        for (uint8_t c = r; c < 7; c++) P.m[r][c] -= K * PH[c];   // upper triangle only
      }
      for (uint8_t r = 0; r < 7; r++)                  // mirror so P stays symmetric
        for (uint8_t c = r + 1; c < 7; c++) P.m[c][r] = P.m[r][c];
    }
    Quat qf = qnormalize(Quat(x[0], x[1], x[2], x[3]));
    x[0] = qf.w; x[1] = qf.x; x[2] = qf.y; x[3] = qf.z;
  }
  outputRPY(rpy_deg);
}