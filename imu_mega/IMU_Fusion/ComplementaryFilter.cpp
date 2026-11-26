#include "ComplementaryFilter.h"

void ComplementaryFilter::step(const Vec3& g, const Vec3& a, const Vec3& m,
                               float dt, float rpy_deg[3]) {
  float alpha = tau / (tau + dt);
  float phi = roll, th = pitch, psi = yaw;

  // body rates -> Euler rates (ZYX), pitch kept away from +-90 deg
  float thc = clampf(th, -85.0f * DEG2RAD_F, 85.0f * DEG2RAD_F);
  float sphi = sinf(phi), cphi = cosf(phi);
  float tth = tanf(thc), cth = cosf(thc);
  float phiDot = g.x + sphi * tth * g.y + cphi * tth * g.z;
  float thDot  = cphi * g.y - sphi * g.z;
  float psiDot = (sphi * g.y + cphi * g.z) / cth;

  // gyro prediction
  float phiP = phi + phiDot * dt;
  float thP  = th  + thDot  * dt;
  float psiP = psi + psiDot * dt;

  // roll/pitch from accel (ignored when sensor accelerates hard)
  float rA, pA;
  accelToRollPitch(a, rA, pA);
  if (fabsf(vnorm(a) - 1.0f) < 0.2f) {
    phi = phiP + (1 - alpha) * wrapPi(rA - phiP);
    th  = thP  + (1 - alpha) * wrapPi(pA - thP);
  } else {
    phi = phiP;
    th  = thP;
  }

  // yaw from mag, tilt-compensated with the filtered phi/th
  float mn = vnorm(m);
  bool magOK = vfinite(m) && mn > 20.0f && mn < 80.0f;
  if (magOK) {
    float hx = m.x * cosf(th) + sinf(th) * (m.y * sinf(phi) + m.z * cosf(phi));
    float hy = m.y * cosf(phi) - m.z * sinf(phi);
    float yM = atan2f(-hy, hx);
    psi = psiP + (1 - alpha) * wrapPi(yM - psiP);
  } else {
    psi = psiP;
  }

  roll = wrapPi(phi); pitch = th; yaw = wrapPi(psi);
  rpy_deg[0] = roll * RAD2DEG_F;
  rpy_deg[1] = pitch * RAD2DEG_F;
  rpy_deg[2] = yaw * RAD2DEG_F;
}