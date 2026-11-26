#include "Orientation.h"

void accelToRollPitch(const Vec3& a, float& roll, float& pitch) {
  roll  = atan2f(a.y, a.z);
  pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));
}

void accelMagToEuler(const Vec3& a, const Vec3& m, bool useMag,
                     float& roll, float& pitch, float& yaw) {
  accelToRollPitch(a, roll, pitch);
  yaw = NAN;
  if (useMag && vfinite(m)) {
    float sr = sinf(roll),  cr = cosf(roll);
    float sp = sinf(pitch), cp = cosf(pitch);
    float hx = m.x * cp + sp * (m.y * sr + m.z * cr);
    float hy = m.y * cr - m.z * sr;
    yaw = atan2f(-hy, hx);   // 0 = magnetic north
  }
}

Quat eul2quat_zyx(float roll, float pitch, float yaw) {
  float cr = cosf(roll / 2),  sr = sinf(roll / 2);
  float cp = cosf(pitch / 2), sp = sinf(pitch / 2);
  float cy = cosf(yaw / 2),   sy = sinf(yaw / 2);
  Quat q(cr * cp * cy + sr * sp * sy,
         sr * cp * cy - cr * sp * sy,
         cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy);
  return qnormalize(q);
}

void quat2eul_zyx(const Quat& qin, float& roll, float& pitch, float& yaw) {
  Quat q = qnormalize(qin);
  float w = q.w, x = q.x, y = q.y, z = q.z;
  roll  = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  pitch = asinf(clampf(2 * (w * y - z * x), -1.0f, 1.0f));
  yaw   = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}