#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <Arduino.h>
#include <math.h>

// ---------- constants ----------
static const float PI_F       = 3.14159265358979f;
static const float DEG2RAD_F  = PI_F / 180.0f;
static const float RAD2DEG_F  = 180.0f / PI_F;
static const float G_MS2      = 9.80665f;

// ---------- 3-vector ----------
struct Vec3 {
  float x, y, z;
  Vec3() : x(0), y(0), z(0) {}
  Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

inline float vnorm(const Vec3& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline bool vfinite(const Vec3& v) {
  return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// ---------- quaternion [w x y z] ----------
struct Quat {
  float w, x, y, z;
  Quat() : w(1), x(0), y(0), z(0) {}
  Quat(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}
};

inline float qnorm(const Quat& q) {
  return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

inline Quat qnormalize(const Quat& q) {
  float n = qnorm(q);
  if (n < 1e-12f) return Quat(1, 0, 0, 0);
  return Quat(q.w / n, q.x / n, q.y / n, q.z / n);
}

// Hamilton product a (x) b  (same as qmul in ekf_step.m)
inline Quat qmul(const Quat& a, const Quat& b) {
  return Quat(
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
  );
}

// 3x3 rotation matrix, body -> world (same as rotm in the MATLAB files)
// stored row-major: R[0..2] = row 0, R[3..5] = row 1, R[6..8] = row 2
inline void quatToRotm(const Quat& qin, float R[9]) {
  Quat q = qnormalize(qin);
  float w = q.w, x = q.x, y = q.y, z = q.z;
  R[0] = 1 - 2 * (y * y + z * z);  R[1] = 2 * (x * y - w * z);      R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z);      R[4] = 1 - 2 * (x * x + z * z);  R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y);      R[7] = 2 * (y * z + w * x);      R[8] = 1 - 2 * (x * x + y * y);
}

// ---------- angle helpers ----------
// wrap to [-pi, pi]  (MATLAB: atan2(sin(x), cos(x)))
inline float wrapPi(float a) {
  return atan2f(sinf(a), cosf(a));
}

// wrap to [-180, 180] degrees
inline float wrap180(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

#endif