#ifndef QUAT_EKF_H
#define QUAT_EKF_H

#include "MathUtils.h"
#include "Orientation.h"
#include "Matrix.h"

struct EKFParams {
  float sigG;   // gyro noise [rad/s]
  float sigB;   // gyro bias random walk
  float sigA;   // accel measurement noise (unit-vector units)
  float sigM;   // mag measurement noise   (unit-vector units)
  EKFParams() : sigG(0.01f), sigB(1e-4f), sigA(0.05f), sigM(0.15f) {}
};

class QuatEKF {
public:
  float      x[7];     // [q0 q1 q2 q3 bx by bz], q body->world, bias rad/s
  Mat<7, 7>  P;
  Vec3       ref;      // unit reference mag vector in world frame
  EKFParams  prm;

  // Call with the sensor still, ideally with averaged accel/mag.
  // Returns false if the mag sample is invalid.
  bool init(const Vec3& accel_g, const Vec3& mag_uT,
            const EKFParams& p = EKFParams());

  // One predict + update cycle. Pass mag = NaN to skip the mag update.
  void step(const Vec3& gyro_rads, const Vec3& accel_g, const Vec3& mag_uT,
            float dt, float rpy_deg[3]);

  Quat quat() const { return Quat(x[0], x[1], x[2], x[3]); }

  // max |analytic - numeric| Jacobian difference at the current state (should be ~1e-3 or less)
  float jacobianCheck() const;

private:
  static void hMeas(const float q[4], const Vec3& r, bool useMag, float h[6]);
  static void jacobian(const float q[4], const Vec3& r, bool useMag, float H[6][4]);
  void outputRPY(float rpy_deg[3]) const;
};

#endif