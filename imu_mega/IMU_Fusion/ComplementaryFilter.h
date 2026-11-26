#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "MathUtils.h"
#include "Orientation.h"

class ComplementaryFilter {
public:
  float roll, pitch, yaw;   // rad
  float tau;                // time constant (s). Larger = trust gyro longer

  ComplementaryFilter() : roll(0), pitch(0), yaw(0), tau(0.5f) {}

  void init(float r0, float p0, float y0, float tau_s = 0.5f) {
    roll = r0; pitch = p0; yaw = y0; tau = tau_s;
  }

  // One step. gyro in rad/s, accel in g, mag in uT (pass last good value;
  // NaN or out-of-range skips the yaw correction). Output rpy_deg[3].
  void step(const Vec3& gyro_rads, const Vec3& accel_g, const Vec3& mag_uT,
            float dt, float rpy_deg[3]);
};

#endif