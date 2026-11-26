#ifndef ORIENTATION_H
#define ORIENTATION_H

#include "MathUtils.h"

// Direct (unfiltered) angles from accel (+ optional mag), radians.
// roll/pitch from gravity, yaw from tilt-compensated magnetometer.
// Pass useMag = false (or an invalid mag) and yaw is returned as NAN.
void accelMagToEuler(const Vec3& accel, const Vec3& mag, bool useMag,
                     float& roll, float& pitch, float& yaw);

// Roll/pitch only (no magnetometer)
void accelToRollPitch(const Vec3& accel, float& roll, float& pitch);

// Euler (rad) -> quaternion [w x y z], body->world, R = Rz*Ry*Rx
Quat eul2quat_zyx(float roll, float pitch, float yaw);

// Quaternion -> Euler (rad), output [roll, pitch, yaw]
void quat2eul_zyx(const Quat& q, float& roll, float& pitch, float& yaw);

#endif