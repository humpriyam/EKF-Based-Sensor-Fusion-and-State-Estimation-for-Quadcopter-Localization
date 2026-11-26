#include "MathUtils.h"

void setup() {
  Serial.begin(115200);

  // 1) identity quaternion -> identity matrix
  float R[9];
  quatToRotm(Quat(1, 0, 0, 0), R);
  Serial.print("Identity R diag: ");
  Serial.print(R[0]); Serial.print(' ');
  Serial.print(R[4]); Serial.print(' ');
  Serial.println(R[8]);

  // 2) 90 deg about Z: q = [cos45, 0, 0, sin45]  -> R[3] should be 1, R[1] should be -1
  float c = cosf(PI_F / 4), s = sinf(PI_F / 4);
  quatToRotm(Quat(c, 0, 0, s), R);
  Serial.print("90deg about Z: R[1]=");
  Serial.print(R[1]); Serial.print("  R[3]=");
  Serial.println(R[3]);

  // 3) qmul of two 90 deg-about-Z rotations = 180 deg about Z -> [0 0 0 1]
  Quat q = qmul(Quat(c, 0, 0, s), Quat(c, 0, 0, s));
  Serial.print("qmul result: ");
  Serial.print(q.w); Serial.print(' ');
  Serial.print(q.x); Serial.print(' ');
  Serial.print(q.y); Serial.print(' ');
  Serial.println(q.z);

  // 4) wrapPi(190 deg) should be -170 deg
  Serial.print("wrapPi(190deg) in deg = ");
  Serial.println(wrapPi(190 * DEG2RAD_F) * RAD2DEG_F);
}

void loop() {}