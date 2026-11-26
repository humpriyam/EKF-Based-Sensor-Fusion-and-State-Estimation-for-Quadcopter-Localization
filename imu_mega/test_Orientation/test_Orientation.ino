#include "Orientation.h"

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("--- Step 4 test ---");

  // 1) Sensor flat: accel = (0,0,1) -> roll 0, pitch 0
  float r, p, y;
  accelMagToEuler(Vec3(0, 0, 1), Vec3(20, 0, -40), true, r, p, y);
  Serial.print("Flat:   roll/pitch/yaw (deg) = ");
  Serial.print(r * RAD2DEG_F); Serial.print(' ');
  Serial.print(p * RAD2DEG_F); Serial.print(' ');
  Serial.println(y * RAD2DEG_F);

  // 2) Tilted 30 deg roll: accel = (0, sin30, cos30) -> roll 30
  accelMagToEuler(Vec3(0, 0.5f, 0.8660254f), Vec3(20, 0, -40), true, r, p, y);
  Serial.print("Roll30: roll/pitch (deg)     = ");
  Serial.print(r * RAD2DEG_F); Serial.print(' ');
  Serial.println(p * RAD2DEG_F);

  // 3) Round trip: euler -> quat -> euler (30, 20, 100 deg)
  Quat q = eul2quat_zyx(30 * DEG2RAD_F, 20 * DEG2RAD_F, 100 * DEG2RAD_F);
  quat2eul_zyx(q, r, p, y);
  Serial.print("Round trip (30 20 100):      = ");
  Serial.print(r * RAD2DEG_F); Serial.print(' ');
  Serial.print(p * RAD2DEG_F); Serial.print(' ');
  Serial.println(y * RAD2DEG_F);
}

void loop() {}