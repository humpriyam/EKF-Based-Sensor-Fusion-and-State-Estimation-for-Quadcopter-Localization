#include "MPU9250.h"

MPU9250 imu;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("--- Step 5 test ---"));

  MPUConfig cfg;            // defaults: 4 g, 500 dps, DLPF 3, 200 Hz
  if (!imu.begin(cfg)) {
    Serial.println(F("begin() FAILED"));
    while (1) {}
  }

  Serial.print(F("hasMag   = ")); Serial.println(imu.hasMag);
  Serial.print(F("gyroLSB  = ")); Serial.println(imu.gyroLSB);
  Serial.print(F("accelLSB = ")); Serial.println(imu.accelLSB);
  Serial.print(F("magASA   = "));
  Serial.print(imu.magASA[0], 3); Serial.print(' ');
  Serial.print(imu.magASA[1], 3); Serial.print(' ');
  Serial.println(imu.magASA[2], 3);
}

void loop() {}