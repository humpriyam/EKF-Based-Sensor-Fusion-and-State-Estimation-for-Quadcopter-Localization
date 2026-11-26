#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  Serial.println("Scanning I2C...");

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("No devices found. Check wiring.");

  // WHO_AM_I (register 0x75) on 0x68
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
  if (Wire.available()) {
    Serial.print("WHO_AM_I = 0x");
    Serial.println(Wire.read(), HEX);
  }
}

void loop() {}