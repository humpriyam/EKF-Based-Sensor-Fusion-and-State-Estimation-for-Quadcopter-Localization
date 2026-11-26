#include "MPU9250.h"

static const uint8_t AK8963_ADDR = 0x0C;

MPU9250::MPU9250()
  : addr(0x68), whoami(0), hasMag(false),
    gyroLSB(65.5f), accelLSB(8192.0f), magLSB(0.15f) {
  for (int i = 0; i < 3; i++) {
    magASA[i] = 1.0f;
    gyroBias[i] = 0.0f;
    magOffset[i] = 0.0f;
    magScale[i] = 1.0f;
  }
}

void MPU9250::writeReg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t MPU9250::readReg(uint8_t dev, uint8_t reg) {
  uint8_t v = 0;
  readRegs(dev, reg, &v, 1);
  return v;
}

bool MPU9250::readRegs(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  uint8_t got = Wire.requestFrom(dev, n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool MPU9250::begin(const MPUConfig& cfg) {
  Wire.begin();
  Wire.setClock(400000);

  // 1) find the MPU on 0x68 or 0x69
  bool found = false;
  const uint8_t addrs[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t w;
    if (readRegs(addrs[i], 0x75, &w, 1)) {
      addr = addrs[i];
      whoami = w;
      found = true;
      break;
    }
  }
  if (!found) {
    Serial.println(F("ERROR: MPU not found on 0x68/0x69. Check SDA(20)/SCL(21)/VCC/GND."));
    return false;
  }
  Serial.print(F("MPU found at 0x")); Serial.print(addr, HEX);
  Serial.print(F(", WHO_AM_I = 0x"));  Serial.print(whoami, HEX);
  if (whoami == 0x71 || whoami == 0x73)      Serial.println(F(" (MPU-9250/9255/9265, OK)"));
  else if (whoami == 0x70)                   Serial.println(F(" (MPU-6500: no magnetometer!)"));
  else                                       Serial.println(F(" (unknown clone, continuing)"));

  // 2) reset and configure
  writeReg(addr, 0x6B, 0x80); delay(100);   // device reset
  writeReg(addr, 0x6B, 0x01); delay(50);    // wake, clock = PLL
  writeReg(addr, 0x19, cfg.sampleDiv);      // SMPLRT_DIV
  writeReg(addr, 0x1A, cfg.dlpfCfg);        // CONFIG: gyro DLPF

  int8_t gIdx = -1, aIdx = -1;
  const uint16_t gFS[4] = {250, 500, 1000, 2000};
  const uint8_t  aFS[4] = {2, 4, 8, 16};
  for (int i = 0; i < 4; i++) {
    if (gFS[i] == cfg.gyroFS)  gIdx = i;
    if (aFS[i] == cfg.accelFS) aIdx = i;
  }
  if (gIdx < 0 || aIdx < 0) {
    Serial.println(F("ERROR: invalid full-scale option."));
    return false;
  }
  writeReg(addr, 0x1B, (uint8_t)(gIdx << 3));   // GYRO_CONFIG
  writeReg(addr, 0x1C, (uint8_t)(aIdx << 3));   // ACCEL_CONFIG
  writeReg(addr, 0x1D, 0x03);                   // ACCEL_CONFIG2: accel DLPF ~44 Hz

  const float gyroLSBs[4] = {131.0f, 65.5f, 32.8f, 16.4f};
  const float accLSBs[4]  = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
  gyroLSB  = gyroLSBs[gIdx];
  accelLSB = accLSBs[aIdx];

  // 3) magnetometer via I2C bypass
  hasMag = initMag();
  if (!hasMag) Serial.println(F("WARNING: magnetometer not found. Yaw will be gyro-only (drifts)."));
  return true;
}

bool MPU9250::initMag() {
  writeReg(addr, 0x6A, 0x00);    // I2C master off
  writeReg(addr, 0x37, 0x02);    // INT_PIN_CFG: BYPASS_EN -> host sees AK8963
  delay(10);

  uint8_t wia = 0;
  if (!readRegs(AK8963_ADDR, 0x00, &wia, 1) || wia != 0x48) return false;   // WIA must be 0x48

  writeReg(AK8963_ADDR, 0x0B, 0x01); delay(10);   // soft reset
  writeReg(AK8963_ADDR, 0x0A, 0x00); delay(10);   // power down
  writeReg(AK8963_ADDR, 0x0A, 0x0F); delay(10);   // fuse ROM access

  uint8_t asa[3];
  if (!readRegs(AK8963_ADDR, 0x10, asa, 3)) return false;
  for (int i = 0; i < 3; i++) magASA[i] = ((float)asa[i] - 128.0f) / 256.0f + 1.0f;

  writeReg(AK8963_ADDR, 0x0A, 0x00); delay(10);   // power down
  writeReg(AK8963_ADDR, 0x0A, 0x16); delay(10);   // 16-bit, continuous mode 2 (100 Hz)

  Serial.print(F("AK8963 magnetometer OK (ASA = "));
  Serial.print(asa[0]); Serial.print(' ');
  Serial.print(asa[1]); Serial.print(' ');
  Serial.print(asa[2]); Serial.println(')');
  return true;
}