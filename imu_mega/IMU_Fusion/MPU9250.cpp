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
    accelScale = 1.0f;
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

// ======================= Step 6 =======================

static inline int16_t be16(uint8_t hi, uint8_t lo) {   // big-endian (MPU registers)
  return (int16_t)(((uint16_t)hi << 8) | lo);
}
static inline int16_t le16(uint8_t lo, uint8_t hi) {   // little-endian (AK8963 registers)
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

bool MPU9250::readRaw(RawData& r) {
  // accel(6) + temp(2) + gyro(6) = 14 bytes starting at ACCEL_XOUT_H (0x3B)
  uint8_t b[14];
  if (!readRegs(addr, 0x3B, b, 14)) return false;
  r.accel[0] = be16(b[0], b[1]);
  r.accel[1] = be16(b[2], b[3]);
  r.accel[2] = be16(b[4], b[5]);
  r.temp     = be16(b[6], b[7]);
  r.gyro[0]  = be16(b[8],  b[9]);
  r.gyro[1]  = be16(b[10], b[11]);
  r.gyro[2]  = be16(b[12], b[13]);

  r.mag[0] = r.mag[1] = r.mag[2] = 0;
  r.magNew = false;
  r.magOverflow = false;

  if (hasMag) {
    // ST1, HXL..HZH, ST2  (reading ST2 releases the data latch)
    uint8_t m[8];
    if (readRegs(0x0C, 0x02, m, 8)) {
      r.magNew = (m[0] & 0x01) != 0;                 // DRDY
      if (r.magNew) {
        r.mag[0] = le16(m[1], m[2]);
        r.mag[1] = le16(m[3], m[4]);
        r.mag[2] = le16(m[5], m[6]);
        r.magOverflow = (m[7] & 0x08) != 0;          // HOFL
      }
    }
  }
  return true;
}

void MPU9250::convert(const RawData& r, SensorData& d) const {
    d.accel_g  = Vec3(r.accel[0] / accelLSB * accelScale,
                    r.accel[1] / accelLSB * accelScale,
                    r.accel[2] / accelLSB * accelScale);
  d.accel_ms2 = Vec3(d.accel_g.x * G_MS2, d.accel_g.y * G_MS2, d.accel_g.z * G_MS2);

  d.gyro_dps = Vec3(r.gyro[0] / gyroLSB - gyroBias[0],
                    r.gyro[1] / gyroLSB - gyroBias[1],
                    r.gyro[2] / gyroLSB - gyroBias[2]);
  d.gyro_rads = Vec3(d.gyro_dps.x * DEG2RAD_F, d.gyro_dps.y * DEG2RAD_F, d.gyro_dps.z * DEG2RAD_F);

  // MPU-9250 datasheet: 333.87 LSB/degC, 0 LSB at 21 degC
  d.temp_C = r.temp / 333.87f + 21.0f;

  if (r.magNew && !r.magOverflow) {
    // uT in AK8963 axes
    float mx = r.mag[0] * magASA[0] * magLSB;
    float my = r.mag[1] * magASA[1] * magLSB;
    float mz = r.mag[2] * magASA[2] * magLSB;
    // AK8963 -> accel/gyro axes: swap X/Y, flip Z
    float ax = my, ay = mx, az = -mz;
    d.mag_uT = Vec3((ax - magOffset[0]) * magScale[0],
                    (ay - magOffset[1]) * magScale[1],
                    (az - magOffset[2]) * magScale[2]);
    d.magValid = true;
  } else {
    d.mag_uT = Vec3(NAN, NAN, NAN);
    d.magValid = false;
  }
}

bool MPU9250::read(SensorData& d) {
  RawData r;
  if (!readRaw(r)) return false;
  convert(r, d);
  return true;
}

// ======================= Step 7 =======================

void MPU9250::calibrateGyro(uint16_t N) {
  gyroBias[0] = gyroBias[1] = gyroBias[2] = 0;
  Serial.println(F("Gyro calibration: keep sensor still..."));
  RawData r; SensorData d;
  for (int k = 0; k < 10; k++) { readRaw(r); delay(5); }   // discard startup samples

  float sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
  uint16_t n = 0;
  for (uint16_t k = 0; k < N; k++) {
    if (!readRaw(r)) continue;
    convert(r, d);
    sum[0] += d.gyro_dps.x; sq[0] += d.gyro_dps.x * d.gyro_dps.x;
    sum[1] += d.gyro_dps.y; sq[1] += d.gyro_dps.y * d.gyro_dps.y;
    sum[2] += d.gyro_dps.z; sq[2] += d.gyro_dps.z * d.gyro_dps.z;
    n++;
    delay(5);
  }
  if (n < N / 2) { Serial.println(F("WARNING: too many I2C failures. Gyro calibration skipped.")); return; }

  float mean[3], sd[3];
  bool moved = false;
  for (int i = 0; i < 3; i++) {
    mean[i] = sum[i] / n;
    float var = sq[i] / n - mean[i] * mean[i];
    sd[i] = sqrtf(var > 0 ? var : 0);
    if (sd[i] > 0.5f) moved = true;
  }
  if (moved) Serial.println(F("WARNING: sensor moved during calibration. Redo."));
  for (int i = 0; i < 3; i++) gyroBias[i] = mean[i];
  if (fabsf(mean[0]) > 5 || fabsf(mean[1]) > 5 || fabsf(mean[2]) > 5)
    Serial.println(F("WARNING: bias outside datasheet +/-5 dps."));
  Serial.print(F("Gyro bias (dps): "));
  Serial.print(gyroBias[0], 3); Serial.print(' ');
  Serial.print(gyroBias[1], 3); Serial.print(' ');
  Serial.println(gyroBias[2], 3);
}

void MPU9250::calibrateAccel(uint16_t N) {
  accelScale = 1.0f;
  Serial.println(F("Accel calibration: keep sensor flat and still..."));
  RawData r; SensorData d;
  float sum = 0; uint16_t n = 0;
  for (uint16_t k = 0; k < N; k++) {
    if (!readRaw(r)) continue;
    convert(r, d);
    sum += vnorm(d.accel_g);
    n++;
    delay(5);
  }
  if (n < N / 2) { Serial.println(F("WARNING: accel calibration skipped.")); return; }
  float avg = sum / n;
  if (avg < 0.8f || avg > 1.2f) {
    Serial.print(F("WARNING: |a| = ")); Serial.print(avg, 3);
    Serial.println(F(" g is far from 1 g. Scale not applied."));
    return;
  }
  accelScale = 1.0f / avg;
  Serial.print(F("Accel scale = ")); Serial.println(accelScale, 4);
}

void MPU9250::calibrateMag(uint16_t seconds) {
  magOffset[0] = magOffset[1] = magOffset[2] = 0;
  magScale[0]  = magScale[1]  = magScale[2]  = 1;
  if (!hasMag) { Serial.println(F("No magnetometer: calibration skipped.")); return; }

  Serial.print(F("Mag calibration: rotate the sensor in all directions for "));
  Serial.print(seconds); Serial.println(F(" s..."));

  float mn[3] = { 1e9f,  1e9f,  1e9f};
  float mx[3] = {-1e9f, -1e9f, -1e9f};
  uint16_t n = 0;
  RawData r; SensorData d;
  unsigned long t0 = millis();
  unsigned long lastDot = t0;

  while (millis() - t0 < (unsigned long)seconds * 1000UL) {
    if (readRaw(r)) {
      convert(r, d);
      if (d.magValid && vnorm(d.mag_uT) < 200.0f) {      // ignore wild samples
        float v[3] = {d.mag_uT.x, d.mag_uT.y, d.mag_uT.z};
        for (int i = 0; i < 3; i++) {
          if (v[i] < mn[i]) mn[i] = v[i];
          if (v[i] > mx[i]) mx[i] = v[i];
        }
        n++;
      }
    }
    if (millis() - lastDot > 1000) { Serial.print('.'); lastDot = millis(); }
    delay(10);
  }
  Serial.println();

  if (n < 100) {
    Serial.print(F("WARNING: too few mag samples (")); Serial.print(n);
    Serial.println(F("). Calibration skipped."));
    return;
  }
  float rad[3];
  for (int i = 0; i < 3; i++) rad[i] = (mx[i] - mn[i]) / 2.0f;
  if (rad[0] < 5 || rad[1] < 5 || rad[2] < 5) {
    Serial.print(F("WARNING: poor coverage, radius = "));
    Serial.print(rad[0], 1); Serial.print(' ');
    Serial.print(rad[1], 1); Serial.print(' ');
    Serial.print(rad[2], 1); Serial.println(F(" uT. Calibration skipped."));
    return;
  }
  float avgRad = (rad[0] + rad[1] + rad[2]) / 3.0f;
  for (int i = 0; i < 3; i++) {
    magOffset[i] = (mx[i] + mn[i]) / 2.0f;
    magScale[i]  = avgRad / rad[i];
  }
  Serial.print(F("Mag offset (uT): "));
  Serial.print(magOffset[0], 1); Serial.print(' ');
  Serial.print(magOffset[1], 1); Serial.print(' ');
  Serial.println(magOffset[2], 1);
  Serial.print(F("Mag scale: "));
  Serial.print(magScale[0], 2); Serial.print(' ');
  Serial.print(magScale[1], 2); Serial.print(' ');
  Serial.println(magScale[2], 2);
}