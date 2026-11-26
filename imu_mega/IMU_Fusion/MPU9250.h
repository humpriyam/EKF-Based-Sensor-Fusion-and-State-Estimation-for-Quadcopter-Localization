#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h>
#include "MathUtils.h"

// ---- options (same defaults as mpu9250_init.m) ----
struct MPUConfig {
  uint8_t accelFS;    // 2, 4, 8, 16 g          default 4
  uint16_t gyroFS;    // 250, 500, 1000, 2000   default 500
  uint8_t dlpfCfg;    // 0..6 gyro DLPF         default 3 (41 Hz)
  uint8_t sampleDiv;  // rate = 1kHz/(1+div)    default 4 (200 Hz)
  MPUConfig() : accelFS(4), gyroFS(500), dlpfCfg(3), sampleDiv(4) {}
};

// ---- raw counts (MATLAB: raw struct) ----
struct RawData {
  int16_t accel[3];
  int16_t gyro[3];
  int16_t temp;
  int16_t mag[3];        // AK8963 axes
  bool    magNew;        // DRDY
  bool    magOverflow;   // HOFL
};

// ---- physical units (MATLAB: d struct) ----
struct SensorData {
  Vec3  accel_g;
  Vec3  accel_ms2;
  Vec3  gyro_dps;        // bias removed
  Vec3  gyro_rads;
  Vec3  mag_uT;          // aligned to accel/gyro axes, NaN when invalid
  float temp_C;
  bool  magValid;
};

class MPU9250 {
public:
  // ---- state (the MATLAB struct 's') ----
  uint8_t addr;          // 0x68 or 0x69
  uint8_t whoami;
  bool    hasMag;
  float   gyroLSB;       // LSB per (deg/s)
  float   accelLSB;      // LSB per g
  float   accelScale;    // multiplies accel_g, set by calibrateAccel (default 1)
  float   magLSB;        // uT per LSB (16-bit mode)
  float   magASA[3];     // factory sensitivity adjustment
  float   gyroBias[3];   // deg/s
  float   magOffset[3];  // uT, hard-iron
  float   magScale[3];   // soft-iron scale

  MPU9250();

  bool begin(const MPUConfig& cfg = MPUConfig());

  // Step 6
  bool readRaw(RawData& r);                       // false on I2C failure
  void convert(const RawData& r, SensorData& d) const;
  bool read(SensorData& d);                       // readRaw + convert
    // Step 7
  void calibrateGyro(uint16_t N = 300);           // keep sensor STILL
  void calibrateAccel(uint16_t N = 200);          // sensor flat and STILL
  void calibrateMag(uint16_t seconds = 30);       // rotate in ALL directions

  // low-level helpers
  void    writeReg(uint8_t dev, uint8_t reg, uint8_t val);
  uint8_t readReg(uint8_t dev, uint8_t reg);
  bool    readRegs(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t n);

private:
  bool initMag();
};

#endif