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

class MPU9250 {
public:
  // ---- state (the MATLAB struct 's') ----
  uint8_t addr;          // 0x68 or 0x69
  uint8_t whoami;
  bool    hasMag;
  float   gyroLSB;       // LSB per (deg/s)
  float   accelLSB;      // LSB per g
  float   magLSB;        // uT per LSB (16-bit mode)
  float   magASA[3];     // factory sensitivity adjustment
  float   gyroBias[3];   // deg/s
  float   magOffset[3];  // uT, hard-iron
  float   magScale[3];   // soft-iron scale

  MPU9250();

  // returns true on success; magnetometer failure is NOT fatal (hasMag = false)
  bool begin(const MPUConfig& cfg = MPUConfig());

  // low-level helpers (public so later steps and tests can use them)
  void    writeReg(uint8_t dev, uint8_t reg, uint8_t val);
  uint8_t readReg(uint8_t dev, uint8_t reg);
  bool    readRegs(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t n);

private:
  bool initMag();
};

#endif