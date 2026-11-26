#include "MPU9250.h"
#include "Orientation.h"
#include "ComplementaryFilter.h"
#include "QuatEKF.h"

// ================= settings (tuning options only, no calibration values) =================
const uint8_t  OUTPUT_MODE   = 0;      // 0 = Serial Plotter labels, 1 = CSV for MATLAB
const uint16_t PRINT_MS      = 50;     // output period (50 = 20 Hz)
const bool     SHOW_STATS    = false;  // true = print loop rate and EKF time every 5 s (as '#' lines)
const uint16_t GYRO_CAL_N    = 300;    // samples for gyro bias (sensor still)
const uint16_t ACCEL_CAL_N   = 200;    // samples for accel scale (sensor flat and still)
const uint16_t MAG_CAL_SEC   = 30;     // seconds of rotating for mag calibration
const int      N_INIT        = 50;     // samples averaged for the starting orientation
const float    TAU           = 0.5f;   // complementary filter time constant (s)

// EKF noise parameters (tune in Step 14): gyro noise, bias walk, accel noise, mag noise
const float    SIG_G = 0.01f, SIG_B = 1e-4f, SIG_A = 0.05f, SIG_M = 0.15f;

// ================= globals =================
MPU9250 imu;
ComplementaryFilter cf;
QuatEKF ekf;
Vec3 lastMag;
unsigned long tPrevUs = 0;
unsigned long lastPrint = 0, lastStats = 0;
unsigned long ekfUs = 0, nSteps = 0, readFails = 0;

// ================= setup =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("--- IMU fusion: raw sensor -> complementary filter + quaternion EKF ---"));

  if (!imu.begin()) { Serial.println(F("begin() FAILED")); while (1) {} }
  if (!imu.hasMag)  { Serial.println(F("No magnetometer")); while (1) {} }

  // 1) gyro + accel calibration: sensor flat and still
  Serial.println(F("Step 1/3: keep the sensor FLAT and STILL..."));
  delay(2000);
  imu.calibrateGyro(GYRO_CAL_N);
  imu.calibrateAccel(ACCEL_CAL_N);

  // 2) magnetometer calibration: rotate through every orientation
  Serial.println(F("Step 2/3: ROTATE the sensor slowly through every orientation. Starting in 3 s..."));
  delay(3000);
  imu.calibrateMag(MAG_CAL_SEC);

  // 3) initial orientation: flat and still
  Serial.println(F("Step 3/3: place the sensor FLAT and STILL. Starting in 3 s..."));
  delay(3000);

  Vec3 aSum, mSum;
  int k = 0, tries = 0;
  while (k < N_INIT && tries < 2000) {
    tries++;
    SensorData d;
    if (!imu.read(d) || !d.magValid) continue;
    aSum.x += d.accel_g.x; aSum.y += d.accel_g.y; aSum.z += d.accel_g.z;
    mSum.x += d.mag_uT.x;  mSum.y += d.mag_uT.y;  mSum.z += d.mag_uT.z;
    k++;
    delay(8);
  }
  if (k < N_INIT / 2) { Serial.println(F("Too few valid mag samples")); while (1) {} }
  Vec3 a0(aSum.x / k, aSum.y / k, aSum.z / k);
  Vec3 m0(mSum.x / k, mSum.y / k, mSum.z / k);
  Serial.print(F("Init: |a| = ")); Serial.print(vnorm(a0), 3);
  Serial.print(F(" g, |m| = "));   Serial.print(vnorm(m0), 1); Serial.println(F(" uT"));
  if (fabsf(vnorm(a0) - 1.0f) > 0.1f) Serial.println(F("WARNING: |a| not close to 1 g. Was the sensor still?"));
  if (vnorm(m0) < 20.0f || vnorm(m0) > 80.0f)
    Serial.println(F("WARNING: |m| outside 20-80 uT. Yaw correction will be rejected. Redo mag calibration."));

  lastMag = m0;
  float r0, p0, y0;
  accelMagToEuler(a0, m0, true, r0, p0, y0);
  cf.init(r0, p0, y0, TAU);

  EKFParams prm;
  prm.sigG = SIG_G; prm.sigB = SIG_B; prm.sigA = SIG_A; prm.sigM = SIG_M;
  if (!ekf.init(a0, m0, prm)) { while (1) {} }

  Serial.print(F("Jacobian check (should be < 0.01): "));
  Serial.println(ekf.jacobianCheck(), 6);

  if (OUTPUT_MODE == 1) {
    Serial.println(F("# CSV columns: $,t_ms,rollCF,pitchCF,yawCF,rollEKF,pitchEKF,yawEKF,qw,qx,qy,qz"));
  }
  Serial.println(F("Running..."));
  tPrevUs = micros();
  lastStats = millis();
}

// ================= output =================
void printOutput(const float rpyCF[3], const float rpyEKF[3]) {
  if (OUTPUT_MODE == 0) {
    // Serial Plotter format: label:value pairs
    Serial.print(F("RollCF:"));   Serial.print(rpyCF[0], 1);  Serial.print(',');
    Serial.print(F("RollEKF:"));  Serial.print(rpyEKF[0], 1); Serial.print(',');
    Serial.print(F("PitchCF:"));  Serial.print(rpyCF[1], 1);  Serial.print(',');
    Serial.print(F("PitchEKF:")); Serial.print(rpyEKF[1], 1); Serial.print(',');
    Serial.print(F("YawCF:"));    Serial.print(rpyCF[2], 1);  Serial.print(',');
    Serial.print(F("YawEKF:"));   Serial.println(rpyEKF[2], 1);
  } else {
    Quat q = ekf.quat();
    Serial.print(F("$,"));  Serial.print(millis());      Serial.print(',');
    Serial.print(rpyCF[0], 2);  Serial.print(',');
    Serial.print(rpyCF[1], 2);  Serial.print(',');
    Serial.print(rpyCF[2], 2);  Serial.print(',');
    Serial.print(rpyEKF[0], 2); Serial.print(',');
    Serial.print(rpyEKF[1], 2); Serial.print(',');
    Serial.print(rpyEKF[2], 2); Serial.print(',');
    Serial.print(q.w, 5); Serial.print(',');
    Serial.print(q.x, 5); Serial.print(',');
    Serial.print(q.y, 5); Serial.print(',');
    Serial.println(q.z, 5);
  }
}

// ================= loop =================
void loop() {
  SensorData d;
  if (!imu.read(d)) { readFails++; return; }

  unsigned long now = micros();
  float dt = clampf((now - tPrevUs) * 1e-6f, 1e-3f, 0.2f);
  tPrevUs = now;

  // EKF gets NaN when there is no NEW magnetometer sample (it skips the mag update);
  // the complementary filter uses the last good value.
  Vec3 magEKF = d.mag_uT;
  if (d.magValid) lastMag = d.mag_uT; else d.mag_uT = lastMag;

  float rpyCF[3], rpyEKF[3];
  cf.step(d.gyro_rads, d.accel_g, d.mag_uT, dt, rpyCF);

  unsigned long t0 = micros();
  ekf.step(d.gyro_rads, d.accel_g, magEKF, dt, rpyEKF);
  ekfUs += micros() - t0;
  nSteps++;

  unsigned long ms = millis();
  if (ms - lastPrint >= PRINT_MS) {
    lastPrint = ms;
    printOutput(rpyCF, rpyEKF);
  }

  if (SHOW_STATS && ms - lastStats >= 5000) {
    Serial.print(F("# loop ")); Serial.print(1000.0f * nSteps / (float)(ms - lastStats), 0);
    Serial.print(F(" Hz, EKF ")); Serial.print(ekfUs / nSteps);
    Serial.print(F(" us, read fails ")); Serial.print(readFails);
    Serial.print(F(", gyro bias est (dps) "));
    Serial.print(ekf.x[4] * RAD2DEG_F, 2); Serial.print(' ');
    Serial.print(ekf.x[5] * RAD2DEG_F, 2); Serial.print(' ');
    Serial.println(ekf.x[6] * RAD2DEG_F, 2);
    lastStats = ms; ekfUs = 0; nSteps = 0;
  }
}