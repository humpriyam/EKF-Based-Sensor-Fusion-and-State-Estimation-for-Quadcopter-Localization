# EKF-Based Sensor Fusion and State Estimation for Quadcopter Localization

This repository contains MATLAB and Simulink models covering quadcopter kinematics, Blade Element Theory (BET) and lumped-parameter propeller modeling, and real-time 9-DOF IMU sensor fusion using a custom 7-state Quaternion Extended Kalman Filter (EKF) and 3D digital-twin visualization.

## Repository Structure

### 📁 /Kinematics
* `rot2d.m`: MATLAB script for visualizing 2D planar coordinate transformations and vector rotations.
* `rot3d.m`: MATLAB script for computing and visualizing 3D rigid body rotations using Euler angles to map the Body-Fixed Frame to the Inertial Frame.

### 📁 /Propulsion_Modeling
* `th_to_r.m`: Evaluates and plots thrust and torque distribution along the radial length of a single propeller blade.
* `th_to_rpm.m`: Calculates total quadcopter thrust and torque across varying motor RPMs using strict integral Blade Element Theory (BET) and a Simplified Lumped-Parameter Model, which collapses the complex integrals into dimensionless thrust ($C_T$) and torque ($C_Q$) coefficients.

### 📁 /Simulink_Models
* `final.slx`: A Simulink environment designed to cross-verify the analytical BET calculations and Lumped-Parameter models directly against the built-in Simulink Multirotor dynamics physics block.

### 📁 /Sensor_Fusion
* `ekf_comp_inbuild.m`: MATLAB script for real-time MPU6050 sensor data acquisition via Arduino, comparing raw accelerometer and gyroscope readings against MATLAB's built-in Complementary and Extended Kalman Filters (EKF) for roll, pitch, and yaw estimation.
* `ekf_comp_manual.m`: MATLAB script that implements a custom Complementary Filter and a 2-state linear Kalman Filter (tracking angle and gyro bias) from scratch, benchmarking their real-time performance directly against MATLAB's built-in sensor fusion algorithms.

### 📁 /IMU
A complete 14-file MATLAB suite for 9-DOF MPU-9250/9265 hardware interfacing, sensor calibration, 7-state quaternion EKF attitude estimation, and live 3D orientation visualization via Arduino I2C:

#### Hardware Driver & Calibration
* `mpu9250_init.m`: Scans I2C addresses (`0x68`/`0x69`), verifies `WHO_AM_I`, configures accelerometer/gyroscope full-scale ranges and digital low-pass filters, and enables I2C bypass mode to initialize the AK8963 magnetometer (`0x0C`) at 100 Hz with factory sensitivity adjustments (`ASA`)[cite: 8].
* `mpu9250_readRaw.m`: Performs burst I2C reads for raw 16-bit accelerometer, temperature, and gyroscope counts (`0x3B`), along with AK8963 magnetometer counts (`0x02`), checking data-ready (`DRDY`) and overflow (`HOFL`) flags[cite: 9].
* `mpu9250_convert.m`: Converts raw counts into physical units ($g$, $\text{m/s}^2$, $\text{dps}$, $\text{rad/s}$, $^\circ\text{C}$, and $\mu\text{T}$), removes gyroscope bias, aligns AK8963 magnetometer axes to the accelerometer/gyroscope frame, and applies hard-iron and soft-iron corrections[cite: 7].
* `mpu9250_calibrateGyro.m`: Captures stationary samples to verify sensor stillness and compute static 3-axis gyroscope bias in $\text{dps}$[cite: 5].
* `mpu9250_calibrateMag.m`: Records 30 seconds of multi-axis rotations using percentile outlier rejection to calculate magnetometer hard-iron offsets and soft-iron scaling factors[cite: 6].

#### State Estimation & Attitude Math
* `ekf_init.m`: Initializes the 7-state Quaternion EKF ($x = [q_0, q_1, q_2, q_3, b_x, b_y, b_z]^T$) and error covariance $P$ from stationary averaged accelerometer and magnetometer readings, establishing the world magnetic reference vector in a ZYX (Magnetic North / Up) frame[cite: 2].
* `ekf_step.m`: Executes one predict-and-update cycle of the 7-state Quaternion EKF using exponential-map quaternion integration, dynamic accelerometer covariance scaling/gating ($|a| \approx 1g$), magnetometer outlier gating ($20\text{–}80\ \mu\text{T}$), numerical Jacobian computation, and quaternion re-normalization[cite: 3].
* `complementary_filter.m`: Implements a discrete-time 3-axis ZYX Euler complementary filter ($\alpha = \tau / (\tau + \Delta t)$) with $\pm 85^\circ$ gimbal-lock clamping, dynamic acceleration rejection, and tilt-compensated magnetometer yaw updates[cite: 14].
* `accelMagToEuler.m`: Computes direct (unfiltered) roll and pitch from gravity and tilt-compensated yaw from magnetometer readings[cite: 1].
* `eul2quat_zyx.m`: Converts ZYX Euler angles (radians) into a normalized scalar-first `[w x y z]` unit quaternion[cite: 4].
* `quat2eul_zyx.m`: Converts a scalar-first `[w x y z]` unit quaternion into ZYX Euler angles (radians) with `asin` clamping near $\pm 90^\circ$ pitch[cite: 10].

#### Execution & Live 3D Visualization
* `main_imu_fusion.m`: Main hardware-in-the-loop acquisition script that calibrates the MPU-9250, plots live Roll/Pitch/Yaw (`Raw` vs. `Complementary` vs. `EKF`), logs all sensor and attitude streams to `imu_log.mat`, and computes RMS angle differences[cite: 12].
* `imu_visualizer.m`: Renders a dual-subplot live 3D orientation view comparing the Quaternion EKF (left) and Complementary Filter (right) side by side with relative start-heading zeroing[cite: 11].
* `main_imuFinal.m`: Renders a 3D hardware-accurate digital twin of the MPU-92/65 breakout PCB ($15.5 \times 25.5 \times 1.6\text{ mm}$), overlaying the solid blue Quaternion EKF board against a transparent green Complementary Filter ghost board in real time[cite: 13].

## Future Work Roadmap

The upcoming phases of the project will focus on full vehicle dynamics, closed-loop control, and position localization:
1. **Non-Linear 6-DOF Equations:** Replacing small-angle approximations with strict Newton-Euler formulations for translational and rotational motion.
2. **Gyroscopic Torque Integration:** Mathematically modeling the gyroscopic effect generated by the high-speed rotation of the propeller masses during pitch and roll maneuvers.
3. **Full Position & Velocity EKF Localization:** Extending the current 7-state attitude EKF by fusing accelerometer linear accelerations with GPS/barometer/optical-flow measurements to estimate 3D quadcopter position and velocity.
4. **Closed-Loop Flight Controller Design:** Implementing cascaded PID/LQR attitude and altitude controllers in Simulink using the EKF state estimates.
