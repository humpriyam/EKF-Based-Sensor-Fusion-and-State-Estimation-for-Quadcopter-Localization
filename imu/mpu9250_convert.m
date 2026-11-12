function d = mpu9250_convert(raw, s)
%MPU9250_CONVERT  Raw counts -> physical units.
%   d.accel_g, d.accel_ms2   [1x3]  linear acceleration
%   d.gyro_dps, d.gyro_rads  [1x3]  angular velocity (bias removed)
%   d.temp_C                        die temperature
%   d.mag_uT [1x3], d.magValid      magnetic field, axes aligned to accel/gyro

g = 9.80665;
d.accel_g   = raw.accel / s.accelLSB;
d.accel_ms2 = d.accel_g * g;

d.gyro_dps  = raw.gyro / s.gyroLSB - s.gyroBias;
d.gyro_rads = deg2rad(d.gyro_dps);

% MPU-9250 datasheet: 333.87 LSB/degC, 0 LSB at 21 degC (differs from MPU-6050!)
d.temp_C = raw.temp/333.87 + 21;

if raw.magNew && ~raw.magOverflow
    m = raw.mag .* s.magASA * s.magLSB;      % uT in AK8963 axes
    m = [m(2) m(1) -m(3)];                   % AK8963 -> accel/gyro axes (swap X/Y, flip Z)
    d.mag_uT   = (m - s.magOffset) .* s.magScale;
    d.magValid = true;
else
    d.mag_uT   = [NaN NaN NaN];
    d.magValid = false;
end
end
