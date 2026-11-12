function [cf, rpy_deg] = complementary_filter(cf, gyro_rads, accel_g, mag_uT, dt)
%COMPLEMENTARY_FILTER  One step of a 3-axis complementary filter.
%   cf: struct with fields roll,pitch,yaw (rad) and tau (s, time constant)
%   Gyro is integrated (high-pass); accel/mag angles correct the drift (low-pass).
%   alpha = tau/(tau+dt). Larger tau = trust gyro longer.
%   Yaw correction uses the FILTERED roll/pitch for tilt compensation and is
%   skipped when |mag| is outside the plausible Earth-field range.
wrap  = @(x) atan2(sin(x), cos(x));
alpha = cf.tau/(cf.tau + dt);
phi = cf.roll; th = cf.pitch; psi = cf.yaw;
gx = gyro_rads(1); gy = gyro_rads(2); gz = gyro_rads(3);
% body rates -> Euler rates (ZYX), pitch kept away from +-90 deg
thc = max(min(th, deg2rad(85)), deg2rad(-85));
phiDot = gx + sin(phi)*tan(thc)*gy + cos(phi)*tan(thc)*gz;
thDot  = cos(phi)*gy - sin(phi)*gz;
psiDot = (sin(phi)*gy + cos(phi)*gz)/cos(thc);
phiP = phi + phiDot*dt;                      % gyro prediction
thP  = th  + thDot*dt;
psiP = psi + psiDot*dt;
[rA, pA] = accelMagToEuler(accel_g);         % roll/pitch only (no mag arg)
% ignore accel when the sensor is accelerating hard (|a| far from 1 g)
if abs(norm(accel_g) - 1) < 0.2
    phi = phiP + (1-alpha)*wrap(rA - phiP);
    th  = thP  + (1-alpha)*wrap(pA - thP);
else
    phi = phiP; th = thP;
end
% yaw from mag, tilt-compensated with the filtered phi/th
magOK = all(isfinite(mag_uT)) && norm(mag_uT) > 20 && norm(mag_uT) < 80;
if magOK
    hx = mag_uT(1)*cos(th) + sin(th)*(mag_uT(2)*sin(phi) + mag_uT(3)*cos(phi));
    hy = mag_uT(2)*cos(phi) - mag_uT(3)*sin(phi);
    yM = atan2(-hy, hx);
    psi = psiP + (1-alpha)*wrap(yM - psiP);
else
    psi = psiP;
end
cf.roll = wrap(phi); cf.pitch = th; cf.yaw = wrap(psi);
rpy_deg = rad2deg([cf.roll cf.pitch cf.yaw]);
end