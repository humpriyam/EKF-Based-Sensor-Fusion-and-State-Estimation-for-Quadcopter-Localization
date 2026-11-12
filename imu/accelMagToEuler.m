function [roll,pitch,yaw] = accelMagToEuler(accel, mag)
%ACCELMAGTOEULER  Direct (unfiltered) angles in radians. ZYX convention, Z up.
%   roll/pitch from gravity, yaw from tilt-compensated magnetometer.
%   yaw = NaN if mag is missing/invalid.
roll  = atan2(accel(2), accel(3));
pitch = atan2(-accel(1), sqrt(accel(2)^2 + accel(3)^2));
yaw = NaN;
if nargin > 1 && all(isfinite(mag))
    sr = sin(roll); cr = cos(roll); sp = sin(pitch); cp = cos(pitch);
    hx = mag(1)*cp + sp*(mag(2)*sr + mag(3)*cr);
    hy = mag(2)*cr - mag(3)*sr;
    yaw = atan2(-hy, hx);        % 0 = magnetic north, positive counter-clockwise
end
end
