function [roll,pitch,yaw] = quat2eul_zyx(q)
%QUAT2EUL_ZYX  Quaternion [w x y z] -> Euler (rad), body->world, R = Rz*Ry*Rx.
%   Output order is [roll, pitch, yaw]. Accepts row or column input.
%   Roll and yaw are ill-defined at pitch = +/-90 deg (gimbal lock).
q = q(:) / norm(q);                 % guard against a non-unit quaternion
w = q(1); x = q(2); y = q(3); z = q(4);
roll  = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
pitch = asin(max(-1, min(1, 2*(w*y - z*x))));   % clamp avoids complex values
yaw   = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
end