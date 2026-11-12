function q = eul2quat_zyx(roll, pitch, yaw)
%EUL2QUAT_ZYX  Euler (rad) -> quaternion [w x y z], body->world, R = Rz*Ry*Rx.
cr = cos(roll/2);  sr = sin(roll/2);
cp = cos(pitch/2); sp = sin(pitch/2);
cy = cos(yaw/2);   sy = sin(yaw/2);
q = [cr*cp*cy + sr*sp*sy;
     sr*cp*cy - cr*sp*sy;
     cr*sp*cy + sr*cp*sy;
     cr*cp*sy - sr*sp*cy];
q = q/norm(q);
end
