function ekf = ekf_init(accel_g, mag_uT, params)
%EKF_INIT  Initialise a quaternion EKF. State x = [q0 q1 q2 q3 bx by bz]'
%   (orientation body->world, gyro bias in rad/s).  World: X = magnetic north, Z = up.
%   Call with the sensor STILL, using averaged accel/mag samples if possible.
%   params (optional): sigG gyro noise [rad/s], sigB bias random walk,
%                      sigA accel meas. noise, sigM mag meas. noise (unit-vector units)
if nargin < 3, params = struct(); end
def = struct('sigG',0.01,'sigB',1e-4,'sigA',0.05,'sigM',0.15);
f = fieldnames(def);
for i = 1:numel(f)
    if ~isfield(params,f{i}), params.(f{i}) = def.(f{i}); end
end
if nargin < 2 || ~all(isfinite(mag_uT)) || norm(mag_uT) < 1e-6
    error('ekf_init: need a valid magnetometer sample to build the reference field.');
end
if abs(norm(accel_g) - 1) > 0.2
    warning('ekf_init: |accel| = %.2f g, sensor may not be still.', norm(accel_g));
end
[r,p,y] = accelMagToEuler(accel_g, mag_uT);
q0 = eul2quat_zyx(r,p,y);
% reference magnetic vector in the world frame (unit): [horizontal; 0; vertical]
mn = mag_uT(:)/norm(mag_uT);
h = rotm(q0)*mn;
ref = [sqrt(h(1)^2 + h(2)^2); 0; h(3)];
ekf.x = [q0(:); 0; 0; 0];
ekf.P = diag([1e-2 1e-2 1e-2 1e-2 1e-3 1e-3 1e-3]);
ekf.ref = ref/norm(ref);
ekf.sigG = params.sigG; ekf.sigB = params.sigB;
ekf.sigA = params.sigA; ekf.sigM = params.sigM;
end
function R = rotm(q)
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
     2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
     2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end