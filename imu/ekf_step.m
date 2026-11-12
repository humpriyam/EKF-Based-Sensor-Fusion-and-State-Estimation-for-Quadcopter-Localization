function [ekf, rpy_deg] = ekf_step(ekf, gyro_rads, accel_g, mag_uT, dt)
%EKF_STEP  One predict+update cycle of the quaternion EKF.
%   Predict : q_dot = 0.5 * q (x) (gyro - bias)      (nonlinear -> Jacobian F)
%   Update  : z = [unit accel; unit mag]  vs  h(x) = R(q)' * [gravity; mag reference]
%   Pass mag_uT = [NaN NaN NaN] to skip the magnetometer update.
%   Accel is skipped when |a| is far from 1 g; mag is skipped when |mag| is
%   outside 20-80 uT, or whenever the accel is skipped (so a bad mag can't
%   re-tilt the estimate on its own).
x = ekf.x; P = ekf.P;
if ~isscalar(dt) || ~isfinite(dt) || dt <= 0 || any(~isfinite(gyro_rads))
    [r,p,y] = quat2eul_zyx(x(1:4));          % bad inputs: keep state, no step
    rpy_deg = rad2deg([r p y]);
    return;
end
dt = min(dt, 0.2);
q = x(1:4); b = x(5:7);
w = gyro_rads(:) - b;
%% ---- predict ----
Xi = xiMat(q);
F = eye(7);
F(1:4,1:4) = eye(4) + 0.5*dt*omegaMat(w);
F(1:4,5:7) = -0.5*dt*Xi;
n = norm(w);                                 % exact rotation step q = q (x) dq
if n*dt > 1e-9
    a  = 0.5*n*dt;
    dq = [cos(a); sin(a)*w/n];
else
    dq = [1; 0.5*dt*w];
end
q = qmul(q, dq);
q = q/norm(q);
Qq = (0.5*dt)^2 * ekf.sigG^2 * (Xi*Xi');
Q  = blkdiag(Qq, ekf.sigB^2*dt*eye(3));
x(1:4) = q;
P = F*P*F' + Q;
%% ---- update ----
na    = norm(accel_g);
useA  = all(isfinite(accel_g)) && abs(na - 1) < 0.5;
mn    = norm(mag_uT);
useMag = useA && all(isfinite(mag_uT)) && mn > 20 && mn < 80;
if useA
    z = accel_g(:)/na;
    sigA = ekf.sigA*(1 + 20*abs(na-1));      % distrust accel when |a| != 1 g
    R = sigA^2*eye(3);
    if useMag
        z = [z; mag_uT(:)/mn];
        R = blkdiag(R, ekf.sigM^2*eye(3));
    end
    h0 = hMeas(x(1:4), ekf.ref, useMag);
    H  = zeros(numel(z),7);
    e  = 1e-6;
    for i = 1:4                              % numerical Jacobian wrt quaternion
        dx = zeros(4,1); dx(i) = e;
        H(:,i) = (hMeas(x(1:4)+dx, ekf.ref, useMag) - h0)/e;
    end
    S = H*P*H' + R;
    K = P*H'/S;
    x = x + K*(z - h0);
    x(1:4) = x(1:4)/norm(x(1:4));
    P = (eye(7) - K*H)*P;
    P = (P + P')/2;
end
ekf.x = x; ekf.P = P;
[r,p,y] = quat2eul_zyx(x(1:4));
rpy_deg = rad2deg([r p y]);
end
function Xi = xiMat(q)       % q (x) [0 v] = Xi(q)*v
w=q(1); x=q(2); y=q(3); z=q(4);
Xi = [-x -y -z;  w -z  y;  z  w -x; -y  x  w];
end
function Om = omegaMat(v)    % q (x) [0 v] = Om(v)*q
vx=v(1); vy=v(2); vz=v(3);
Om = [0 -vx -vy -vz; vx 0 vz -vy; vy -vz 0 vx; vz vy -vx 0];
end
function r = qmul(a, b)      % Hamilton product, [w x y z] column vectors
av = a(2:4); bv = b(2:4);
r = [a(1)*b(1) - dot(av,bv);
     a(1)*bv + b(1)*av + cross(av,bv)];
end
function h = hMeas(q, ref, useMag)
w=q(1); x=q(2); y=q(3); z=q(4);
ha = [2*(x*z - w*y); 2*(y*z + w*x); 1 - 2*(x^2 + y^2)];   % R' * [0;0;1]
if useMag
    R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
    h = [ha; R'*ref];
else
    h = ha;
end
end