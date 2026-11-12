%% IMU_VISUALIZER  Live 3D view: a board that moves like your real MPU-9250.
%  Left board  = quaternion EKF  (blue top face)
%  Right board = complementary filter (green top face)
%  Red end = FRONT (+x), blue top = UP (+z). Body axes: x red, y green, z blue.
%  Needs the 12 project files on the MATLAB path. Click "Stop" or close the figure to end.
clear; close all; clc;

%% ---------------- settings ----------------
PORT          = 'COM12';
BOARD         = 'Mega2560';
RUN_TIME      = 300;      % seconds (max)
DO_MAG_CAL    = true;     % rotate sensor in all directions at start
MAG_CAL_TIME  = 30;       % seconds
TAU           = 0.5;      % complementary filter time constant (s)
N_INIT        = 50;       % samples averaged for the initial orientation
RELATIVE_YAW  = true;     % true: boards start facing you (yaw shown relative to start)

%% ---------------- connect, calibrate, init (same steps as main_imu_fusion) ----------------
disp('Connecting to Arduino Mega...');
a = arduino(PORT, BOARD, 'Libraries', 'I2C');
s = mpu9250_init(a, struct('accelFS',4,'gyroFS',500));
if ~s.hasMag, error('No magnetometer found: the EKF needs one.'); end

disp('Keep the sensor STILL for gyro calibration...'); pause(1);
s = mpu9250_calibrateGyro(s, 300);
if DO_MAG_CAL, s = mpu9250_calibrateMag(s, MAG_CAL_TIME); end

fprintf('Place the sensor flat and STILL. Starting in 3 s...\n'); pause(3);
A0 = nan(N_INIT,3); M0 = nan(N_INIT,3); k0 = 0; tries = 0;
while k0 < N_INIT && tries < 20*N_INIT
    tries = tries + 1;
    d = getSample(s);
    if ~d.magValid, continue; end
    k0 = k0 + 1; A0(k0,:) = d.accel_g; M0(k0,:) = d.mag_uT;
end
if k0 < N_INIT/2, error('Only %d valid mag samples during init.', k0); end
accel0 = mean(A0(1:k0,:),1);  mag0 = mean(M0(1:k0,:),1);
fprintf('Init: |accel| = %.3f g, |mag| = %.1f uT\n', norm(accel0), norm(mag0));

lastMag = mag0;
[r0,p0,y0] = accelMagToEuler(accel0, mag0);
cf  = struct('roll',r0,'pitch',p0,'yaw',y0,'tau',TAU);
ekf = ekf_init(accel0, mag0);
if RELATIVE_YAW, yawRef = y0; else, yawRef = 0; end
Rz0 = [cos(-yawRef) -sin(-yawRef) 0; sin(-yawRef) cos(-yawRef) 0; 0 0 1];  % removes start heading

%% ---------------- figure: two 3D scenes ----------------
fig = figure('Name','Live IMU orientation','Position',[80 80 1100 560],'Color','w');
setappdata(fig,'stop',false);
uicontrol(fig,'Style','pushbutton','String','Stop','Position',[10 10 70 28], ...
          'Callback',@(~,~) setappdata(fig,'stop',true));

[axE, htE, txtE] = makeScene(subplot(1,2,1), 'EKF',                  [0.20 0.45 0.90]);
[axC, htC, txtC] = makeScene(subplot(1,2,2), 'Complementary filter', [0.15 0.70 0.30]);

%% ---------------- live loop ----------------
disp('Running... move the sensor! (Stop button or close the window to finish)');
n = 0; nMagNew = 0; t0 = tic; tPrev = 0;
while ishandle(fig) && ~getappdata(fig,'stop') && toc(t0) < RUN_TIME
    d  = getSample(s);
    t  = toc(t0);
    dt = min(max(t - tPrev, 1e-3), 0.2); tPrev = t;

    magEKF = d.mag_uT;                         % NaN when stale -> EKF skips mag
    if d.magValid, lastMag = d.mag_uT; nMagNew = nMagNew + 1; else, d.mag_uT = lastMag; end

    [cf,  rpyCF ] = complementary_filter(cf, d.gyro_rads, d.accel_g, d.mag_uT, dt);
    [ekf, rpyEKF] = ekf_step(ekf, d.gyro_rads, d.accel_g, magEKF, dt);
    n = n + 1;

    % rotation matrices (body -> world), same convention as the filters
    R_E = Rz0 * rotm(ekf.x(1:4));
    R_C = Rz0 * rotm(eul2quat_zyx(cf.roll, cf.pitch, cf.yaw));
    set(htE,'Matrix',[R_E zeros(3,1); 0 0 0 1]);
    set(htC,'Matrix',[R_C zeros(3,1); 0 0 0 1]);

    yE = wrapTo180(rpyEKF(3) - rad2deg(yawRef));
    yC = wrapTo180(rpyCF(3)  - rad2deg(yawRef));
    set(txtE,'String',sprintf('roll %6.1f   pitch %6.1f   yaw %6.1f  (deg)', rpyEKF(1), rpyEKF(2), yE));
    set(txtC,'String',sprintf('roll %6.1f   pitch %6.1f   yaw %6.1f  (deg)', rpyCF(1),  rpyCF(2),  yC));
    drawnow limitrate;
end
if n > 0
    fprintf('Done: %d frames, average rate %.1f Hz, fresh mag in %.0f%% of frames\n', ...
            n, n/toc(t0), 100*nMagNew/n);
end

%% ---------------- local functions ----------------
function d = getSample(s)
    d = mpu9250_convert(mpu9250_readRaw(s), s);
end

function R = rotm(q)               % body -> world, q = [w x y z]
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end

function a = wrapTo180(a)          % avoids needing the Mapping Toolbox
    a = mod(a + 180, 360) - 180;
end

function [ax, ht, txt] = makeScene(ax, name, topColor)
    axes(ax); hold(ax,'on'); %#ok<LAXES>
    axis(ax,'equal'); axis(ax,[-2 2 -2 2 -2 2]); grid(ax,'on'); box(ax,'on');
    view(ax,135,25);
    xlabel(ax,'X (start: forward)'); ylabel(ax,'Y'); zlabel(ax,'Z (up)');
    title(ax,name,'FontSize',13);
    % fixed world axes + floor reference
    plot3(ax,[-2 2],[0 0],[0 0],'-','Color',[.8 .8 .8]);
    plot3(ax,[0 0],[-2 2],[0 0],'-','Color',[.8 .8 .8]);
    plot3(ax,[0 0],[0 0],[-2 2],'-','Color',[.8 .8 .8]);
    patch(ax,'XData',[-2 2 2 -2],'YData',[-2 -2 2 2],'ZData',[-1.2 -1.2 -1.2 -1.2], ...
          'FaceColor',[.93 .93 .93],'EdgeColor',[.8 .8 .8],'FaceAlpha',0.6);
    % the board (hgtransform lets us rotate everything at once)
    ht = hgtransform('Parent',ax);
    V = [-1 -.5 -.08; 1 -.5 -.08; 1 .5 -.08; -1 .5 -.08; ...
         -1 -.5  .08; 1 -.5  .08; 1 .5  .08; -1 .5  .08];
    Fc = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    C  = [0.55 0.55 0.55;      % bottom
          topColor;            % top  (+z)
          0.75 0.75 0.75;      % side -y
          0.90 0.20 0.20;      % FRONT +x (red)
          0.75 0.75 0.75;      % side +y
          0.60 0.60 0.60];     % back -x
    patch('Parent',ht,'Vertices',V,'Faces',Fc,'FaceVertexCData',C, ...
          'FaceColor','flat','EdgeColor','k');
    % body axes drawn on the board
    plot3(ht,[0 1.6],[0 0],[0 0],'r-','LineWidth',2.5);
    plot3(ht,[0 0],[0 1.2],[0 0],'g-','LineWidth',2.5);
    plot3(ht,[0 0],[0 0],[0 1.2],'b-','LineWidth',2.5);
    camlight(ax,'headlight'); lighting(ax,'gouraud');
    txt = text(ax,0,0,-1.9,'','HorizontalAlignment','center','FontName','Consolas','FontSize',10);
    set(txt,'Units','data');
end