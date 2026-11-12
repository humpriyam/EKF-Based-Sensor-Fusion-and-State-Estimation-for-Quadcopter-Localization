%% IMU_BOARD_VISUALIZER  Live 3-D model of the MPU-92/65 board that moves like the real sensor.
%  Body axes follow the silkscreen on the board:
%     +X = toward the right edge (away from the pin header)
%     +Y = along the board toward the VCC end of the pin header
%     +Z = out of the component face (right-handed)
%  Solid blue board = EKF.  Transparent green ghost = complementary filter.
%  Needs the 12 project files on the MATLAB path. Close the figure to stop.
clear; close all; clc;

%% ---------------- settings ----------------
PORT         = 'COM12 ';
BOARD        = 'Mega2560';
RUN_TIME     = 300;      % seconds (or close the window)
TAU          = 0.5;      % complementary filter time constant (s)
N_INIT       = 50;       % samples averaged for the starting orientation
DO_MAG_CAL   = true;     % rotate the sensor in all directions at start
MAG_CAL_TIME = 30;       % seconds
ZERO_YAW     = true;     % true: model starts facing the same way you start
SHOW_CF      = true;     % draw the complementary-filter ghost board

%% ---------------- connect, calibrate, init ----------------
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

Rz0 = eye(3);                                   % optional heading zeroing
if ZERO_YAW
    c = cos(-y0); sn = sin(-y0);
    Rz0 = [c -sn 0; sn c 0; 0 0 1];
end

%% ---------------- 3-D scene ----------------
fig = figure('Name','MPU-92/65 live orientation','Color','w','Position',[100 60 900 750]);
ax  = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
lim = 35; axis(ax,[-lim lim -lim lim -lim lim]); view(ax,135,25);
xlabel(ax,'World X'); ylabel(ax,'World Y'); zlabel(ax,'World Z (up)');
if ZERO_YAW, xName = 'X (start heading)'; else, xName = 'X (mag. north)'; end
plot3(ax,[0 30],[0 0],[0 0],'-','Color',[.7 .7 .7],'LineWidth',1);
plot3(ax,[0 0],[0 30],[0 0],'-','Color',[.7 .7 .7],'LineWidth',1);
plot3(ax,[0 0],[0 0],[0 30],'-','Color',[.7 .7 .7],'LineWidth',1);
text(ax,32,0,0,xName,'Color',[.5 .5 .5]); text(ax,0,32,0,'Y','Color',[.5 .5 .5]);
text(ax,0,0,32,'Up','Color',[.5 .5 .5]);
patch(ax,[-lim lim lim -lim],[-lim -lim lim lim],[-12 -12 -12 -12],[.93 .93 .93], ...
      'FaceAlpha',0.5,'EdgeColor','none');      % floor, makes tilt easy to see

BE = makeBoard(ax, [0.10 0.25 0.75], 1.0, true);
if SHOW_CF, BC = makeBoard(ax, [0.2 0.8 0.3], 0.25, false); end
ttl = title(ax,'');
camlight(ax,'headlight'); lighting(ax,'gouraud');

%% ---------------- live loop ----------------
disp('Running... move the sensor! Close the figure to stop.');
t0 = tic; tPrev = 0; k = 0;
while ishandle(fig) && toc(t0) < RUN_TIME
    d = getSample(s);
    t = toc(t0);
    dt = min(max(t - tPrev, 1e-3), 0.2); tPrev = t;

    magEKF = d.mag_uT;                          % NaN when stale (EKF skips it)
    if d.magValid, lastMag = d.mag_uT; else, d.mag_uT = lastMag; end

    [cf,  rpyCF]  = complementary_filter(cf, d.gyro_rads, d.accel_g, d.mag_uT, dt); %#ok<ASGLU>
    [ekf, rpyEKF] = ekf_step(ekf, d.gyro_rads, d.accel_g, magEKF, dt);

    if ~ishandle(fig), break; end
    poseBoard(BE, Rz0*rotm(ekf.x(1:4)));
    if SHOW_CF
        poseBoard(BC, Rz0*rotm(eul2quat_zyx(cf.roll,cf.pitch,cf.yaw)));
    end
    k = k + 1;
    if mod(k,5) == 0
        set(ttl,'String',sprintf('EKF  roll %6.1f   pitch %6.1f   yaw %7.1f deg   (%.0f Hz)', ...
            rpyEKF(1), rpyEKF(2), rpyEKF(3), k/t));
    end
    drawnow limitrate;
end
disp('Stopped.');

%% ---------------- helpers ----------------
function d = getSample(s)
    d = mpu9250_convert(mpu9250_readRaw(s), s);
end

function R = rotm(q)                            % body -> world, q = [w x y z]
    q = q(:)/norm(q);
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end

function B = makeBoard(ax, col, alpha, detailed)
    % Board 15.5 x 25.5 x 1.6 mm. Pin header on the -X edge, chip in the middle.
    parts = { [-7.75 7.75 -12.75 12.75 -0.8 0.8], col };
    if detailed
        parts(end+1,:) = {[-3 3 -3 3 0.8 2.6],        [0.12 0.12 0.12]};  % MPU chip
        parts(end+1,:) = {[-7.4 -5.2 -12 12 0.8 2.0], [0.85 0.72 0.25]};  % pin header
        parts(end+1,:) = {[ 3.5 6 -2 3 0.8 2.2],      [0.95 0.65 0.1]};   % tantalum cap
    end
    if alpha > 0.5, ec = [.15 .15 .15]; else, ec = [.1 .5 .1]; end
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    n = size(parts,1);
    B.V = cell(1,n); B.h = gobjects(1,n);
    for i = 1:n
        p = parts{i,1};
        V = [p(1) p(3) p(5); p(2) p(3) p(5); p(2) p(4) p(5); p(1) p(4) p(5);
             p(1) p(3) p(6); p(2) p(3) p(6); p(2) p(4) p(6); p(1) p(4) p(6)];
        B.V{i} = V;
        B.h(i) = patch(ax,'Faces',F,'Vertices',V,'FaceColor',parts{i,2}, ...
                       'FaceAlpha',alpha,'EdgeColor',ec);
    end
    B.detailed = detailed;
    if detailed
        L = 24;
        B.axP = {[0 0 2.6; L 0 2.6], [0 0 2.6; 0 L 2.6], [0 0 2.6; 0 0 2.6+L]};
        cols  = {[.85 0 0], [0 .6 0], [0 0 .9]};  names = {'X','Y','Z'};
        B.axH = gobjects(1,3); B.axT = gobjects(1,3);
        for i = 1:3
            B.axH(i) = plot3(ax,B.axP{i}(:,1),B.axP{i}(:,2),B.axP{i}(:,3),'-', ...
                             'Color',cols{i},'LineWidth',3);
            tip = B.axP{i}(2,:)*1.12;
            B.axT(i) = text(ax,tip(1),tip(2),tip(3),names{i},'Color',cols{i}, ...
                            'FontWeight','bold','FontSize',13);
        end
        B.pinP = [-12 0 1];                      % label on the pin-header side
        B.pinT = text(ax,B.pinP(1),B.pinP(2),B.pinP(3),'pins','Color',[.5 .4 .0],'FontSize',9);
    end
end

function poseBoard(B, R)
    for i = 1:numel(B.h)
        set(B.h(i),'Vertices',(R*B.V{i}')');
    end
    if B.detailed
        for i = 1:3
            P = (R*B.axP{i}')';
            set(B.axH(i),'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));
            set(B.axT(i),'Position',P(2,:)*1.12);
        end
        set(B.pinT,'Position',(R*B.pinP')');
    end
end
