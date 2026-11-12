%% MPU-92/65 + Arduino Mega: raw vs complementary vs EKF (roll, pitch, yaw)
clear; close all; clc;

%% ---------------- settings ----------------
PORT         = 'COM7';      % your Arduino COM port
BOARD        = 'Mega2560';
RUN_TIME     = 60;          % seconds of live fusion
DO_MAG_CAL   = true;        % rotate the sensor in all directions at start
MAG_CAL_TIME = 30;          % seconds
TAU          = 0.5;         % complementary filter time constant (s)
N_INIT       = 50;          % samples averaged for the initial orientation

%% ---------------- connect + init ----------------
disp('Connecting to Arduino Mega...');
a = arduino(PORT, BOARD, 'Libraries', 'I2C');
s = mpu9250_init(a, struct('accelFS',4,'gyroFS',500));
if ~s.hasMag
    error('No magnetometer found: the EKF needs one to build its reference field.');
end

disp('Keep the sensor STILL for gyro calibration...'); pause(1);
s = mpu9250_calibrateGyro(s, 300);
if DO_MAG_CAL
    s = mpu9250_calibrateMag(s, MAG_CAL_TIME);
end

%% ---------------- initial orientation (flat and still) ----------------
fprintf('Place the sensor flat and STILL. Starting in 3 s...\n'); pause(3);
A0 = nan(N_INIT,3); M0 = nan(N_INIT,3); k0 = 0; tries = 0;
while k0 < N_INIT && tries < 20*N_INIT
    tries = tries + 1;
    d = getSample(s);
    if ~d.magValid, continue; end
    k0 = k0 + 1; A0(k0,:) = d.accel_g; M0(k0,:) = d.mag_uT;
end
if k0 < N_INIT/2
    error('Only %d valid mag samples during init. Check the magnetometer and calibration.', k0);
end
accel0 = mean(A0(1:k0,:),1);
mag0   = mean(M0(1:k0,:),1);
if std(vecnorm(A0(1:k0,:),2,2)) > 0.05
    warning('Accel varied during init (std %.3f g): was the sensor still?', std(vecnorm(A0(1:k0,:),2,2)));
end
fprintf('Init: |accel| = %.3f g, |mag| = %.1f uT\n', norm(accel0), norm(mag0));
if norm(mag0) < 20 || norm(mag0) > 80
    warning('|mag| outside 20-80 uT: yaw correction will be rejected. Redo mag calibration.');
end

lastMag = mag0;
[r0,p0,y0] = accelMagToEuler(accel0, mag0);
cf  = struct('roll',r0,'pitch',p0,'yaw',y0,'tau',TAU);
ekf = ekf_init(accel0, mag0);                   % tune: ekf_init(...,struct('sigM',0.3))

%% ---------------- live figure ----------------
figure('Name','Roll / Pitch / Yaw: Raw vs Complementary vs EKF','Position',[100 50 1000 800]);
names = {'Roll','Pitch','Yaw'}; L = cell(1,3);
for i = 1:3
    subplot(3,1,i); hold on; grid on; ylabel([names{i} ' (deg)']);
    L{i}.raw = animatedline('Color',[.6 .6 .6],'LineStyle',':','LineWidth',1,  'DisplayName','Raw (accel+mag)');
    L{i}.cf  = animatedline('Color','g','LineWidth',1.5,'DisplayName','Complementary');
    L{i}.ekf = animatedline('Color','b','LineWidth',1.5,'DisplayName','EKF');
    legend('Location','eastoutside');
end
xlabel('Time (s)');

%% ---------------- logging ----------------
nMax = RUN_TIME*400;
T = nan(nMax,1);
RPY_raw = nan(nMax,3); RPY_cf = nan(nMax,3); RPY_ekf = nan(nMax,3);
ACC = nan(nMax,3); GYR = nan(nMax,3); MAG = nan(nMax,3); TMP = nan(nMax,1);

disp('Running... move the sensor!');
n = 0; nMagNew = 0; t0 = tic; tPrev = 0;
while toc(t0) < RUN_TIME && n < nMax
    d = getSample(s);
    t = toc(t0);
    dt = min(max(t - tPrev, 1e-3), 0.2); tPrev = t;

    % mag: the EKF gets NaN when there is no NEW valid sample (it skips the update);
    % raw angles and the complementary filter use the last good value.
    magEKF = d.mag_uT;                          % NaN when stale/overflow
    if d.magValid
        lastMag = d.mag_uT; nMagNew = nMagNew + 1;
    else
        d.mag_uT = lastMag;
    end

    % (1) raw / direct angles
    [r,p,y] = accelMagToEuler(d.accel_g, d.mag_uT);
    rpyRaw = rad2deg([r p y]);
    % (2) complementary filter
    [cf, rpyCF]   = complementary_filter(cf, d.gyro_rads, d.accel_g, d.mag_uT, dt);
    % (3) EKF
    [ekf, rpyEKF] = ekf_step(ekf, d.gyro_rads, d.accel_g, magEKF, dt);

    n = n + 1;
    T(n) = t; RPY_raw(n,:) = rpyRaw; RPY_cf(n,:) = rpyCF; RPY_ekf(n,:) = rpyEKF;
    ACC(n,:) = d.accel_ms2; GYR(n,:) = d.gyro_dps; MAG(n,:) = d.mag_uT; TMP(n) = d.temp_C;

    for i = 1:3
        addpoints(L{i}.raw, t, rpyRaw(i));
        addpoints(L{i}.cf,  t, rpyCF(i));
        addpoints(L{i}.ekf, t, rpyEKF(i));
    end
    drawnow limitrate;
end
fprintf('Done: %d samples, average rate %.1f Hz, fresh mag in %.0f%% of samples\n', ...
        n, n/T(n), 100*nMagNew/n);

%% ---------------- trim + save ----------------
T = T(1:n); RPY_raw = RPY_raw(1:n,:); RPY_cf = RPY_cf(1:n,:); RPY_ekf = RPY_ekf(1:n,:);
ACC = ACC(1:n,:); GYR = GYR(1:n,:); MAG = MAG(1:n,:); TMP = TMP(1:n);
save('imu_log.mat','T','RPY_raw','RPY_cf','RPY_ekf','ACC','GYR','MAG','TMP');

%% ---------------- converted sensor data ----------------
figure('Name','Converted sensor data','Position',[150 80 1000 800]);
subplot(4,1,1); plot(T,ACC); grid on; ylabel('Accel (m/s^2)'); legend('x','y','z');
subplot(4,1,2); plot(T,GYR); grid on; ylabel('Gyro (deg/s)');  legend('x','y','z');
subplot(4,1,3); plot(T,MAG); grid on; ylabel('Mag (\muT)');    legend('x','y','z');
subplot(4,1,4); plot(T,TMP); grid on; ylabel('Temp (\circC)'); xlabel('Time (s)');

%% ---------------- numeric comparison ----------------
wrap180 = @(x) mod(x+180,360)-180;
dif = RPY_cf - RPY_ekf;  dif(:,3) = wrap180(dif(:,3));
fprintf('RMS difference  CF vs EKF  [roll pitch yaw] (deg): %.2f  %.2f  %.2f\n', sqrt(mean(dif.^2,'omitnan')));
dif = RPY_raw - RPY_ekf; dif(:,3) = wrap180(dif(:,3));
fprintf('RMS difference  Raw vs EKF [roll pitch yaw] (deg): %.2f  %.2f  %.2f\n', sqrt(mean(dif.^2,'omitnan')));

%% ---------------- local helper ----------------
function d = getSample(s)
    raw = mpu9250_readRaw(s);        % 1) raw counts
    d   = mpu9250_convert(raw, s);   % 2) physical units
end