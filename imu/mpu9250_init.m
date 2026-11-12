function s = mpu9250_init(a, opts)
%MPU9250_INIT  Configure MPU-9250/9255/9265 and its AK8963 magnetometer over I2C.
%   s = mpu9250_init(a)            a = arduino object (Libraries,'I2C')
%   s = mpu9250_init(a, opts)      opts fields (all optional):
%       accelFS (2|4|8|16 g) default 4
%       gyroFS  (250|500|1000|2000 dps) default 500
%       dlpfCfg (0..6) gyro DLPF, default 3 (41 Hz)
%       sampleDiv  rate = 1kHz/(1+div), default 4 (200 Hz internal)
%   Requires R2019b+ (hex literals). Older MATLAB: replace 0x68 -> 104 etc.

if nargin < 2, opts = struct(); end
if ~isfield(opts,'accelFS'),   opts.accelFS   = 4;   end
if ~isfield(opts,'gyroFS'),    opts.gyroFS    = 500; end
if ~isfield(opts,'dlpfCfg'),   opts.dlpfCfg   = 3;   end
if ~isfield(opts,'sampleDiv'), opts.sampleDiv = 4;   end

wr = @(dev,reg,val) write(dev, uint8([reg val]));   % write one register

%% 1) find the MPU on 0x68 or 0x69
imu = [];
for addr = [104 105]
    try
        tmp = device(a,'I2CAddress',addr);
        write(tmp, uint8(0x75));            % WHO_AM_I
        who = read(tmp,1);
        imu = tmp; s.addr = addr; s.whoami = who;
        break
    catch
    end
end
if isempty(imu)
    error('MPU not found on 0x68/0x69. Check SDA(20)/SCL(21)/VCC/GND wiring.');
end
fprintf('MPU found at 0x%02X, WHO_AM_I = 0x%02X ', s.addr, who);
if ismember(who,[0x71 0x73]), fprintf('(MPU-9250/9255/9265, OK)\n');
elseif who == 0x70,           fprintf('(MPU-6500: no magnetometer!)\n');
else,                         fprintf('(unknown clone, continuing)\n'); end

%% 2) reset and configure
wr(imu,0x6B,0x80); pause(0.1);              % device reset
wr(imu,0x6B,0x01); pause(0.05);             % wake, clock = PLL (best available)
wr(imu,0x19,opts.sampleDiv);                % SMPLRT_DIV
wr(imu,0x1A,opts.dlpfCfg);                  % CONFIG: gyro DLPF

gIdx = find([250 500 1000 2000]==opts.gyroFS)-1;
aIdx = find([2 4 8 16]==opts.accelFS)-1;
if isempty(gIdx) || isempty(aIdx), error('Invalid full-scale option.'); end
wr(imu,0x1B,bitshift(uint8(gIdx),3));       % GYRO_CONFIG
wr(imu,0x1C,bitshift(uint8(aIdx),3));       % ACCEL_CONFIG
wr(imu,0x1D,0x03);                          % ACCEL_CONFIG2: accel DLPF ~44 Hz

gyroLSBs = [131 65.5 32.8 16.4];
accLSBs  = [16384 8192 4096 2048];
s.imu      = imu;
s.gyroLSB  = gyroLSBs(gIdx+1);              % LSB per (deg/s)
s.accelLSB = accLSBs(aIdx+1);               % LSB per g
s.gyroBias = [0 0 0];                       % deg/s, set by calibrateGyro
s.magLSB   = 0.15;                          % uT/LSB (AK8963, 16-bit mode)
s.magASA   = [1 1 1];
s.magOffset= [0 0 0];                       % hard-iron (uT)
s.magScale = [1 1 1];                       % soft-iron scale
s.hasMag   = false;

%% 3) magnetometer (AK8963 @0x0C) via I2C bypass
wr(imu,0x6A,0x00);                          % I2C master off
wr(imu,0x37,0x02);                          % INT_PIN_CFG: BYPASS_EN -> host sees AK8963
pause(0.01);
try
    mag = device(a,'I2CAddress',0x0C);
    write(mag,uint8(0x00)); wia = read(mag,1);   % WIA must be 0x48
    if wia ~= 0x48, error('bad WIA'); end
    wr(mag,0x0B,0x01); pause(0.01);         % soft reset
    wr(mag,0x0A,0x00); pause(0.01);         % power down
    wr(mag,0x0A,0x0F); pause(0.01);         % fuse ROM access
    write(mag,uint8(0x10)); asa = double(read(mag,3));
    s.magASA = (asa-128)/256 + 1;           % factory sensitivity adjustment
    wr(mag,0x0A,0x00); pause(0.01);
    wr(mag,0x0A,0x16); pause(0.01);         % 16-bit, continuous mode 2 (100 Hz)
    s.mag = mag; s.hasMag = true;
    fprintf('AK8963 magnetometer OK (ASA = %s)\n', mat2str(asa(:)'));
catch
    warning('Magnetometer not found. Yaw will be gyro-only (drifts).');
end
end
