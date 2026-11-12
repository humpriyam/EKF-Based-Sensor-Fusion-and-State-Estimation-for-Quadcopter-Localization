function s = mpu9250_calibrateGyro(s, N)
if nargin < 2, N = 300; end
s.gyroBias = [0 0 0];
fprintf('Keep sensor still...\n');
for k = 1:10, mpu9250_readRaw(s); end          % discard startup samples
acc = zeros(N,3);
for k = 1:N
    d = mpu9250_convert(mpu9250_readRaw(s), s);
    acc(k,:) = d.gyro_dps;
end
sd = std(acc,0,1);
if any(sd > 0.5), warning('Sensor moved during calibration (std=%s). Redo.', mat2str(sd,2)); end
s.gyroBias = mean(acc,1);
if any(abs(s.gyroBias) > 5), warning('Bias outside datasheet ±5 dps.'); end
fprintf('Gyro bias (dps): [%.3f %.3f %.3f]\n', s.gyroBias);
end