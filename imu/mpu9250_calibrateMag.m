function s = mpu9250_calibrateMag(s, seconds)
if nargin < 2, seconds = 30; end
s.magOffset = [0 0 0]; s.magScale = [1 1 1];
pts = nan(seconds*120,3); n = 0;
fprintf('Mag calibration: rotate the sensor in all directions for %d s...\n', seconds);
t0 = tic;
while toc(t0) < seconds && n < size(pts,1)
    d = mpu9250_convert(mpu9250_readRaw(s), s);
    if d.magValid, n = n+1; pts(n,:) = d.mag_uT; end
end
pts = pts(1:n,:);
if n < 100, warning('Too few mag samples (%d). Calibration skipped.', n); return; end
lim = prctile(pts,[2 98]);
mn = lim(1,:); mx = lim(2,:);
rad = (mx-mn)/2;
if any(rad < 5)
    warning('Poor coverage (radius = %s uT). Calibration skipped.', mat2str(rad,2));
    return;
end
s.magOffset = (mx+mn)/2;
s.magScale  = mean(rad)./rad;
fprintf('Mag offset (uT): [%.1f %.1f %.1f]  scale: [%.2f %.2f %.2f]\n', s.magOffset, s.magScale);
end