function raw = mpu9250_readRaw(s)
%MPU9250_READRAW  Burst-read raw 16-bit counts from the sensor (no scaling).
%   raw.accel [1x3] counts, raw.gyro [1x3] counts, raw.temp counts,
%   raw.mag [1x3] counts (AK8963 axes), raw.magNew, raw.magOverflow

% accel(6) + temp(2) + gyro(6) = 14 bytes starting at ACCEL_XOUT_H (0x3B), big-endian
write(s.imu, uint8(0x3B));
d = read(s.imu, 14);
v = double(swapbytes(typecast(uint8(d(:)'),'int16')));
raw.accel = v(1:3);
raw.temp  = v(4);
raw.gyro  = v(5:7);

raw.mag = [NaN NaN NaN]; raw.magNew = false; raw.magOverflow = false;
if s.hasMag
    % ST1, HXL..HZH (little-endian), ST2 (reading ST2 releases the data latch)
    write(s.mag, uint8(0x02));
    m = read(s.mag, 8);
    raw.magNew = bitand(m(1),1) > 0;                 % DRDY
    if raw.magNew
        raw.mag = double(typecast(uint8(m(2:7)),'int16'));
        raw.magOverflow = bitand(m(8),8) > 0;        % HOFL
    end
end
end
