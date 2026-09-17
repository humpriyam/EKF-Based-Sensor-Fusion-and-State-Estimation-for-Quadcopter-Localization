clear all; close all; clc;

disp('Connecting to Arduino Mega... Please wait.');
a=arduino('COM7','Mega2560','Libraries','I2C');
disp('Arduino Connected Successfully!');

disp('Initializing MPU6050 Sensor...');
imu=mpu6050(a);
disp('MPU6050 Initialized!');

LOOP_RATE=10; 
compFilter=complementaryFilter('HasMagnetometer',false,'SampleRate',LOOP_RATE);
ekfFilter=imufilter('SampleRate',LOOP_RATE); 

disp('Setting up graphs...');
fig=figure('Name','MPU6050 Angle Comparison (Raw vs Filters)','NumberTitle','off','Position',[100,50,800,800]);

subplot(3,1,1); title('Roll (X-Axis Rotation)'); hold on; grid on; ylabel('Degrees');
lineRollRaw=animatedline('Color','r','LineStyle',':','DisplayName','Raw');
lineRollComp=animatedline('Color','g','LineWidth',1.5,'DisplayName','Complementary');
lineRollEKF=animatedline('Color','b','LineWidth',1.5,'DisplayName','EKF');
legend('Location','eastoutside');

subplot(3,1,2); title('Pitch (Y-Axis Rotation)'); hold on; grid on; ylabel('Degrees');
linePitchRaw=animatedline('Color','r','LineStyle',':','DisplayName','Raw');
linePitchComp=animatedline('Color','g','LineWidth',1.5,'DisplayName','Complementary');
linePitchEKF=animatedline('Color','b','LineWidth',1.5,'DisplayName','EKF');
legend('Location','eastoutside');

subplot(3,1,3); title('Yaw (Z-Axis Rotation)'); hold on; grid on; ylabel('Degrees'); xlabel('Time (seconds)');
lineYawRaw=animatedline('Color','r','LineStyle',':','DisplayName','Raw');
lineYawComp=animatedline('Color','g','LineWidth',1.5,'DisplayName','Complementary');
lineYawEKF=animatedline('Color','b','LineWidth',1.5,'DisplayName','EKF');
legend('Location','eastoutside');

disp('Starting 60-second data collection... Move your sensor!');

rawYaw=0; 
lastTime=0;
r=rateControl(LOOP_RATE); 

tic; 
while toc<60
    t=toc;
    dt=t-lastTime;
    lastTime=t;

    sensorData=read(imu);
    accel=sensorData.Acceleration(end,:);
    gyro=sensorData.AngularVelocity(end,:);

    rawRoll=atan2d(accel(2),accel(3));
    rawPitch=atan2d(-accel(1),sqrt(accel(2)^2+accel(3)^2));
    rawYaw=rawYaw+(rad2deg(gyro(3))*dt);

    qComp=compFilter(accel,gyro);
    qEKF=ekfFilter(accel,gyro);

    eComp=eulerd(qComp,'ZYX','frame'); 
    eEKF=eulerd(qEKF,'ZYX','frame');

    addpoints(lineRollRaw,t,rawRoll);
    addpoints(lineRollComp,t,eComp(3));
    addpoints(lineRollEKF,t,eEKF(3));

    addpoints(linePitchRaw,t,rawPitch);
    addpoints(linePitchComp,t,eComp(2));
    addpoints(linePitchEKF,t,eEKF(2));

    addpoints(lineYawRaw,t,rawYaw);
    addpoints(lineYawComp,t,eComp(1));
    addpoints(lineYawEKF,t,eEKF(1));

    drawnow limitrate;
    waitfor(r); 
end
disp('Data collection complete.');