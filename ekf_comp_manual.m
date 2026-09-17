clear all; close all; clc;

disp('Connecting to Arduino Mega... Please wait.');
a=arduino('COM7','Mega2560','Libraries','I2C');
disp('Arduino Connected Successfully!');

disp('Initializing MPU6050 Sensor...');
imu=mpu6050(a);
disp('MPU6050 Initialized!');

LOOP_RATE=10; 
compFilter_builtIn=complementaryFilter('HasMagnetometer',false,'SampleRate',LOOP_RATE);
ekfFilter_builtIn=imufilter('SampleRate',LOOP_RATE); 

tau=0.5; 
customCompRoll=0;
customCompPitch=0;

x_roll=[0;0]; 
x_pitch=[0;0];

P_roll=[1 0;0 1]; 
P_pitch=[1 0;0 1];

Q_angle=0.001; 
Q_bias=0.003;
Q=[Q_angle 0;0 Q_bias]; 

R_measure=0.03; 
H=[1 0]; 

disp('Setting up graphs...');
fig=figure('Name','MPU6050 Angle Comparison (Custom vs Built-in)','NumberTitle','off','Position',[100,50,1000,800]);

subplot(2,1,1); title('Roll (X-Axis Rotation)'); hold on; grid on; ylabel('Degrees');
lineRollRaw=animatedline('Color','k','LineStyle',':','DisplayName','Raw (Accel)');
lineRollCompBuiltIn=animatedline('Color','b','LineStyle',':','DisplayName','Comp (Built-in)');
lineRollEKFBuiltIn=animatedline('Color','r','LineStyle',':','DisplayName','EKF (Built-in)');
lineRollCompCustom=animatedline('Color','b','LineWidth',1,'DisplayName','Comp (Custom)');
lineRollKFCustom=animatedline('Color','r','LineWidth',1,'DisplayName','KF (Custom)');
legend('Location','eastoutside');

subplot(2,1,2); title('Pitch (Y-Axis Rotation)'); hold on; grid on; ylabel('Degrees'); xlabel('Time (seconds)');
linePitchRaw=animatedline('Color','k','LineStyle',':','DisplayName','Raw (Accel)');
linePitchCompBuiltIn=animatedline('Color','b','LineStyle',':','DisplayName','Comp (Built-in)');
linePitchEKFBuiltIn=animatedline('Color','r','LineStyle',':','DisplayName','EKF (Built-in)');
linePitchCompCustom=animatedline('Color','b','LineWidth',1,'DisplayName','Comp (Custom)');
linePitchKFCustom=animatedline('Color','r','LineWidth',1,'DisplayName','KF (Custom)');
legend('Location','eastoutside');

disp('Starting 60-second data collection...');
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

    gyroRateRoll=rad2deg(gyro(1)); 
    gyroRatePitch=rad2deg(gyro(2));

    qComp=compFilter_builtIn(accel,gyro);
    qEKF=ekfFilter_builtIn(accel,gyro);
    eComp=eulerd(qComp,'ZYX','frame'); 
    eEKF=eulerd(qEKF,'ZYX','frame');

    alpha=tau/(tau+dt);
    customCompRoll=alpha*(customCompRoll+gyroRateRoll*dt)+(1-alpha)*rawRoll;
    customCompPitch=alpha*(customCompPitch+gyroRatePitch*dt)+(1-alpha)*rawPitch;

    F=[1 -dt;0 1];
    B=[dt;0];
    
    x_roll=F*x_roll+B*gyroRateRoll;
    P_roll=F*P_roll*F'+Q;
    S_roll=H*P_roll*H'+R_measure;
    K_roll=P_roll*H'/S_roll; 
    y_roll=rawRoll-(H*x_roll);
    x_roll=x_roll+K_roll*y_roll;
    P_roll=(eye(2)-K_roll*H)*P_roll;

    x_pitch=F*x_pitch+B*gyroRatePitch;
    P_pitch=F*P_pitch*F'+Q;
    S_pitch=H*P_pitch*H'+R_measure;
    K_pitch=P_pitch*H'/S_pitch;
    y_pitch=rawPitch-(H*x_pitch);
    x_pitch=x_pitch+K_pitch*y_pitch;
    P_pitch=(eye(2)-K_pitch*H)*P_pitch;

    addpoints(lineRollRaw,t,rawRoll);
    addpoints(lineRollCompBuiltIn,t,eComp(3));
    addpoints(lineRollEKFBuiltIn,t,eEKF(3));
    addpoints(lineRollCompCustom,t,customCompRoll);
    addpoints(lineRollKFCustom,t,x_roll(1));

    addpoints(linePitchRaw,t,rawPitch);
    addpoints(linePitchCompBuiltIn,t,eComp(2));
    addpoints(linePitchEKFBuiltIn,t,eEKF(2));
    addpoints(linePitchCompCustom,t,customCompPitch);
    addpoints(linePitchKFCustom,t,x_pitch(1));

    drawnow limitrate;
    waitfor(r); 
end
disp('Data collection complete.');