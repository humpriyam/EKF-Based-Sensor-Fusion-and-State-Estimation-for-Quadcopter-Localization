clear;
clc;close all;

rho=1.225;%airDensity
B=input('Enter the Number of blades (default B=2): ');
if isempty(B), B=2; end
omega_rpm=input('Rotational speed in RPM (default 5000): ');
if isempty(omega_rpm), omega_rpm=5000; end
c=input('ChordLength (default c=0.02m): ');
if isempty(c), c=0.02; end
omega=omega_rpm*(2*pi/60); 
R_min=0.0254;%hubRadius
R_max=0.254;%tipRadius
theta_root_deg=30;%initially 30 degrees then 5 degrees at the tip
theta_tip_deg=5;
m_slope=(theta_tip_deg-theta_root_deg)/(R_max-R_min);
b_intercept=theta_root_deg-m_slope*R_min;

CL=@(theta_val) 2*pi*theta_val;
CD=@(theta_val) 0.01+5*theta_val.^2;
theta=@(r) deg2rad(m_slope*r+b_intercept);%Theta changes linearly from 30 degrees to 5 degrees

dTdr=@(r) B.*0.5.*rho.*(omega.*r).^2.*c.*sqrt(1+tan(theta(r)).^2).*(CL(theta(r))-CD(theta(r)).*tan(theta(r)));
dtaudr=@(r) B.*0.5.*rho.*(omega.*r).^2.*c.*sqrt(1+tan(theta(r)).^2).*(CL(theta(r)).*tan(theta(r))+CD(theta(r))).*r;

num_points=100;
r_vec=linspace(R_min, R_max, num_points);
T_vec=zeros(1,num_points);
tau_vec=zeros(1,num_points);

for i=1:num_points
    T_vec(i)=integral(dTdr, R_min, r_vec(i));%integrating using Matlab inbuild function "integral"
    tau_vec(i)=integral(dtaudr, R_min, r_vec(i));%integral(fun, lower_limit, upper_limit)
end

r3=r_vec(:);
th3=T_vec(:);
to3=tau_vec(:);

sim_th3=[r3,th3];
sim_to3=[r3,to3];
% Plot results
figure('Name', 'Blade Element Momentum Theory Results');

% Thrust Plot
subplot(2,1,1);
plot(r3, th3, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Radius (m)');
ylabel('Cumulative Thrust (N)');
title('Thrust Distribution along the Blade');

% Torque Plot
subplot(2,1,2);
plot(r3, to3, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Radius (m)');
ylabel('Cumulative Torque (N-m)');
title('Torque Distribution along the Blade');