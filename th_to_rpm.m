clear;clc;
close all;

rho=1.225;%airDensity
B=input('Enter the Number of blades (B=2): ');
if isempty(B), B=2; end
R_max=(5/2)*0.0254;%tipRadius
R_min=0.005;%hubRadius
c=input('ChordLength (c=0.015m): ');
if isempty(c), c=0.015; end
rpm_min=0;
rpm_max=10000;
num_points=200;
rpm=linspace(rpm_min, rpm_max, num_points)';
r1=rpm;%speedRef1
r2=rpm;%speedRef2
theta_root_deg=30;%initially 30 degrees then 5 degrees at the tip
theta_tip_deg=5;
m_slope=(theta_tip_deg-theta_root_deg)/(R_max-R_min);
b_intercept=theta_root_deg-m_slope*R_min;

%using BET,Lift (C_L) and Drag (C_D) coefficients
CL=@(theta_val) 2*pi*theta_val; 
CD=@(theta_val) 0.01+5*theta_val.^2; 
theta_r=@(r) deg2rad(m_slope*r+b_intercept);%Theta changes linearly from 30 degrees to 5 degrees

th1=zeros(num_points,1);%thrustMethod1
to1=zeros(num_points,1);%torqueMethod1

for i=1:num_points
    omega=r1(i)*(2*pi/60);
    dTdr=@(r) B.*0.5.*rho.*(omega.*r).^2.*c.*sqrt(1+tan(theta_r(r)).^2).*(CL(theta_r(r))-CD(theta_r(r)).*tan(theta_r(r)));
    dtaudr=@(r) B.*0.5.*rho.*(omega.*r).^2.*c.*sqrt(1+tan(theta_r(r)).^2).*(CL(theta_r(r)).*tan(theta_r(r))+CD(theta_r(r))).*r;

    th1(i)=integral(dTdr, R_min, R_max);%integrating using Matlab inbuild function "integral" Gauss-Kronrod Method
    to1(i)=integral(dtaudr, R_min, R_max); 
end

%using Lumped Parameter Model,Thrust (C_T) and Torque (C_Q) coefficients
theta_x=@(x) deg2rad(m_slope*(x*R_max)+b_intercept);%using x=r/R; when r=0 x=0; when r=R x=1
x_min=R_min/R_max;%so instead of integrating from 0 to R, we can ingrate from 0 to 1
x_max=1;


integrand_CT=@(x) x.^2.*c.*sqrt(1+tan(theta_x(x)).^2).*(CL(theta_x(x))-CD(theta_x(x)).*tan(theta_x(x)));
integrand_CQ=@(x) x.^3.*c.*sqrt(1+tan(theta_x(x)).^2).*(CL(theta_x(x)).*tan(theta_x(x))+CD(theta_x(x)));
%integrating using Matlab inbuild function "integral" Gauss-Kronrod Method adn we will have to integrate only once
C_T=(B/(2*pi*R_max))*integral(integrand_CT, x_min, x_max);%0.0315
C_Q=(B/(2*pi*R_max))*integral(integrand_CQ, x_min, x_max);%0.0089

omega_vec=r2.*(2*pi/60);
v_tip=omega_vec.*R_max;
A=pi*R_max^2;

th2=C_T.*rho.*A.*v_tip.^2; %T = C_T * rho * A * (V_tip)^2
to2=C_Q.*rho.*A.*v_tip.^2.*R_max; %tau = C_Q * rho * A * (V_tip)^2 * R_max

sim_th1=[r1,th1];
sim_to1=[r1,to1];
sim_th2=[r2,th2];
sim_to2=[r2,to2];