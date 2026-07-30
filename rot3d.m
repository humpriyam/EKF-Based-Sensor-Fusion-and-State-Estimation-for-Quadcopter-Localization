clc;clear; close all;

Rx= @(phi) [1,0, 0; 
0, cosd(phi),-sind(phi); 
  0, sind(phi), cosd(phi)];
               
 Ry = @(theta) [cosd(theta), 0, sind(theta); 
   0, 1, 0; 
  -sind(theta), 0,cosd(theta)];
              
    Rz=@(psi) [cosd(psi),-sind(psi), 0; 
sind(psi), cosd(psi), 0; 
0, 0, 1];

v_input=input('Enter 3D vector [x, y, z]: ');
  v= v_input(:);
    
loop_flag =1;

while loop_flag== 1
disp(' ');
   disp('--- 3D Rotation Menu ---')
disp('1: Yaw Rotation (Z-axis)')
 disp('2: Pitch Rotation (Y-axis)')
  disp('3: Roll Rotation (X-axis)')
   disp('4: Body to Inertial Matrix (R_IB)')
disp('5: Inertial to Body Matrix (R_BI)')
disp('6: Exit')
    
  ch =input('Choose option: ');
    
 switch ch
  case 1
   psi= input('Enter Yaw angle (psi) in degrees: ');
     R =Rz(psi);
   t_str = sprintf('Yaw by %g deg',psi);
            
   case 2
  theta =input('Enter Pitch angle (theta) in degrees: ');
  R= Ry(theta);
     t_str= sprintf('Pitch by %g deg',theta);
            
  case 3
phi= input('Enter Roll angle (phi) in degrees: ');
   R =Rx(phi);
    t_str =sprintf('Roll by %g deg', phi);
            
 case 4
  psi = input('Enter Yaw (psi) in deg: ');
    theta= input('Enter Pitch (theta) in deg: ');
   phi = input('Enter Roll (phi) in deg: ');
            
  R=Rz(psi)* Ry(theta)* Rx(phi);
   t_str ='Body to Inertial (R_{IB})';
            
   case 5
  psi= input('Enter Yaw (psi) in deg: ');
  theta = input('Enter Pitch (theta) in deg: ');
    phi= input('Enter Roll (phi) in deg: ');
            
  R_IB= Rz(psi)*Ry(theta)*Rx(phi);
   R = R_IB'; 
    t_str = 'Inertial to Body (R_{BI})';
            
   case 6
  disp('Exiting...');
   break;
            
  otherwise
   disp('Invalid input. Please try again.');
     continue;
  end
    
disp(' ');
 disp('Transformation Matrix R:');
  disp(R);
    
v_rot =R *v;
    
disp('Rotated Vector:');
 disp(v_rot);
    
 figure;hold on;grid on;
 quiver3(0,0,0,v(1),v(2),v(3), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.1);
  quiver3(0,0,0,v_rot(1),v_rot(2),v_rot(3), 0, 'r', 'LineWidth', 2,'MaxHeadSize', 0.1);
    
 legend('Original Vector','Rotated Vector','Location', 'best');
  xlabel('X-axis'); ylabel('Y-axis');zlabel('Z-axis');
 title(t_str);
    
 max_val=max(abs([v;v_rot]))* 1.5;
   if max_val == 0
   max_val = 1; 
   end 
    
xlim([-max_val max_val]); 
  ylim([-max_val max_val]); 
 zlim([-max_val max_val]);
 view(3); 
   hold off;
end