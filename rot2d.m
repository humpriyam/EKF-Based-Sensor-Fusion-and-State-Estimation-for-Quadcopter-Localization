v_input= input('Enter[x, y]: ');
v =v_input(:);
theta = input('EnterRotation angle theta in degrees: ');

R =[cosd(theta) -sind(theta); 
    sind(theta)  cosd(theta)];            
v_rot= R * v;

figure; hold on; % added hold on so the first arrow doesnt get deleted
quiver(0, 0, v(1), v(2), 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver(0, 0, v_rot(1), v_rot(2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.1);

legend('Original Vector', 'Rotated Vector', 'Location', 'best');
xlabel('X-axis'); ylabel('Y-axis');
title(sprintf('2DRotation by %g degrees', theta));

max_val =max(abs([v; v_rot])) * 1.5;
xlim([-max_val max_val]);ylim([-max_val max_val]);
grid on;
hold off;