% projectile

v0 = 250; theta = 65; t_wind = 30; g = 9.81;
t_flight = 2*v0*sin(theta)/g;     % time of flight without wind
t = linspace(0, t_flight , 1000); % time vector

z = v0 * sin(theta) * t - 0.5 * g * t.^2; % Height
y = v0 * cos(theta) * t;                  % Distance north
x = t_wind * t;                           % Distance west cuz wind

plot3(0*t, y, z, 'b', x, y, z, 'r');   % No wind (x = 0)  % With wind

% or,
% plot3(0*t, y, z, 'b');  % no wind
% hold on;
% plot3(x, y, z, 'r');   % with wind

grid on; xlabel('West (m)'); ylabel('North (m)'); zlabel('Height (m)');
legend('No Wind', 'With Wind'); title('Projectile Trajectory');
