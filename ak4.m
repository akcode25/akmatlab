% piston-rod-crank

r = 120; l = 250; rpm = 500; omega = 2 * pi * rpm / 60;
t = linspace(0, 2*pi/omega, 1000);
theta = omega * t;

x = r * cos(theta) + sqrt(l^2 - (r * sin(theta)).^2);
v = gradient(x) ./ gradient(t);  % Vel- derivative of posn wrt time
a = gradient(v) ./ gradient(t);  % Acc- derivative of vel wrt time

plot(t, x, 'b', t, v, 'r', t, a, 'g'); grid on; 
legend('Position (mm)', 'Velocity (mm/s)', 'Acceleration (mm/s^2)');
xlabel('Time (s)'); title('Piston Position, Velocity, and Acceleration');

