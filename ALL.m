% 11111111111111111111111111111111111111111111111111

x = pi/5;
LHS = cos(x/2)^2;
RHS = (tan(x) + sin(x)) / (2 * tan(x));
disp(['LHS: ', num2str(LHS)]);
disp(['RHS: ', num2str(RHS)]);
if abs(LHS - RHS) < 1e-10
   disp('The identity holds true.');
else
   disp('The identity does not hold.');
end

% 22222222222222222222222222222222222222222222222222

To = 120;
Ts = 38; 
k = 0.45;
t = 3; 
T = Ts + (To - Ts) * exp(-k * t);
T

% 3333333333333333333333333333333333333333333333333

m = [2, 4, 5, 10, 20, 50];    
F = [12.5, 23.5, 30, 61, 117, 294]; 
g = 9.81;   
mu = F ./ (m * g);
mu_avg = mean(mu);
mu
mu_avg

% 44444444444444444444444444444444444444444444444444

r = 120; l = 250; rpm = 500; omega = 2 * pi * rpm / 60;
t = linspace(0, 2*pi/omega, 1000);
theta = omega * t;
x = r * cos(theta) + sqrt(l^2 - (r * sin(theta)).^2);
v = gradient(x) ./ gradient(t); 
a = gradient(v) ./ gradient(t); 
plot(t, x, 'b', t, v, 'r', t, a, 'g'); grid on;
legend('Position (mm)', 'Velocity (mm/s)', 'Acceleration (mm/s^2)');
xlabel('Time (s)'); title('Piston Position, Velocity, and Acceleration');

% 55555555555555555555555555555555555555555555555555

v0 = 250; theta = 65; v_wind = 30; g = 9.81;
t_flight = 2*v0*sin(theta)/g; 
t = linspace(0, t_flight , 1000); 
z = v0 * sin(theta) * t - 0.5 * g * t.^2;
y = v0 * cos(theta) * t;                 
x = v_wind * t;                          
plot3(0*t, y, z, 'b', x, y, z, 'r'); 
grid on; xlabel('West (m)'); ylabel('North (m)'); zlabel('Height (m)');
legend('No Wind', 'With Wind'); title('Projectile Trajectory');

% 66666666666666666666666666666666666666666666666666

b = 300000; w = 25000; r = 0.05; inf = 0.02; y = 0;
while b > 0
    y = y + 1; 
    b = b * (1 + r) - w;
    B(y) = b; 
    W(y) = w; 
    w = w * (1 + inf);  
end
plot(1:y, W, 'r', 1:y, [300000 B(1:end-1)], 'b');  
xlabel('Year'); ylabel('Amount ($)'); grid on;
legend('Withdrawals', 'Balance'); title('Withdrawals and Balance');

% 7777777777777777777777777777777777777777777777777

singers = {'John', 'Mary', 'Tracy', 'Mike', 'Katie', 'David'};
randomOrder = randperm(length(singers));
disp('The random performance order is:');
for i = 1:length(singers)
   disp([num2str(i),'. ' singers{randomOrder(i)}]);
   % fprintf('%d. %s\n', i, singers{randomOrder(i)});
end
% disp(singers(randperm(length(singers))));

% 8888888888888888888888888888888888888888888888888

% projectile.m
function [h_max, d_max] = projectile(v0, theta)
    g = 9.81;
    theta = deg2rad(theta);
    h_max = (v0^2 * sin(theta)^2) / (2 * g);
    d_max = (v0^2 * sin(2 * theta)) / g;
    T = v0 * 2 * sin(theta) / g;
    t = linspace(0, T, 100);
    x = v0 * cos(theta) * t; 
    y = v0 * sin(theta) * t - 0.5 * g * t.^2;  
    plot(x, y); grid on;
    xlabel('Horizontal Distance (m)'); ylabel('Vertical Distance (m)');  
end

% ak8.m
[v0, theta] = deal(230, 39);
[h, d] = projectile(v0, theta);
fprintf('Max Height: %.2f m, Max Distance: %.2f m\n', h, d);

% 99999999999999999999999999999999999999999999999

function thickness = box_thickness(weight)
    L = 24; W = 12; H = 4;    
    specific_weight = 0.101;   
    eq = @(x) specific_weight * (L*W*H - (L-2*x)*(W-2*x)*(H-x)) - weight;
    thickness = fzero(eq, 0.1); 
end
thickness = box_thickness(15);
thickness


