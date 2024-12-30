% projection trajectory

function [h_max, d_max] = projectile(v0, theta)
    g = 9.81;
    theta = deg2rad(theta);
    h_max = (v0^2 * sin(theta)^2) / (2 * g);
    d_max = (v0^2 * sin(2 * theta)) / g;
    t = linspace(0, 2 * v0 * sin(theta) / g, 100);

    x = v0 * cos(theta) * t;  % Horizontal position
    y = v0 * sin(theta) * t - 0.5 * g * t.^2;  % Vertical position
    plot(x, y);
    xlabel('Horizontal Distance (m)'); ylabel('Vertical Distance (m)');
    grid on;
end

%% ak8.m
% [v0, theta] = deal(230, 39);
% [h, d] = projectile(v0, theta);
% fprintf('Max Height: %.2f m, Max Distance: %.2f m\n', h, d);