% projection trajectory

[v0, theta] = deal(230, 39);
[h, d] = projectile(v0, theta);
fprintf('Max Height: %.2f m, Max Distance: %.2f m\n', h, d);