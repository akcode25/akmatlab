% coeff of friction

m = [2, 4, 5, 10, 20, 50];    
F = [12.5, 23.5, 30, 61, 117, 294]; 
g = 9.81;   

mu = F ./ (m * g);
mu_avg = mean(mu);

mu
mu_avg
% disp(['Average coefficient of friction: ', num2str(mu_avg)]);