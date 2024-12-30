%Temperature after 3 hrs

To = 120;  % Init temp of soda can
Ts = 38;   % Const temp of refrigerator
k = 0.45;  % Given const
t = 3;     % Time

T = Ts + (To - Ts) * exp(-k * t); % Calc temp after 3 hrs
T

%fprintf('Temp after 3 hrs: %f deg F',T);
%disp('Temp after 3 hrs:')
%disp(T);