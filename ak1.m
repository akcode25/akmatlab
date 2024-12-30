% tigonometric identity

x = pi/5;

LHS = cos(x/2)^2;
RHS = (tan(x) + sin(x)) / (2 * tan(x));
disp(['LHS: ', num2str(LHS)]);
disp(['RHS: ', num2str(RHS)]);

% Verify if LHS is approximately equal to RHS
if abs(LHS - RHS) < 1e-10  % Tolerance level for floating-point comparison
   disp('The identity holds true.');
else
   disp('The identity does not hold.');
end
