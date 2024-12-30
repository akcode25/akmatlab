% box thickness

function thickness = box_thickness(weight)
    L = 24; W = 12; H = 4;     % Outer dimensions
    specific_weight = 0.101;   % Aluminium
    eq = @(x) specific_weight * (L*W*H - (L-2*x)*(W-2*x)*(H-x)) - weight;
    thickness = fzero(eq, 0.1); % Initial guess of 0.1 for thickness 
end

thickness = box_thickness(15);
fprintf('The thickness of the box is %.4f inches\n', thickness);
