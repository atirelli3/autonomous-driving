% Bicycle Dynamical model ODE function /w road allignment
function xdot_ra = xdot_ra(x, u, A, B, B_d, d)
    xdot_ra = A*x+B*u+B_d*d;
end