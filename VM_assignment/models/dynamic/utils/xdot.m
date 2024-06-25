% Bicycle Dynamical model ODE function
function xdot = xdot(x, u, A, B)
    xdot = A*x + B*u;
end