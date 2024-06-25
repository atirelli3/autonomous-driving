% Bicycle model ODE function
function xdot = xdot(t, x, u, beta, l_f, l_r)
    xdot = [u(1)*cos(x(3)+beta); u(1)*sin(x(3)+beta); (u(1)*cos(beta)/(l_f + l_r))*(tan(u(2))-tan(u(3)))];
end