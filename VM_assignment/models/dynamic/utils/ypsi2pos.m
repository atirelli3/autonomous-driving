% Compute Y and PSI to obtain position_dot(x_dot, y_dot)
function ypsi2pos = ypsi2pos(x, v_x)
    v_y = x(2, end);    % Using the notation V_y = y_dot (ref. 30)
    psi = x(3, end);    % Vehicle yaw_rate

    % Global position transformation
    xdot = v_x * cos(psi) + v_y * sin(psi);
    ydot = v_y * cos(psi) + v_x * sin(psi);

    ypsi2pos = [xdot; ydot];
end