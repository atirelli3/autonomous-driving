function [psi_des_dot, psi_des, x_des, y_des] = road_aligned_integrator(v_x, R, t_i)
    % Integral function
    
    % Symbolic variables
    syms psi_des_dot_sym(t) psi_des_sym(t) x_des_sym(t) y_des_sym(t)
    psi_des_dot_sym(t) = piecewise(t <= 10, 0, t>10, v_x/R);

    % Integrate psi_des_dot to get psi_des
    psi_des_sym(t) = int(psi_des_dot_sym(t), 0, t);

    % Integrate v*cos(psi_des) to get x_des
    x_des_sym(t) = int(v_x * cos(psi_des_sym(t)), 0, t);

    % Integrate v*sin(psi_des) to get y_des
    y_des_sym(t) = int(v_x * sin(psi_des_sym(t)), 0, t);

    % Evaluate the symbolic functions at the given time points
    psi_des_dot = double(psi_des_dot_sym(t_i));
    psi_des = double(psi_des_sym(t_i));
    x_des = double(x_des_sym(t_i));
    y_des = double(y_des_sym(t_i));
end
