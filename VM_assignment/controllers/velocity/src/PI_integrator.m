function x_des = PI_integrator(t_i, v_x, v_des)
    % Symbolic variables
    syms x_des_sym(t)
    x_des_sym(t) = int(v_x - v_des, 0, t);

    x_des = double(x_des_sym(t_i));
end