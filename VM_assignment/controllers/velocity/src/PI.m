function PI = PI(t, v_x, v_des, k_p, k_i)
    x_des = PI_integrator(t, v_x, v_des);

    PI = -k_p*(v_x - v_des) - k_i*x_des;
end