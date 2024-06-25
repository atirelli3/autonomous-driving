function K = power_train(v_interval, flag)
% flag = true => Pole Placement, call design_k
%
% flag = false => LQ Regolator, call lq_regolator

d_vx = 10;

% Vehicle geomtry
[mass, i_z, l_f, l_r, C_f, C_r] = vehicle_geometry();

v_min = v_interval(1) * 1000 / 3600;
v_max = v_interval(2) * 1000 / 3600;

% Min v_x in the velocity interval
% Road Allignment system matrices A, B, B_d
[A_min, B_min, B_dmin] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_min);

% Max v_x in the velocity interval
% Road Allignment system matrices A, B, B_d
[A_max, B_max, B_dmax] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_max);

% Steering constraints
[Ts, overshoot_max] = steering_constraints();

% Controller constraints
[delta, S, w_n] = controller_constraints(Ts, overshoot_max);

if flag 
    K_min = design_k(A_min, B_min, delta, w_n);
    K_max = design_k(A_max, B_max, delta, w_n);
else
    [Q_min, K_min, S_min, CLP_min] = lq_regolator(A_min, B_min);
    [Q_max, K_max, S_max, CLP_max] = lq_regolator(A_max, B_max);
end

% Initialize cell arrays
K = {};
K{end + 1} = K_min;

for v_x = v_interval(1) + 10:d_vx:v_interval(2) - 1
    % Normalize in m/s
    v_x = v_x * 1000 / 3600;

    % Road Allignment system matrices A, B, B_d
    [A, B, B_d] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_x);
    if flag
        K_t = design_k(A, B, delta, w_n);
    else
        [Q_t, K_t, S_t, CLP_t] = lq_regolator(A, B);
    end

    % Add the new controller K_t to K
    K{end+1} = K_t;
end


K{end + 1} = K_max;

end