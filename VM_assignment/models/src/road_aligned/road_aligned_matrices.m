function [A, B, B_d] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_x)
    % Speed dependent vars (errors => e_n) in the matrix A
    e_1_dot_2 = (2*C_f + 2*C_r) / (mass*v_x);
    e_1_dot_3 = (2*C_f + 2*C_r) / mass;
    e_1_dot_4 = ((-2*C_f*l_f) + 2*C_r*l_r) / (mass*v_x);

    e_2_dot_2 = (2*C_f*l_f - 2*C_r*l_r) / (i_z*v_x);
    e_2_dot_3 = (2*C_f*l_f - 2*C_r*l_r) / i_z;
    e_2_dot_4 = (2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_z*v_x);

    % Orientations vars (delta_n) in the matrix B
    delta_2 = (2*C_f) / mass;
    delta_4 = (2*l_f*C_f) / i_z;

    % ---
    psides_dot_2 = (-((2*C_f*l_f - 2*C_r*l_r) / (mass*v_x)) - v_x);
    psides_dot_4 = (2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_z*v_x);

    % Dynamic lateral model matrix A, B
    A = [0 1 0 0;
         0 -e_1_dot_2 e_1_dot_3 e_1_dot_4;
         0 0 0 1;
         0 -e_2_dot_2 e_2_dot_3 -e_2_dot_4];

    B = [0; delta_2; 0; delta_4];

    B_d = [0; psides_dot_2; 0; -psides_dot_4];
end
