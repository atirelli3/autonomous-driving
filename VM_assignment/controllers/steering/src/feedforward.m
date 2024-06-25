function delta_ff = feedforward(mass, v_x, l_f, l_r, C_f, C_r, R, K)
    L = l_f + l_r;

    delta_ff = mass * v_x^2 / (R*L) * (l_r / (2*C_f) - l_f / (2*C_r) + l_f / (2*C_r) * K(3)) + L/R - l_r / R*K(3);
end


