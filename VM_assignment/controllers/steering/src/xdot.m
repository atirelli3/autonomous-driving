function xdot = xdot(x, A, B, K, B_d, d, delta_ff)
    xdot = (A - B*K)*x + B*delta_ff + B_d*d;
end