function xdot_pp = xdot_pp(x, A, B, K, K_r, r)
    xdot_pp = (A - B*K)*x + K_r*r;
end