function pole_placement_xdot = pole_placement_xdot(x, A, B, K, B_d, d)
    pole_placement_xdot = (A - B*K)*x + B_d*d;
end