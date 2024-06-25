function [sys, k_p, k_i, k_pinterval] = find_pic_values(tau, ratio)
    k_pinterval = 0:.01:.75;

    sys = tf([1 ratio], [tau 1 0 0]);

    k_p = .7;
    k_i = k_p * ratio;
end