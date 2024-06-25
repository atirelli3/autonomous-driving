function [Q, K, S, CLP] = lq_regolator(A, B)
    Q = 0.01;
    R = 10;
    N = 0;
    [K,S,CLP] = lqr(A, B, Q, R, N);
end