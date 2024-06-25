function K = design_k(A, B, delta, w_n)
    % Pole computation
    p1 = -delta*w_n + 1i*w_n*sqrt(1 - delta^2); % Dominant eigenvalue +
    p2 = -delta*w_n - 1i*w_n*sqrt(1 - delta^2); % Dominant eigenvalue -
    p = [p1; p2; -50; -100];                    % eigenvalues vector
    
    % Design K placing the eigenvalues in the space
    K = place(A,B,p);
end