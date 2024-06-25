function y = step_response(t, w_n, delta)
    % Calculate step response based on the given formula
    w = w_n * sqrt(1 - delta^2);
    phi = acos(delta);
    y = 1 - (exp(-delta * w_n * t) / sqrt(1 - delta^2)) * sin(w * t + phi);
end
