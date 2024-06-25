function u = steering_angle_u(t, freq)
    % Calculate steering angle as a function of time
    u = deg2rad(30 * sin(2 * pi * freq * t));
end
