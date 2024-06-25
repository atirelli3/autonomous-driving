function PI_controller = PI_controller(t, tau, x)
    PI_controller = 1/(tau*t + 1)*x;
end