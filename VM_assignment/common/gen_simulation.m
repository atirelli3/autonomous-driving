function [v, freq, amp_steering, t, t_end, dt] = gen_simulation()
    % Movement parameters
    v = 70 * 1000 / 3600;   % Vehicle speed in m/s
    
    % Signal parameters
    freq = 0.5;             % Frequency of sinusoidal steering input in Hz
    amp_steering = 30;      % Amplitude of sinusoidal steering input in rad
    
    % Time parameters
    t = 0;          % Simulation time start
    t_end = 10;     % Simulation time end
    dt = 0.1;       % Simulation time step
end