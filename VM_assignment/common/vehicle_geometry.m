function [mass, inertia, l_f, l_r, C_f, C_r] = vehicle_geometry()
    mass = 2164;    % Vehicle mass in kg
    inertia = 4373; % Inertia in kg*m^2
    l_f = 1.3384;   % Distance from the center of mass to front axle
    l_r = 1.6456;   % Distance from the center of mass to rear axle
    C_f = 1.0745e5; % Front cornering stiffness coefficient
    C_r = 1.9032e5; % Rear cornering stiffness coefficient
end
