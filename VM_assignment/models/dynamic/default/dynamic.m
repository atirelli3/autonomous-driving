%% Load external path
root_folder = fileparts(fileparts(pwd));
%  * Get the current working directory (dynamical model)
model_folder = fileparts(pwd);

%  * Construct the new path
model_utils_path = [model_folder, '/utils'];
common_sym_path = [root_folder, '/common'];

%  * Add the new path to the MATLAB search path
addpath(common_sym_path);
addpath(model_utils_path);

%% Simulation parameters
%  Movement parameteres
v = 50 * 1000 / 3600;   % Vehicle speed in m/s

%  Signal parameters
freq = 0.5;             % Frequency of sinusoidal steering input in Hz
amp_steering = 30;      % Amplitude of sinusoidal steering input in rad

%  Time parameteres
t = 0;          % Simulation time start
t_end = 10;     % Simulation time end
dt = 0.1;       % Simulation time step

t_pos = 0;

%% Vehicle parameteres
%  Physical parameteres
mass = 2164;        % Vehicle mass in kg
%  Forces parameteres
inertia = 4373;     % Inertia in kg*m^2
%  Geometry parameteres
l_f = 1.3384;       % Distance from the center of mass to front axle
l_r = 1.6456;       % Distance from the center of mass to rear axle
%  Constant parameteres
C_f = 1.0745e5;     % Front cornering stiffness coefficient
C_r = 1.9032e5;     % Rear cornering stiffness coefficient

%% Vehicle model
%  Dynamic model
%  Initialization parameters
%  * Easy vars name for the forumals
v_x = v;
i_x = inertia;

%  * Speed dependent vars in the matrix A
Ay_dot_dot_2 = ((2*C_f + 2*C_r) / (mass*v_x));
Ay_dot_dot_4 = v_x - ((2*C_f*l_f - 2*C_r*l_r) / (mass*v_x));

Apsi_dot_dot_2 = ((2*C_f*l_f - 2*C_r*l_r) / (i_x*v_x));
Apsi_dot_dot_4 = ((2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_x*v_x));

%  * Speed dependent vars in the matrix B
By_dot_dot = 2*C_f / mass;          % We use only the coefficents and
Bpsi_dot_dot = 2*l_f*C_f / i_x;     % constants of Front tire because the
                                    % the assumtion is Rear = 0 slip angle.
                                    % In more easy term, the vehicle is not
                                    % a 4x4.

%  Dynamic lateral model matrix A, B
A = [0 1 0 0;
    0 -Ay_dot_dot_2 0 -Ay_dot_dot_4;
    0 0 0 1;
    0 -Apsi_dot_dot_2 0 -Apsi_dot_dot_4];

B = [0; By_dot_dot; 0; Bpsi_dot_dot];

%  State vectors
%  * Support vars
y = 0;
y_dot = 0;
psi = 0;
psi_dot = 0;
delta = 0;

%  * States vectors
x = [y; y_dot; psi; psi_dot];       % Lateral position state (v_x depend)
u = delta;                          % Input singal state (steering angle)

%  * Position in the global fram
X = 0;                              % Coordinate X in the global frame
Y = 0;                              % Coordinate Y in the global frame
global_frame_pos = [X; Y];          % Global frame position (X,Y)

%  * Other storage support struct
global_u = u;
global_c = 0;
global_sf = 0;
global_sr = 0;
global_vs = 0;

%% Support anonymous fun
%  Calculate the slip angle in relation of the time (t)
slip = @(t) deg2rad(30 * sin(2 * pi * freq * t));
%  Load the external function
xdot = @xdot;
ypsi2pos = @ypsi2pos;
plotter = @plotter;

%% Simulation
while t<=t_end
    u = slip(t);
    [tsol, xsol] = ode45(@(t,x) xdot(x, u, A, B), [t t+dt], x(:,end));
    x = [x xsol(end,:)'];

    % Calculate the position in the global frame
    [tsol_pos, xsol_pos] = ode45(@(t,global_frame_pos) ypsi2pos(x, v_x), [t_pos t_pos+dt], global_frame_pos(:,end));
    global_frame_pos = [global_frame_pos xsol_pos(end,:)'];
    
    v_s = x(2, end) / v_x;
    course_f = v_s + x(3, end);

    slip_f = u - ((x(2,end) + l_f*x(4,end)) / v);
    slip_r = -(x(2,end) + l_r*x(4,end)) / v;

    % Update the store structs
    global_u = [global_u u'];
    global_vs = [global_vs v_s'];
    global_c = [global_c course_f'];
    global_sf = [global_sf slip_f'];
    global_sr = [global_sr slip_r'];

    t = t + dt;
end

%% Plot the result
%  Re-fetch the time simulation
t_plot = 0:dt:t_end+dt;

%  Transfer sim datas into vars to send at the plotter
position = global_frame_pos;        % Global position
vehicle_heading = x(3, :);          % Vehicle heading
slip_angle = global_vs;             % Slip angle
course_angle = global_c;            % Course angle
steering_angle = global_u(1, :);    % Steering angle (front)

%  Plot
plotter(t_plot, x, position, vehicle_heading, slip_angle, course_angle, steering_angle, []);