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
v = 30 * 1000 / 3600;   % Vehicle speed in m/s

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
i_z = inertia;

R = 9999999999;     % Path

%  * Speed dependent vars (erros => e_n) in the matrix A
e_1_dot_2 = (2*C_f + 2*C_r) / (mass*v_x);
e_1_dot_3 = (2*C_f + 2*C_r) / mass;
e_1_dot_4 = ((-2*C_f*l_f) + 2*C_r*l_r) / (mass*v_x);

e_2_dot_2 = (2*C_f*l_f - 2*C_r*l_r) / (i_z*v_x);
e_2_dot_3 = (2*C_f*l_f - 2*C_r*l_r) / i_z;
e_2_dot_4 = (2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_z*v_x);

%  * Orientations vars (delta_n) in the matrix B
delta_2 = (2*C_f) / mass;
delta_4 = (2*l_f*C_f) / i_z;

%  * ---
psides_dot_2 = (-((2*C_f*l_f - 2*C_r*l_r) / (mass*v_x)) - v_x);
psides_dot_4 = (2*C_f*(l_f^2) + 2*C_r*(l_r^2)) / (i_z*v_x);

%  Dynamic lateral model matrix A, B
A = [0 1 0 0;
    0 -e_1_dot_2 e_1_dot_3 e_1_dot_4;
    0 0 0 1;
    0 -e_2_dot_2 e_2_dot_3 -e_2_dot_4];

B = [0; delta_2; 0; delta_4];

B_d = [0; psides_dot_2; 0; -psides_dot_4];

%  State vectors
%  * Support vars
e1 = 0;
e1_dot = 0;
e2 = 0;
e2_dot = 0;
delta = 0;

%  * States vectors
x = [e1; e1_dot; e2; e2_dot];       % Lateral position state (error of desired path)
u = delta;                          % Input singal state (steering angle)

%  * Position in the global fram
X = 0;                              % Coordinate X in the global frame
Y = 0;                              % Coordinate Y in the global frame
pos = [X; Y];
desired_pos = [X; Y];               % Desire position (X,Y)
global_frame_pos = [X; Y];          % Global frame position (X,Y)

%  * Other storage support struct
global_h = 0;       % Vehicle heading
global_u = u;       % Steering angle
global_s = 0;       % Slip angle
global_c = 0;       % Course angle

%% Support anonymous fun
%  Calculate the slip angle in relation of the time (t)
slip = @(t) deg2rad(30 * sin(2 * pi * freq * t));
xdot_ra = @xdot_ra;

%% Integral section
psi_des_dot = v_x/R;
syms psi_des(i) x_des(i) y_des(i)

% psi_des_dot(i) = v_x / R;                     % Calculate psi_des_dot
psi_des(i) = int(psi_des_dot, 0, i);            % Integrate psi_des_dot to get psi_des
x_des(i) = int(v_x * cos(psi_des(i)), 0, i);    % Integrate v*cos(psi_des) to get x_des
y_des(i) = int(v_x * sin(psi_des(i)), 0, i);    % Integrate v*sin(psi_des) to get y_des

%% Simulation
while t<=t_end
    % Update u with the give input signal of steering angle
    u = slip(t);

    % Integrate psi_des_dot, psi_des, x_des, y_des
    psi_des_T = psi_des(t);
    x_des_T = x_des(t);
    y_des_T = y_des(t);

    d = psi_des_dot;

    [tsol, xsol] = ode45(@(t,x) xdot_ra(x, u, A, B, B_d, d), [t t+dt], x(:,end));
    x = [x xsol(end,:)'];  
    
    % Update the desired position
    pos_des = [x_des_T y_des_T];
    desired_pos = [desired_pos pos_des'];

    % Update the global position
    global_x = x_des_T - x(1,end)*sin(psi_des_T);
    global_y = y_des_T + x(1,end)*cos(psi_des_T);
    global_pos = [global_x global_y];

    global_frame_pos = [global_frame_pos global_pos'];

    % Update the vehicle heading
    psi = x(3, end) + psi_des_T;
    global_h = [global_h psi'];

    % Update the slip angle
    vehicle_slip = (1/v_x)*x(2, end) - x(3, end);
    global_s = [global_s vehicle_slip'];

    % Update the course angle
    course = psi + vehicle_slip;
    global_c = [global_c course'];

    % Update steering angle
    global_u = [global_u u'];

    t = t + dt;
end

%% Plot the result
%  Re-fetch the time simulation
t_plot = 0:dt:t_end+dt;

%  Transfer sim datas into vars to send at the plotter
position = global_frame_pos;        % Global position
vehicle_heading = global_h;         % Vehicle heading
slip_angle = global_s;              % Slip angle
course_angle = global_c;            % Course angle
steering_angle = global_u(1, :);    % Steering angle

%  Plot
plotter(t_plot, x, position, vehicle_heading, slip_angle, course_angle, steering_angle, desired_pos);