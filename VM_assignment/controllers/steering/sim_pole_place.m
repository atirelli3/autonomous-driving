clear all, close all, clc

%% Envirorment imports
% project root directory
root_folder = fileparts(fileparts(pwd));
% controllers root directory
controllers_folder = fileparts(pwd);
% common directory
common_sym_path = [root_folder, '/common'];
% models directory
models_folder = [root_folder, '/models/src'];
% road aligned directory
road_aligned_folder = [models_folder, '/road_aligned'];
% controllers source directory
controllers_src_folder = [pwd, '/src'];
% pole placement directory
pole_placement_folder = [controllers_src_folder, '/pole_placement'];
% velocity controllers directory
velocity_controlleres_folder = [controllers_folder, '/velocity/src'];

% Add the new path to the MATLAB search path
addpath(common_sym_path);               % common directory
addpath(road_aligned_folder);           % road aligned directory
addpath(controllers_src_folder)         % controllers source directory
addpath(pole_placement_folder);         % pole placement directory
addpath(velocity_controlleres_folder);  % velocity controllers directory

%% Simulation model definitions
% Vehicle geomtry
[mass, i_z, l_f, l_r, C_f, C_r] = vehicle_geometry();

% Simulation envirorment
[v_x, freq, amp_steering, t, t_end, dt] = gen_simulation();
R = inf;        % Path radius, start with inf and at t = 10 change to R = 1000
e_start = -2;   % Simulation starting error, 2m

% DO NOT CHANGE - Velocity generalized speed interval
v_static_interval = [30 120];

% Velocity simulation
v_interval = [30 120];
v_des = v_interval(2);                                                  
% Normalize in m/s
v_x = v_interval(1) * 1000 / 3600;
v_des = v_des * 1000 / 3600;

% Road Allignment system matrices A, B, B_d
[A, B, B_d] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_x);

n = size(A,2);      % number of states

% Rank of the reachability matrix
r = rank(ctrb(A,B));

% Check the reachability of the system
if r ~= n
    error('Error! The system is not reachable.');
end

% Steering constraints
[Ts, overshoot_max] = steering_constraints();

% General controller constraints
[delta, S, w_n] = controller_constraints(Ts, overshoot_max);

% Initial state of the simulation
x = [-2; 0; 0; 0];

% Default plot storage
t_plot = 0:dt:t_end*2;      % Simulation time
pos_des = [0; 0];           % Desire position
pos_global = [0; -2; 0];    % Global position
heading = 0;                % Vehicle heading
heading_err = 0;            % Vehicle heading error
slip_angle = 0;             % Slip angle
course_angle = 0;           % Course angle
steering_angle = 0;         % Steering angle

% Additional plot storage for specific request
y_step_response = 0;        % Step response
long_v = v_x;               % Longitudinal speed
accel = 0;                  % Acceleration

% Support struct
reference = []; % Longitudinal reference
desired = [];   % Desire longitudinal position
long_pos = [];  % Longitudinal position

%% Simulation preparation
% Design steering controller K
% K = design_k(A, B, delta, w_n);     % Pole Placement

% Powertraing the controller
K_trained = power_train(v_static_interval, true);

% Find correct parameters k_p and k_i for the PIs
tau = .5;
ratio = .25;
[sys, k_p, k_i, k_pinterval] = find_pic_values(tau, ratio);

e_acc = 1;  % acceleration error attenuation

%% Simulation
while t <= t_end*2
    % Longitudinal reference, velocity dependent
    x_rf = PI_integrator(t, v_x, v_des);
    reference = [reference x_rf];

    % PI
    [tsol, xpi] = ode45(@(t, reference) PI(t, v_x, v_des, k_p, k_i), [t t+dt], reference(:,end));
    desired = [desired xpi(end,:)];

    % PIC
    [tsol, xpic] = ode45(@(t_pic, desired) PI_controller(t, tau, desired), [t t+dt], desired(:,end));
    long_pos = [long_pos -xpic(end,:)];
    
    v_acc = v_x;

    if t ~= 0
        v_acc = v_acc + long_pos(end) / t;
    end

    disp(["actual km/h: " num2str(v_acc * 3.6)]);
    % Choose the correct K controlled based on the actual speed
    if (v_acc * 3.6) >= 30 && (v_acc * 3.6) < 40 - e_acc
        index = 1;
    elseif (v_acc * 3.6) >= 40 && (v_acc * 3.6) < 50 - e_acc
        index = 2;
    elseif (v_acc * 3.6) >= 50 && (v_acc * 3.6) < 60 - e_acc
        index = 3;
    elseif (v_acc * 3.6) >= 60 && (v_acc * 3.6) < 70 - e_acc
        index = 4;
    elseif (v_acc * 3.6) >= 70 && (v_acc * 3.6) < 80 - e_acc
        index = 5;
    elseif (v_acc * 3.6) >= 80 && (v_acc * 3.6) < 90 - e_acc
        index = 6;
    elseif (v_acc * 3.6) >= 90 && (v_acc * 3.6) < 100 - e_acc
        index = 7;
    elseif (v_acc * 3.6) >= 100 && (v_acc * 3.6) < 110 - e_acc
        index = 8;
    elseif (v_acc * 3.6) >= 110 && (v_acc * 3.6) < 120 - e_acc
        index = 9;
    else
        index = 10;
    end

    K = K_trained{index};

    % Road Allignment system matrices A, B, B_d
    [A, B, B_d] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_acc);

    % Road Aligned integrator to calc d, psi_des(t), x_des(t) and y_des(t)
    [d, psi_des_t, x_des_t, y_des_t] = road_aligned_integrator(v_acc, R, t);

    % Feed-forward
    delta_ff = feedforward(mass, v_acc, l_f, l_r, C_f, C_r, R, K);

    % ODE Lateral movement with controller K
    [tsol, xsol] = ode45(@(t, x) xdot(x, A, B, K, B_d, d, delta_ff), [t t+dt], x(:,end));
    x = [x xsol(end,:)'];     % Update the state
    
    % Step response
    y = step_response(t, w_n, delta);

    % Update storages
    heading_t = x(3, end) + psi_des_t;          % PSI
    x_posg = x_des_t - x(1,end)*sin(heading_t); % X global position
    y_posg = y_des_t + x(1,end)*cos(heading_t); % Y global position
    slip_t = (1/v_acc)*x(2, end) - x(3, end);   % Slip angle
    course_t = heading_t + slip_t;              % Course angle
    u = (-K*x(:,end)) + delta_ff;               % Steering angle
    % Expeted heading error
    e2 = (-(l_r / R)) + ((l_f / (2 * C_r * (l_f + l_r))) * ((mass*v_acc^2) / R));
    
    % Desire position plot
    pos_des = [pos_des [x_des_t y_des_t]'];
    % Global position plot
    pos_global = [pos_global [x_posg y_posg heading_t]'];
    % State heading err plot
    heading = [heading x(3, end)'];
    % Expected heading error plot
    heading_err = [heading_err e2'];
    % Slip angle plot
    slip_angle = [slip_angle slip_t'];
    % Course angle plot
    course_angle = [course_angle course_t'];
    % Steering angle plot
    steering_angle = [steering_angle u'];
    % Step response plot
    y_step_response = [y_step_response y'];
    % Longitudinal speed plot
    long_v = [long_v v_acc'];
    % Acceleration plot
    accel = [accel (long_pos(end) / t)'];

    % Move to the next instant (t)
    t = t + dt;

    % Change the path after 10s of the simulation
    if t >= 10.0
        R = 1000;   % Along a circle
    end
end

%%  Plotters
plotter(t_plot, x, pos_global, heading, slip_angle, course_angle, steering_angle, pos_des);
plotter2(t_plot, heading, heading_err, y_step_response, long_v, accel);

figure;
rlocus(sys, k_pinterval);
