function plotter(t, x, position, vehicle_heading, slip_angle, course_angle, steering_angle, desired_pos)
figure;

%% Global frame position
subplot(3,2,[1,2]);
if ~isempty(desired_pos)
    plot(desired_pos(1,1:end),desired_pos(2,1:end), 'g-', 'LineWidth', 1.5);
    hold on;
end
hold on;
plot(position(1,1:end),position(2,1:end), 'r-', 'LineWidth', 1.5)
title('Vehicle position in the global frame');
xlabel('Position X (m)');
ylabel('Position Y (m)');
grid on;

if ~isempty(desired_pos)
    legend('Desired path', 'Position in the global frame');
end

index_p = 3;

%% Vehicle heading
if ~isempty(vehicle_heading)
    subplot(3,2,index_p);
    plot(t, rad2deg(vehicle_heading(1,1:end)), 'm-', 'LineWidth', 1.5);
    title('Vehicle heading');
    xlabel('Time (s)');
    ylabel('Vehicle heading (°)');
    grid on;
    index_p = index_p+1;
end

%% Slip angle
if ~isempty(slip_angle)
    subplot(3,2,index_p);
    plot(t, rad2deg(slip_angle(1,1:end)), 'b-', 'LineWidth', 1.5);
    title('Slip angle');
    xlabel('Time (s)');
    ylabel('Slip angle (°)');
    grid on;
    index_p = index_p+1;
end

%% Course angle
if ~isempty(course_angle)
    subplot(3,2,index_p);
    plot(t, rad2deg(course_angle(1,1:end)), 'c-', 'LineWidth', 1.5);
    title('Course angle');
    xlabel('Time (s)');
    ylabel('Course angle (°)');
    grid on;
    index_p = index_p+1;
end

%% Steering angle
if ~isempty(steering_angle)
    subplot(3,2,index_p);
    plot(t, rad2deg(steering_angle(1,1:end)), 'k-', 'LineWidth', 1.5);
    title('Steering angle');
    xlabel('Time (s)');
    ylabel('Steering angle (°)');
    grid on;
    index_p = index_p+1;
end

%% Show better position (and desired position)
figure;

if ~isempty(desired_pos)
    plot(desired_pos(1,1:end),desired_pos(2,1:end), 'g-', 'LineWidth', 1.5);
    hold on;
end
hold on;
plot(position(1,1:end),position(2,1:end), 'r-', 'LineWidth', 1.5);
title('Vehicle position in the global frame');
xlabel('Position X (m)');
ylabel('Position Y (m)');
grid on;

if ~isempty(desired_pos)
    legend('Desired path', 'Position in the global frame');
end
end