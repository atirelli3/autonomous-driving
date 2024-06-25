function plotter2(t_plot, heading, heading_err, step_response, speed, accel)
    figure;

    % Heading errors
    subplot(2, 2, 1);
    plot(t_plot, heading_err, 'g-', 'LineWidth', 1.5);
    hold on;
    plot(t_plot, heading, 'r-', 'LineWidth', 1.5);
    title("Heading errors");
    xlabel('Time (s)');
    ylabel('Heading (°)');
    grid on;
    legend('Expected', 'State');

    % Step response
    subplot(2, 2, 2);
    plot(t_plot, step_response, 'c-', 'LineWidth', 1.5);
    title('Step response');
    xlabel('Time (s)');
    ylabel('Step response (y(t))');
    grid on;

    % Longitudinal speed
    subplot(2, 2, 3);
    plot(t_plot, speed * 3.6, 'm-', 'LineWidth', 1.5);
    title('Longitudinal speed');
    xlabel('Time (s)');
    ylabel('Speed (km/h)');
    grid on;

    % Acceleration
    subplot(2, 2, 4);
    plot(t_plot, accel, 'k-', 'LineWidth', 1.5);
    title('Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    grid on;
end