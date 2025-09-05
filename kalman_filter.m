function x_filtered = kalman_filter(y, x_true, enable_plotting)
    % Kalman Filter with integrated plotting for Simulink
    % Inputs:
    %   y - measurement vector (3x1)
    %   x_true - true state for comparison (3x1) [optional]
    %   enable_plotting - flag to enable/disable plotting (1/0) [optional]
    % Output: 
    %   x_filtered - filtered state estimate (6x1)
    
    % Handle optional inputs
    if nargin < 3
        enable_plotting = 0; % Default: no plotting
    end
    if nargin < 2
        x_true = zeros(6,1); % Default: unknown true state
    end
    
    % Persistent variables to maintain state between function calls
    persistent x_hat P A Q R H dt initialized k_step
    
    % Initialize on first call
    if isempty(initialized)
        % System parameters
        dt = 0.01;
        k_step = 0;
        
        % Process matrix A
        A = [1, 0, 0, dt, 0,  0;
             0, 1, 0, 0,  dt, 0;
             0, 0, 1, 0,  0,  dt;
             0, 0, 0, 1,  0,  0;
             0, 0, 0, 0,  1,  0;
             0, 0, 0, 0,  0,  1];
        
        % Process noise covariance Q
        sigma_xy = 0.01;
        sigma_theta = deg2rad(4);
        sigma_xyDot = sigma_xy;
        sigma_thetaDot = sigma_theta;
        Q = diag([sigma_xy, sigma_xy, sigma_theta, sigma_xyDot, sigma_xyDot, sigma_thetaDot ]);
        % Measurement noise covariance R

        % show two figure beside each other
        % 150 % stationary
        % 15 - Factor 10
        % 25 - Tune of Low Values
        R_factor = 30;
        % 1 - init
        % 100 - Tune Stationary
        % 200 - HighTheta
        % 50 - LowTheta
        % 60 - LowThetaHighXY
        % 6 - Factor 10
        % 6 - Tune of Low Values
        R_thetaFactor = 5;
        R = diag([sigma_xy * R_factor, sigma_xy * R_factor, sigma_theta * R_thetaFactor]);
        
        % Measurement matrix H
        H = [1, 0, 0, 0, 0, 0;
             0, 1, 0, 0, 0, 0;
             0, 0, 1, 0, 0, 0];
        
        % Initial state estimate
        x_hat = [0; 0; 0; 0; 0; 0];
        
        % Initial error covariance matrix
        % 1000 - init
        % 100 - To Point Tune
        P = eye(6) * 100;  % Large initial uncertainty

        initialized = true;
    end
    
    % Increment time step
    k_step = k_step + 1;
    
    % Kalman Filter Algorithm
    
    % 1. Prediction step
    x_pred = A * x_hat;           % Predicted state
    P_pred = A * P * A' + Q;      % Predicted error covariance
    
    % Innovation (measurement residual)
    z = y - H * x_pred;
    
    % Innovation covariance
    S = H * P_pred * H' + R;
    
    % Kalman gain
    K = P_pred * H' / S;
    
    % Updated state estimate
    x_hat = x_pred + K * z;
    
    % Updated error covariance
    I = eye(6);
    P = (I - K * H) * P_pred;

    % Output filtered state
    x_filtered = x_hat;
    
    % Call plotting function if enabled
    if enable_plotting
        % Extract Kalman gain for x-position (first element of first row)
        k_gain_x = K(1, 1);
        
        % Call the plotting function
        kalman_plot_realtime(x_hat, y, x_true, k_gain_x, k_step);
    end
end
function kalman_plot_realtime(x_estimated, y_measured, x_true, k_gain, k_step)
    % Real-time plotting function for Kalman Filter in Simulink
    % Inputs:
    %   x_estimated - Current estimated state (6x1) (x,y,theta, xdot, ydot, thetadot)
    %   y_measured  - Current measurement (3x1) (x,y,theta)
    %   x_true      - Current true state (3x1) (x,y,theta)
    %   k_gain      - Current Kalman gain (first element for x-position)
    %   k_step      - Current time step
    
    % Persistent variables to store data history
    persistent xk_hat_history yk_history xk_history Kv_history time_history
    persistent fig_handle initialized max_points
    persistent error_history error_fig_handle
    
    % Initialize on first call
    if isempty(initialized)
        % Set maximum number of points to display (for performance)
        max_points = 10000;
        
        % Initialize history arrays
        xk_hat_history = zeros(2, max_points);
        yk_history = zeros(2, max_points);
        xk_history = zeros(2, max_points);
        Kv_history = zeros(1, max_points);
        time_history = zeros(1, max_points);

        % Error history (x,y,theta)
        error_history = zeros(3, max_points);
        
        % Create figures
        fig_handle = figure('Name', 'Figure 2 Kalman Filter Real-time Results', ...
                            'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);
        error_fig_handle = figure('Name', 'Figure 3 Kalman Filter Estimation Error', ...
                                  'NumberTitle', 'off', 'Position', [1400, 100, 800, 600]);
        
        initialized = true;
    end
    
    % Extract position data (first 2 elements)
    if length(x_estimated) >= 2
        x_est_pos = x_estimated(1:2);
    else
        x_est_pos = x_estimated;
    end
    
    if length(y_measured) >= 2
        y_meas_pos = y_measured(1:2);
    else
        y_meas_pos = y_measured;
    end
    
    if length(x_true) >= 2
        x_true_pos = x_true(1:2);
    else
        x_true_pos = x_true;
    end
    
    % Determine current index (circular buffer)
    current_idx = mod(k_step - 1, max_points) + 1;
    
    % Store current data
    xk_hat_history(:, current_idx) = x_est_pos;
    yk_history(:, current_idx) = y_meas_pos;
    xk_history(:, current_idx) = x_true_pos;
    Kv_history(current_idx) = k_gain; % Store first element of Kalman gain
    time_history(current_idx) = k_step;

    % Store error (x, y, theta if available)
    if length(x_estimated) >= 3 && length(x_true) >= 3
        error_history(:, current_idx) = x_estimated(1:3) - x_true(1:3);
    else
        error_history(1:2, current_idx) = x_est_pos - x_true_pos;
    end
    
    % Determine range for plotting
    if k_step <= max_points
        range = 1:k_step;
        plot_range = 1:k_step;
    else
        % Use circular buffer - show last max_points
        range = 1:max_points;
        plot_range = (k_step - max_points + 1):k_step;
    end
    
    % Update plots every N steps (for performance)
    plot_update_interval = 10;
    if mod(k_step, plot_update_interval) == 0 || k_step <= 10
        %% Main figure (estimation, measurement, true, gain)
        figure(fig_handle);
        
        % X Position Plot
        subplot(2, 2, 1);
        cla;
        plot(plot_range, xk_hat_history(1, range), 'b--', 'LineWidth', 2); 
        hold on;
        plot(plot_range, yk_history(1, range), 'r.', 'MarkerSize', 8);
        plot(plot_range, xk_history(1, range), 'g--', 'LineWidth', 1);
        ylabel('x Position');
        xlabel('Time Step (k)');
        title('State Estimation - X Position');
        legend('Estimated', 'Measured', 'True', 'Location', 'best');
        grid on;
        
        % Y Position Plot
        subplot(2, 2, 2);
        cla;
        plot(plot_range, xk_hat_history(2, range), 'b--', 'LineWidth', 2); 
        hold on;
        plot(plot_range, yk_history(2, range), 'r.', 'MarkerSize', 8);
        plot(plot_range, xk_history(2, range), 'g--', 'LineWidth', 1);
        ylabel('y Position');
        xlabel('Time Step (k)');
        title('State Estimation - Y Position');
        legend('Estimated', 'Measured', 'True', 'Location', 'best');
        grid on;
        
        % X-Y Trajectory Plot
        subplot(2, 2, 3);
        cla;
        plot(xk_hat_history(1, range), xk_hat_history(2, range), 'b--', 'LineWidth', 2);
        hold on;
        plot(yk_history(1, range), yk_history(2, range), 'r.', 'MarkerSize', 6);
        plot(xk_history(1, range), xk_history(2, range), 'g--', 'LineWidth', 1);
        ylabel('y Position');
        xlabel('x Position');
        title('State Estimation - X-Y Trajectory');
        legend('Estimated', 'Measured', 'True', 'Location', 'best');
        grid on;
        axis equal;
        
        % Kalman Gain Plot
        subplot(2, 2, 4);
        cla;
        plot(plot_range, Kv_history(range), 'b-', 'LineWidth', 2);
        ylabel('Kalman Gain (x Position)');
        xlabel('Time Step (k)');
        title('Kalman Gain Evolution');
        grid on;

        %% Error figure
        figure(error_fig_handle);
        
        % X error
        subplot(3, 1, 1);
        cla;
        plot(plot_range, error_history(1, range), 'b-', 'LineWidth', 1.5);
        ylabel('x error');
        xlabel('Time Step (k)');
        title('Estimation Error - X');
        grid on;
        
        % Y error
        subplot(3, 1, 2);
        cla;
        plot(plot_range, error_history(2, range), 'r-', 'LineWidth', 1.5);
        ylabel('y error');
        xlabel('Time Step (k)');
        title('Estimation Error - Y');
        grid on;
        
        % Theta error (only if available)
        subplot(3, 1, 3);
        cla;
        plot(plot_range, error_history(3, range), 'g-', 'LineWidth', 1.5);
        ylabel('\theta error');
        xlabel('Time Step (k)');
        title('Estimation Error - Theta');
        grid on;

        % Force drawing update
        drawnow;
    end
end

