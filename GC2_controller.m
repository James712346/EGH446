function [v, omega, WP_index_out, done] = GC2_controller(state, waypoints, WP_index_in, lookahead, k, v, reset)
% Guidance + Control using RVWP
% Inputs:
% - state: [x; y; psi]
% - waypoints: Nx2 matrix of global path
% - WP_index_in: current path segment
% - lookahead: look-ahead distance
% - k: heading correction gain
% - reset: 1 to reset internal index
% Outputs:
% - v: linear velocity
% - omega: angular velocity
% - WP_index_out: updated index
% - done: 1 if goal reached

    persistent WP_index nPoints

    % --- Initialization ---
    if isempty(WP_index) || reset
        WP_index = 1;
    end
    nPoints = size(waypoints, 1);

    % If we've reached final segment, stop
    if WP_index >= nPoints
        v = 0;
        omega = 0;
        done = true;
        WP_index_out = WP_index;
        return;
    end

    % --- Current state ---
    x = state(1);
    y = state(2);
    psi = state(3);

    % --- Current + next waypoints ---
    wp1 = waypoints(WP_index, :);
    wp2 = waypoints(WP_index + 1, :);

    % --- Distance to current segment end ---
    d = norm([x - wp2(1), y - wp2(2)]);

    % --- Compute virtual waypoint
    Ru = norm([x - wp1(1), y - wp1(2)]);
    theta = atan2(wp2(2) - wp1(2), wp2(1) - wp1(1));
    theta_u = atan2(y - wp1(2), x - wp1(1));
    beta = angdiff(theta_u, theta);
    R = sqrt(Ru^2 - (Ru * sin(beta))^2);

    % Virtual target (RVWP)
    S_x = wp1(1) + (R + lookahead) * cos(theta);
    S_y = wp1(2) + (R + lookahead) * sin(theta);

    % --- Heading correction ---
    theta_d = atan2(S_y - y, S_x - x);
    heading_error = angdiff(psi, theta_d);
    omega = k * heading_error;

    % --- Advance segment if near wp2 ---
    if d < lookahead
        WP_index = WP_index + 1;
    end

    WP_index_out = WP_index;
    done = false;
end
