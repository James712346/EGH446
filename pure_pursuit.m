function [v, omega] = pure_pursuit(state, target, v)
    % Relative Position in the inertial Frame
    dx = target(1) - state(1);
    dy = target(2) - state(2);

    gamma = atan2(dy,dx);
    theta=state(3);

    omega = 0.8 * angdiff(theta, gamma);
end
