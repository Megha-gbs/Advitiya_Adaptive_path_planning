% STANLEY_CONTROLLER - Front-axle reference lateral controller
% Inputs:
%   ego_pose    : [X, Y, psi] in global frame (m, m, rad)
%   v           : current forward velocity (m/s)
%   trajectory  : reference trajectory struct [time, x, y, yaw, velocity, curvature]
%   cp          : controller parameters struct (cp.stanley)
%   vp          : vehicle parameters struct
% Outputs:
%   delta_cmd   : front steering angle command (rad)
%   e_fa        : front axle cross-track error (m)
%   theta_e     : heading error (rad)
%   closest_idx : index of closest trajectory point
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [delta_cmd, e_fa, theta_e, closest_idx] = stanley_controller(ego_pose, v, trajectory, cp, vp)
    X   = ego_pose(1);
    Y   = ego_pose(2);
    psi = ego_pose(3);

    % Front axle location
    X_front = X + vp.Lf * cos(psi);
    Y_front = Y + vp.Lf * sin(psi);

    % Find closest waypoint to front axle
    dx_all = trajectory.x - X_front;
    dy_all = trajectory.y - Y_front;
    [~, closest_idx] = min(dx_all.^2 + dy_all.^2);

    % Reference point and reference heading
    x_ref   = trajectory.x(closest_idx);
    y_ref   = trajectory.y(closest_idx);
    yaw_ref = trajectory.yaw(closest_idx);

    % 1. Heading error: theta_e = yaw_ref - psi
    theta_e = wrapToPi(yaw_ref - psi);

    % 2. Cross-track error at front axle
    % Positive when front axle is to the left of the path vector
    dx = X_front - x_ref;
    dy = Y_front - y_ref;
    e_fa = -sin(yaw_ref) * dx + cos(yaw_ref) * dy;

    % 3. Cross-track steering term with softening velocity factor
    v_eff = max(0.2, v);
    delta_e = -atan2(cp.stanley.k_e * e_fa, v_eff + cp.stanley.k_soft);

    % 4. Curvature feedforward
    kappa_ref = trajectory.curvature(closest_idx);
    delta_ff = atan(vp.L * kappa_ref);

    % Total steering command
    delta_cmd = theta_e + delta_e + 0.85 * delta_ff;

    % Clamp to physical steering limits
    delta_cmd = max(vp.delta_min, min(vp.delta_max, delta_cmd));
end
