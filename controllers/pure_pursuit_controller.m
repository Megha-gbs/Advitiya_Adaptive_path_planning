% PURE_PURSUIT_CONTROLLER - Geometric lateral path tracking controller
% Inputs:
%   ego_pose    : [X, Y, psi] in global frame (m, m, rad)
%   v           : current forward velocity (m/s)
%   trajectory  : reference trajectory struct [time, x, y, yaw, velocity, curvature]
%   cp          : controller parameters struct (cp.pp)
%   vp          : vehicle parameters struct
% Outputs:
%   delta_cmd   : front steering angle command (rad)
%   e_y         : lateral cross-track error (m)
%   target_idx  : index of selected lookahead target point
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [delta_cmd, e_y, target_idx] = pure_pursuit_controller(ego_pose, v, trajectory, cp, vp)
    X   = ego_pose(1);
    Y   = ego_pose(2);
    psi = ego_pose(3);

    % Dynamic speed-dependent lookahead distance
    Ld = cp.pp.k_dd * v + cp.pp.Ld_min;
    Ld = max(cp.pp.Ld_min, min(cp.pp.Ld_max, Ld));

    % Rear-axle position (Pure Pursuit center of rotation)
    X_rear = X - vp.Lr * cos(psi);
    Y_rear = Y - vp.Lr * sin(psi);

    % Find closest point on trajectory to vehicle
    dx_all = trajectory.x - X_rear;
    dy_all = trajectory.y - Y_rear;
    dist_sq = dx_all.^2 + dy_all.^2;
    [min_dist, closest_idx] = min(dist_sq);

    % Compute signed cross-track error at closest point
    path_yaw = trajectory.yaw(closest_idx);
    dx_c = X_rear - trajectory.x(closest_idx);
    dy_c = Y_rear - trajectory.y(closest_idx);
    e_y = -sin(path_yaw) * dx_c + cos(path_yaw) * dy_c;

    % Search forward from closest point for lookahead point
    N_pts = length(trajectory.x);
    target_idx = closest_idx;
    for i = closest_idx:N_pts
        d_to_pt = sqrt((trajectory.x(i) - X_rear)^2 + (trajectory.y(i) - Y_rear)^2);
        if d_to_pt >= Ld
            target_idx = i;
            break;
        end
    end

    % Lookahead vector in global frame
    target_x = trajectory.x(target_idx);
    target_y = trajectory.y(target_idx);

    % Vector from rear axle to lookahead target
    dx_tgt = target_x - X_rear;
    dy_tgt = target_y - Y_rear;

    % Target angle relative to vehicle heading
    alpha = atan2(dy_tgt, dx_tgt) - psi;
    alpha = wrapToPi(alpha);

    % Actual distance to lookahead target
    Ld_actual = max(0.5, sqrt(dx_tgt^2 + dy_tgt^2));

    % Pure pursuit steering law: delta = atan(2 * L * sin(alpha) / Ld)
    delta_cmd = atan2(2 * vp.L * sin(alpha), Ld_actual);

    % Actuator limits clamp
    delta_cmd = max(vp.delta_min, min(vp.delta_max, delta_cmd));
end
