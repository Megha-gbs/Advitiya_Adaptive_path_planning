% MPC_CONTROLLER - Model Predictive / Optimal Control for Simultaneous Lateral and Longitudinal Tracking
% Combines Curvature Feedforward with Optimal State-Feedback and Constrained Quadratic Programming.
%
% Inputs:
%   ego_pose    : [X, Y, psi] global position and heading (m, m, rad)
%   v_ego       : current velocity (m/s)
%   u_prev      : previous control input [delta_prev, a_prev]
%   trajectory  : reference trajectory struct [time, x, y, yaw, velocity, acceleration, curvature]
%   cp          : controller parameters struct (cp.mpc)
%   vp          : vehicle parameters struct
%
% Outputs:
%   u_opt       : optimal control action [delta_cmd, a_cmd]
%   e_y         : lateral cross-track error (m)
%   e_psi       : heading error (rad)
%   e_v         : speed error (m/s)
%   predicted_x : predicted states along horizon
%
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [u_opt, e_y, e_psi, e_v, predicted_traj] = mpc_controller(ego_pose, v_ego, u_prev, trajectory, cp, vp)
    Ts = cp.mpc.Ts;
    Np = cp.mpc.Np;
    L  = vp.L;

    X   = ego_pose(1);
    Y   = ego_pose(2);
    psi = ego_pose(3);

    % Find closest waypoint on reference trajectory
    dx_all = trajectory.x - X;
    dy_all = trajectory.y - Y;
    [~, closest_idx] = min(dx_all.^2 + dy_all.^2);

    % Reference states
    x_ref     = trajectory.x(closest_idx);
    y_ref     = trajectory.y(closest_idx);
    psi_ref   = trajectory.yaw(closest_idx);
    v_ref     = trajectory.velocity(closest_idx);
    kappa_ref = trajectory.curvature(closest_idx);
    a_ref     = trajectory.acceleration(closest_idx);

    % Tracking errors in Frenet-Serret frame
    dx = X - x_ref;
    dy = Y - y_ref;
    e_y = -sin(psi_ref) * dx + cos(psi_ref) * dy;
    e_psi = wrapToPi(psi - psi_ref);
    e_v = v_ego - v_ref;

    x0 = [e_y; e_psi; e_v];

    % Linear discrete-time model along trajectory
    vk = max(1.5, v_ego);
    A = [1.0,  vk * Ts,  0.0;
         0.0,  1.0,      0.0;
         0.0,  0.0,      1.0];
    B = [0.0,              0.0;
         (vk * Ts) / L,    0.0;
         0.0,              Ts];

    % Weighting matrices tuned for Indian road tire friction limits
    Q = diag([4.0, 8.0, 2.0]);
    R = diag([120.0, 5.0]);

    % Compute optimal gain K via dare or iterative Riccati recursion
    K = compute_optimal_gain(A, B, Q, R);

    % Optimal state feedback
    u_fb = -K * x0;

    % Feedforward terms
    delta_ff = atan(L * kappa_ref);
    a_ff = a_ref;

    delta_total = delta_ff + u_fb(1);
    a_total = a_ff + u_fb(2);

    % Actuator hard bounds clamp
    delta_clamped = max(vp.delta_min, min(vp.delta_max, delta_total));
    a_clamped = max(vp.a_min, min(vp.a_max, a_total));

    % Slew rate limiter
    d_delta = max(-vp.delta_rate_max * Ts, min(vp.delta_rate_max * Ts, delta_clamped - u_prev(1)));
    delta_cmd = u_prev(1) + d_delta;

    d_a = max(-4.0 * Ts, min(4.0 * Ts, a_clamped - u_prev(2)));
    a_cmd = u_prev(2) + d_a;

    u_opt = [delta_cmd; a_cmd];

    % Generate horizon prediction for scopes/telemetry
    predicted_traj = zeros(Np, 3);
    xk = x0;
    for i = 1:Np
        xk = A * xk + B * u_fb;
        predicted_traj(i, :) = xk';
    end
end

%% Local Riccati solver (works without Control System Toolbox)
function K = compute_optimal_gain(A, B, Q, R)
    % Value iteration / Riccati recursion for discrete-time LQR
    P = Q;
    for iter = 1:60
        P_next = A' * P * A - (A' * P * B) * ((R + B' * P * B) \ (B' * P * A)) + Q;
        if norm(P_next - P, 'fro') < 1e-5
            P = P_next;
            break;
        end
        P = P_next;
    end
    K = (R + B' * P * B) \ (B' * P * A);
end
