% VEHICLE_CONTROL_EXECUTIVE - Meghana's Master Controller Module
% Integrates Trajectory, EgoState, and SafetyStatus to publish ControlCommand and VehicleFeedback.
% Supports dynamic switching between MPC (Advanced), Stanley, and Pure Pursuit (Baselines).
%
% Inputs:
%   trajectory     : Struct [time, x, y, yaw, velocity, acceleration, curvature]
%   egoState       : Struct [timestamp, pose, velocity, heading, covariance]
%   safetyStatus   : Struct [safe, TTC, clearance, reason, emergency_flag]
%   control_mode   : 'mpc' | 'stanley' | 'pure_pursuit'
%   internal_state : Persistent struct holding integrator, actuator, and previous inputs
%   vp             : Vehicle parameters
%   cp             : Controller parameters
%
% Outputs:
%   controlCommand : Struct [timestamp, steering, throttle, brake, accel_cmd]
%   vehicleFeedback: Struct [timestamp, actual_pose, actual_velocity, cross_track_err, ...]
%   internal_state : Updated internal state for next cycle
%
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [controlCommand, vehicleFeedback, internal_state] = ...
    vehicle_control_executive(trajectory, egoState, safetyStatus, control_mode, internal_state, vp, cp)

    % Default state initialization if empty
    if isempty(internal_state)
        internal_state = struct();
        internal_state.u_prev = [0.0; 0.0];      % [delta, a]
        internal_state.actuator_state = struct('delta', 0.0, 'a', 0.0);
        internal_state.int_vel_err = 0.0;
        internal_state.last_time = egoState.timestamp;
    end

    dt = egoState.timestamp - internal_state.last_time;
    if dt <= 0 || dt > 0.2
        dt = 0.05; % Fallback nominal sample step
    end
    internal_state.last_time = egoState.timestamp;

    % Current ego vehicle state
    ego_x = egoState.pose(1);
    ego_y = egoState.pose(2);
    ego_yaw = egoState.heading;
    
    if length(egoState.velocity) >= 2
        ego_v = norm(egoState.velocity(1:2));
    else
        ego_v = egoState.velocity(1);
    end

    ego_pose = [ego_x, ego_y, ego_yaw];

    % Find closest path point for velocity and reference lookup
    dx_all = trajectory.x - ego_x;
    dy_all = trajectory.y - ego_y;
    [~, closest_idx] = min(dx_all.^2 + dy_all.^2);

    v_ref = trajectory.velocity(closest_idx);
    a_ref = trajectory.acceleration(closest_idx);
    kappa_ref = trajectory.curvature(closest_idx);
    yaw_ref = trajectory.yaw(closest_idx);

    % Compute tracking errors
    dx_c = ego_x - trajectory.x(closest_idx);
    dy_c = ego_y - trajectory.y(closest_idx);
    e_y = -sin(yaw_ref) * dx_c + cos(yaw_ref) * dy_c;
    e_psi = wrapToPi(ego_yaw - yaw_ref);
    e_v = ego_v - v_ref;

    %% Lateral & Longitudinal Control Law Execution
    switch lower(control_mode)
        case 'mpc'
            % Simultaneous lateral and longitudinal optimization
            [u_opt, ey_mpc, epsi_mpc, ev_mpc, ~] = mpc_controller(ego_pose, ego_v, ...
                internal_state.u_prev, trajectory, cp, vp);
            
            raw_delta = u_opt(1);
            raw_accel = u_opt(2);

            % Convert raw acceleration to throttle / brake
            if raw_accel >= 0
                raw_throttle = min(1.0, raw_accel / vp.a_max);
                raw_brake    = 0.0;
            else
                raw_throttle = 0.0;
                raw_brake    = min(1.0, abs(raw_accel) / abs(vp.a_min));
            end

        case 'stanley'
            % Stanley front-axle lateral tracking
            [raw_delta, e_fa, theta_e, ~] = stanley_controller(ego_pose, ego_v, trajectory, cp, vp);
            % Coupled with Longitudinal PID
            [raw_accel, raw_throttle, raw_brake, internal_state.int_vel_err] = ...
                longitudinal_controller(ego_v, v_ref, a_ref, internal_state.int_vel_err, dt, cp, vp);

        case 'pure_pursuit'
            % Pure Pursuit geometric lateral tracking
            [raw_delta, ~, ~] = pure_pursuit_controller(ego_pose, ego_v, trajectory, cp, vp);
            % Coupled with Longitudinal PID
            [raw_accel, raw_throttle, raw_brake, internal_state.int_vel_err] = ...
                longitudinal_controller(ego_v, v_ref, a_ref, internal_state.int_vel_err, dt, cp, vp);

        otherwise
            error('Unknown control_mode: %s. Use mpc, stanley, or pure_pursuit.', control_mode);
    end

    %% Actuator Dynamics Filtering (Physical Lag & Slew Rates)
    [filtered_delta, filtered_accel, internal_state.actuator_state] = ...
        actuator_model(raw_delta, raw_accel, internal_state.actuator_state, vp, dt);

    if filtered_accel >= 0
        filtered_throttle = min(1.0, filtered_accel / vp.a_max);
        filtered_brake    = 0.0;
    else
        filtered_throttle = 0.0;
        filtered_brake    = min(1.0, abs(filtered_accel) / abs(vp.a_min));
    end

    %% Independent Deterministic Safety Supervisor Check
    slip_f = filtered_delta - atan2(ego_v * sin(e_psi) + vp.Lf * 0, max(0.5, ego_v * cos(e_psi)));
    slip_r = -atan2(ego_v * sin(e_psi) - vp.Lr * 0, max(0.5, ego_v * cos(e_psi)));

    [is_override, safe_delta, safe_throttle, safe_brake, override_reason] = ...
        safety_override(filtered_delta, filtered_throttle, filtered_brake, ...
                        safetyStatus, e_y, slip_f, slip_r, vp, cp);

    if is_override
        final_delta = safe_delta;
        final_throttle = safe_throttle;
        final_brake = safe_brake;
        if final_brake > 0
            final_accel = -final_brake * abs(vp.a_emergency);
        else
            final_accel = final_throttle * vp.a_max;
        end
    else
        final_delta = filtered_delta;
        final_throttle = filtered_throttle;
        final_brake = filtered_brake;
        final_accel = filtered_accel;
    end

    % Update previous commands
    internal_state.u_prev = [final_delta; final_accel];

    %% Pack ControlCommand (Output for Simulator / Sireesha)
    controlCommand = create_control_command(egoState.timestamp, final_delta, ...
                                            final_throttle, final_brake, final_accel);

    %% Dynamic Feasibility Analysis for Planning Feedback (Output for Yashwanth)
    % Required lateral acceleration: ay = v^2 * kappa
    ay_req = ego_v^2 * abs(kappa_ref);
    mu_eff = vp.mu_current;
    ay_max_available = mu_eff * vp.g;

    feasibility_code = 0; % 0: OK
    msg = 'TRAJECTORY_FEASIBLE';

    if is_override
        feasibility_code = 4; % SAFETY_OVERRIDE_ACTIVE
        msg = override_reason;
    elseif ay_req > 0.85 * ay_max_available
        feasibility_code = 1; % CURVATURE_EXCEEDS_TRACTION
        msg = sprintf('TRACTION_LIMIT_WARNING: required ay=%.2fm/s^2 approaches limit=%.2fm/s^2 (mu=%.2f)', ...
                      ay_req, ay_max_available, mu_eff);
    elseif abs(final_delta) >= 0.95 * vp.delta_max
        feasibility_code = 3; % STEERING_SATURATION
        msg = 'STEERING_ACTUATOR_SATURATED';
    elseif abs(e_y) > 0.8 * cp.safety.max_allowable_ey
        feasibility_code = 2; % HIGH_TRACKING_ERROR
        msg = sprintf('HIGH_TRACKING_ERROR: ey=%.2fm', e_y);
    end

    vehicleFeedback = create_vehicle_feedback(egoState.timestamp, ego_x, ego_y, ego_yaw, ego_v, ...
                                             e_y, e_psi, e_v, ay_req, slip_f, slip_r, ...
                                             feasibility_code, msg);
end
