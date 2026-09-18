% SAFETY_OVERRIDE - Independent deterministic safety arbitration module
% SIH Problem Statement 26037: Vehicle Dynamics & Control
% Evaluates safety flags and dynamic tracking stability independently of planner

function [is_override, delta_safe, throttle_safe, brake_safe, override_reason] = ...
    safety_override(cmd_delta, cmd_throttle, cmd_brake, safetyStatus, ey, slip_f, slip_r, vp, cp)

    is_override = false;
    delta_safe = cmd_delta;
    throttle_safe = cmd_throttle;
    brake_safe = cmd_brake;
    override_reason = 'NORMAL_OPERATION';

    %% 1. Check External Safety Supervisor Flag (from Safety Module)
    if isfield(safetyStatus, 'emergency_flag') && safetyStatus.emergency_flag == true
        is_override = true;
        delta_safe = 0.0; % Align wheels straight to maximize longitudinal braking traction
        throttle_safe = 0.0;
        brake_safe = 1.0; % Max hydraulic braking
        override_reason = sprintf('EXTERNAL_EMERGENCY_STOP: %s (TTC=%.2fs)', ...
                                  safetyStatus.reason, safetyStatus.TTC);
        return;
    end

    if isfield(safetyStatus, 'safe') && (strcmpi(safetyStatus.safe, 'reject') || ...
                                         strcmpi(safetyStatus.safe, 'unsafe'))
        is_override = true;
        delta_safe = 0.0;
        throttle_safe = 0.0;
        brake_safe = 0.85;
        override_reason = sprintf('PLANNER_TRAJECTORY_REJECTED: %s', safetyStatus.reason);
        return;
    end

    %% 2. Internal Dynamic Stability Monitoring (Loss of Control Prevention)
    % Check for extreme tire slip (spin/skid risk on wet/gravel Indian roads)
    max_slip = max(abs(slip_f), abs(slip_r));
    if max_slip > cp.safety.slip_warning_threshold
        % Reduce throttle and moderate steering to regain tire traction
        is_override = true;
        throttle_safe = 0.0;
        brake_safe = min(1.0, cmd_brake + 0.3); % Gentle stabilization braking
        delta_safe = 0.7 * cmd_delta;          % Counter-steer attenuation
        override_reason = sprintf('TIRE_TRACTION_LOSS_DETECTED: max slip=%.2f deg', rad2deg(max_slip));
        return;
    end

    %% 3. Extreme Cross-Track Deviation Alarm (Off-Road / Pothole edge hazard)
    if abs(ey) > cp.safety.max_allowable_ey
        % Soft warning - keep tracking but limit acceleration
        throttle_safe = min(0.3, cmd_throttle);
        override_reason = sprintf('HIGH_TRACKING_ERROR_WARNING: ey=%.2fm', ey);
    end
end
