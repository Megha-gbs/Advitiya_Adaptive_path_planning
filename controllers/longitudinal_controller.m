% LONGITUDINAL_CONTROLLER - Velocity and Acceleration tracking with throttle/brake split
% Inputs:
%   v_ego       : current vehicle speed (m/s)
%   v_ref       : reference speed from trajectory (m/s)
%   a_ref       : reference acceleration from trajectory (m/s^2)
%   int_err     : integrated velocity error state (m)
%   dt          : time step (s)
%   cp          : controller parameters (cp.long)
%   vp          : vehicle parameters
% Outputs:
%   accel_cmd   : net commanded acceleration (m/s^2)
%   throttle    : normalized throttle command [0.0, 1.0]
%   brake       : normalized brake command [0.0, 1.0]
%   int_err_new : updated integral error state
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [accel_cmd, throttle, brake, int_err_new] = longitudinal_controller(v_ego, v_ref, a_ref, int_err, dt, cp, vp)
    % Velocity error
    e_v = v_ref - v_ego;

    % Numerical integration with anti-windup clamping
    int_err_new = int_err + e_v * dt;
    int_err_new = max(-cp.long.anti_windup_limit, min(cp.long.anti_windup_limit, int_err_new));

    % Feedforward aerodynamic and rolling resistance compensation
    F_aero = 0.5 * vp.air_density * vp.Cd * vp.frontal_area * v_ego^2;
    F_roll = vp.rolling_res * vp.m * vp.g;
    a_resist = (F_aero + F_roll) / vp.m;

    % PID + Feedforward control law
    a_feedback = cp.long.Kp * e_v + cp.long.Ki * int_err_new;
    accel_cmd = cp.long.feedforward_gain * a_ref + a_feedback + a_resist;

    % Clamp acceleration command to vehicle physical comfort and traction limits
    accel_cmd = max(vp.a_min, min(vp.a_max, accel_cmd));

    % Throttle / Brake actuator splitter logic
    if accel_cmd >= 0
        % Positive torque requested -> Throttle active, Brake zero
        throttle = min(1.0, accel_cmd / vp.a_max);
        brake    = 0.0;
    else
        % Negative torque requested -> Throttle zero, Brake active
        throttle = 0.0;
        brake    = min(1.0, abs(accel_cmd) / abs(vp.a_min));
    end
end
