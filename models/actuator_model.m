% ACTUATOR_MODEL - Simulates realistic actuator lag, rate limits and physical saturation
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [actual_delta, actual_a, state] = actuator_model(cmd_delta, cmd_a, state, params, dt)
    % state contains current [delta, a]
    % Rate limit calculation
    d_delta = (cmd_delta - state.delta);
    max_d_delta = params.delta_rate_max * dt;
    d_delta_clamped = max(-max_d_delta, min(max_d_delta, d_delta));
    
    % Steering 1st order lag filter
    alpha_steer = dt / (params.tau_steer + dt);
    target_delta = state.delta + d_delta_clamped;
    state.delta = state.delta + alpha_steer * (target_delta - state.delta);
    
    % Clamp physical limits
    state.delta = max(params.delta_min, min(params.delta_max, state.delta));
    actual_delta = state.delta;

    % Longitudinal acceleration lag and slew rate
    max_d_a = params.jerk_max * dt;
    d_a = (cmd_a - state.a);
    d_a_clamped = max(-max_d_a, min(max_d_a, d_a));
    
    if cmd_a >= 0
        tau_long = params.tau_throttle;
        a_bound_max = params.a_max;
        a_bound_min = params.a_min;
    else
        tau_long = params.tau_brake;
        a_bound_max = params.a_max;
        a_bound_min = params.a_emergency;
    end
    
    alpha_long = dt / (tau_long + dt);
    target_a = state.a + d_a_clamped;
    state.a = state.a + alpha_long * (target_a - state.a);
    state.a = max(a_bound_min, min(a_bound_max, state.a));
    actual_a = state.a;
end
