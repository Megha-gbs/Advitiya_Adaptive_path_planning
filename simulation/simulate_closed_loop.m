% SIMULATE_CLOSED_LOOP - Closed-loop simulation engine for autonomous path tracking
% Runs plant simulation (Kinematic or Dynamic Bicycle) driven by vehicle_control_executive.
%
% Inputs:
%   trajectory     : Trajectory struct
%   control_mode   : 'mpc' | 'stanley' | 'pure_pursuit'
%   plant_type     : 'dynamic' | 'kinematic'
%   mu_road        : Road surface friction coefficient (0.85 dry, 0.5 wet, 0.3 gravel)
%   initial_offset : [dx, dy, dpsi, dv] initial disturbance from path start
%
% Output:
%   sim_results    : Struct with comprehensive logged telemetry and metrics
%
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function sim_results = simulate_closed_loop(trajectory, control_mode, plant_type, mu_road, initial_offset)
    if nargin < 3 || isempty(plant_type), plant_type = 'dynamic'; end
    if nargin < 4 || isempty(mu_road), mu_road = 0.85; end
    if nargin < 5 || isempty(initial_offset), initial_offset = [0.0, 0.0, 0.0, 0.0]; end

    vp = vehicle_params();
    vp.mu_current = mu_road;
    cp = controller_params();

    dt = 0.05; % Simulation time step (20 Hz)
    t_end = trajectory.time(end);
    time_steps = 0:dt:t_end;
    N_steps = length(time_steps);

    % Initialize vehicle state at trajectory start + initial offset
    init_x   = trajectory.x(1) + initial_offset(1);
    init_y   = trajectory.y(1) + initial_offset(2);
    init_psi = trajectory.yaw(1) + initial_offset(3);
    init_v   = trajectory.velocity(1) + initial_offset(4);

    if strcmpi(plant_type, 'dynamic')
        % [X, Y, psi, vx, vy, r]
        plant_state = [init_x; init_y; init_psi; init_v; 0.0; 0.0];
    else
        % [X, Y, psi, v]
        plant_state = [init_x; init_y; init_psi; init_v];
    end

    % Pre-allocate logs
    log_time        = zeros(1, N_steps);
    log_x           = zeros(1, N_steps);
    log_y           = zeros(1, N_steps);
    log_yaw         = zeros(1, N_steps);
    log_v           = zeros(1, N_steps);
    log_ref_x       = zeros(1, N_steps);
    log_ref_y       = zeros(1, N_steps);
    log_ref_v       = zeros(1, N_steps);
    log_ey          = zeros(1, N_steps);
    log_epsi        = zeros(1, N_steps);
    log_ev          = zeros(1, N_steps);
    log_delta       = zeros(1, N_steps);
    log_throttle    = zeros(1, N_steps);
    log_brake       = zeros(1, N_steps);
    log_lat_accel   = zeros(1, N_steps);
    log_slip_f      = zeros(1, N_steps);
    log_slip_r      = zeros(1, N_steps);
    log_feasibility = zeros(1, N_steps);

    internal_state = [];
    u_plant = [0.0; 0.0]; % [delta, a]

    % Default nominal safety status
    safetyStatus = struct();
    safetyStatus.safe = 'safe';
    safetyStatus.TTC = 99.0;
    safetyStatus.clearance = 50.0;
    safetyStatus.reason = 'CLEAR_PATH';
    safetyStatus.emergency_flag = false;

    % Execution loop
    for k = 1:N_steps
        t = time_steps(k);

        % Dynamic emergency injection if configured in scenario
        if isfield(trajectory, 'emergency_time') && t >= trajectory.emergency_time
            safetyStatus.emergency_flag = true;
            safetyStatus.safe = 'reject';
            safetyStatus.TTC = 0.8;
            safetyStatus.clearance = 4.5;
            safetyStatus.reason = trajectory.emergency_reason;
        end

        % Extract current ego state representation for Hanish interface
        ego_x = plant_state(1);
        ego_y = plant_state(2);
        ego_yaw = plant_state(3);
        if strcmpi(plant_type, 'dynamic')
            ego_vx = plant_state(4);
            ego_vy = plant_state(5);
            ego_v = sqrt(ego_vx^2 + ego_vy^2);
        else
            ego_v = plant_state(4);
            ego_vx = ego_v;
            ego_vy = 0.0;
        end

        egoState = struct();
        egoState.timestamp  = t;
        egoState.pose       = [ego_x, ego_y, 0.0];
        egoState.velocity   = [ego_vx, ego_vy, 0.0];
        egoState.heading    = ego_yaw;
        egoState.covariance = diag([0.02, 0.02, 0.001]);

        % Execute Meghana's Master Controller Executive
        [controlCommand, vehicleFeedback, internal_state] = ...
            vehicle_control_executive(trajectory, egoState, safetyStatus, control_mode, internal_state, vp, cp);

        % Forward control command into plant simulation
        u_plant = [controlCommand.steering; controlCommand.accel_cmd];

        % Step plant model
        if strcmpi(plant_type, 'dynamic')
            [plant_state, diag_dyn] = dynamic_bicycle(plant_state, u_plant, vp, dt, mu_road);
            lat_accel = diag_dyn.ay_total;
            slip_f = diag_dyn.alpha_f;
            slip_r = diag_dyn.alpha_r;
        else
            [plant_state, ~] = kinematic_bicycle(plant_state, u_plant, vp, dt);
            lat_accel = ego_v^2 * (tan(controlCommand.steering) / vp.L);
            slip_f = 0.0;
            slip_r = 0.0;
        end

        % Reference lookup for logging
        dx_all = trajectory.x - ego_x;
        dy_all = trajectory.y - ego_y;
        [~, c_idx] = min(dx_all.^2 + dy_all.^2);

        % Record logging arrays
        log_time(k)        = t;
        log_x(k)           = ego_x;
        log_y(k)           = ego_y;
        log_yaw(k)         = ego_yaw;
        log_v(k)           = ego_v;
        log_ref_x(k)       = trajectory.x(c_idx);
        log_ref_y(k)       = trajectory.y(c_idx);
        log_ref_v(k)       = trajectory.velocity(c_idx);
        log_ey(k)          = vehicleFeedback.cross_track_err;
        log_epsi(k)        = vehicleFeedback.heading_err;
        log_ev(k)          = vehicleFeedback.speed_err;
        log_delta(k)       = controlCommand.steering;
        log_throttle(k)    = controlCommand.throttle;
        log_brake(k)       = controlCommand.brake;
        log_lat_accel(k)   = lat_accel;
        log_slip_f(k)      = slip_f;
        log_slip_r(k)      = slip_r;
        log_feasibility(k) = vehicleFeedback.feasibility_code;
    end

    % Package results & calculate quantitative performance figures of merit
    sim_results = struct();
    sim_results.scenario_name = trajectory.name;
    sim_results.control_mode  = control_mode;
    sim_results.plant_type    = plant_type;
    sim_results.mu_road       = mu_road;
    
    sim_results.time        = log_time;
    sim_results.x           = log_x;
    sim_results.y           = log_y;
    sim_results.yaw         = log_yaw;
    sim_results.v           = log_v;
    sim_results.ref_x       = log_ref_x;
    sim_results.ref_y       = log_ref_y;
    sim_results.ref_v       = log_ref_v;
    sim_results.ey          = log_ey;
    sim_results.epsi        = log_epsi;
    sim_results.ev          = log_ev;
    sim_results.delta       = log_delta;
    sim_results.throttle    = log_throttle;
    sim_results.brake       = log_brake;
    sim_results.lat_accel   = log_lat_accel;
    sim_results.slip_f      = log_slip_f;
    sim_results.slip_r      = log_slip_r;
    sim_results.feasibility = log_feasibility;

    % Performance Metrics
    sim_results.metrics = struct();
    sim_results.metrics.max_abs_ey     = max(abs(log_ey));
    sim_results.metrics.rmse_ey         = sqrt(mean(log_ey.^2));
    sim_results.metrics.max_abs_epsi   = rad2deg(max(abs(log_epsi)));
    sim_results.metrics.rmse_epsi       = rad2deg(sqrt(mean(log_epsi.^2)));
    sim_results.metrics.max_abs_ev     = max(abs(log_ev));
    sim_results.metrics.rmse_ev         = sqrt(mean(log_ev.^2));
    sim_results.metrics.max_lat_accel  = max(abs(log_lat_accel));
    sim_results.metrics.max_slip_angle = rad2deg(max(max(abs(log_slip_f)), max(abs(log_slip_r))));
    sim_results.metrics.control_effort = trapz(log_time, log_delta.^2 + log_throttle.^2 + log_brake.^2);
end
