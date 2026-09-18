% CONTROLLER_PARAMS - Configuration and tuning gains for tracking controllers
% Problem Statement 26037: SIH - Meghana (Vehicle Dynamics & Control)

function cp = controller_params()
    cp = struct();

    %% Pure Pursuit Parameters
    cp.pp = struct();
    cp.pp.k_dd = 0.55;           % Speed-proportional lookahead gain (s) -> Ld = k_dd * v + Ld_min
    cp.pp.Ld_min = 2.5;          % Minimum lookahead distance (m) (for low speed / parking / sharp turns)
    cp.pp.Ld_max = 25.0;         % Maximum lookahead distance (m) (prevents cutting corners at high speed)

    %% Stanley Controller Parameters
    cp.stanley = struct();
    cp.stanley.k_e = 1.25;       % Cross-track error gain (1/s)
    cp.stanley.k_soft = 1.5;     % Softening parameter (m/s) prevents singularity at near-zero velocity
    cp.stanley.k_yaw_rate = 0.12;% Dynamic damping yaw-rate compensation gain
    cp.stanley.k_d = 0.05;       % Cross-track rate derivative damping gain

    %% Longitudinal Controller (PID + Feedforward)
    cp.long = struct();
    cp.long.Kp = 1.20;           % Proportional gain on velocity error
    cp.long.Ki = 0.15;           % Integral gain on velocity error
    cp.long.Kd = 0.04;           % Derivative gain on acceleration error
    cp.long.anti_windup_limit = 2.0; % Integrator saturation clamping limit
    cp.long.feedforward_gain = 1.0;  % Direct feedforward acceleration gain

    %% Model Predictive Control (MPC) Parameters
    cp.mpc = struct();
    cp.mpc.Ts = 0.05;            % MPC sampling time (50 ms / 20 Hz control rate)
    cp.mpc.Np = 20;              % Prediction horizon steps (20 * 0.05s = 1.0 second lookahead)
    cp.mpc.Nc = 10;              % Control horizon steps
    
    % State weighting matrices Q (diag: [ey, epsi, ev])
    % States: e_y (lateral tracking error [m]), e_psi (heading error [rad]), e_v (velocity error [m/s])
    cp.mpc.Q = diag([25.0, 40.0, 8.0]);
    cp.mpc.Q_terminal = diag([45.0, 70.0, 15.0]); % Terminal weight for stability
    
    % Control input weights R (diag: [delta_rate, accel_rate])
    % Inputs: u = [delta (steering [rad]), a (acceleration [m/s^2])]
    cp.mpc.R = diag([15.0, 2.5]);
    % Rate of change of control inputs weight R_rate
    cp.mpc.R_rate = diag([30.0, 6.0]);

    %% Safety & Emergency Thresholds
    cp.safety = struct();
    cp.safety.emergency_decel = -8.5;  % Maximum deceleration under emergency stop (m/s^2)
    cp.safety.max_allowable_ey = 1.2;  % Feasibility breach warning threshold (m)
    cp.safety.max_allowable_epsi = deg2rad(25); % Maximum heading deviation before re-planning alarm
    cp.safety.slip_warning_threshold = deg2rad(7); % Tire slip angle warning limit
end
