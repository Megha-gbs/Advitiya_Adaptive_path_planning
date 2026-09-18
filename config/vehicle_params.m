% VEHICLE_PARAMS - Default vehicle dynamic parameters for Indian Autonomous Vehicle
% Problem Statement 26037: SIH - Meghana (Vehicle Dynamics & Control)
% Reference Vehicle: Mid-size Hatchback/Compact SUV (Representative of Indian roads: Tata Nexon / Maruti Brezza / Swift)

function params = vehicle_params()
    params = struct();

    %% Vehicle Dimensions & Inertia
    params.m   = 1250;           % Total vehicle mass (kg)
    params.Iz  = 1850;           % Yaw moment of inertia (kg*m^2)
    params.L   = 2.60;           % Total Wheelbase (m)
    params.Lf  = 1.15;           % Distance from CG to front axle (m)
    params.Lr  = 1.45;           % Distance from CG to rear axle (m) - note: Lf + Lr = L
    params.track_width = 1.50;   % Vehicle track width (m)
    params.width = 1.78;         % Vehicle overall width (m)
    params.length = 4.10;        % Vehicle overall length (m)
    params.wheel_radius = 0.31;  % Dynamic rolling tire radius (m)

    %% Tire & Road Friction Characteristics
    % Linear cornering stiffnesses (per axle, sum of 2 tires)
    params.Cf  = 65000;          % Front cornering stiffness (N/rad)
    params.Cr  = 72000;          % Rear cornering stiffness (N/rad)
    params.mu_nominal = 0.85;    % Nominal dry asphalt tire-road friction coefficient
    params.mu_wet     = 0.50;    % Wet asphalt friction (Monsoon conditions)
    params.mu_gravel  = 0.30;    % Unpaved/mud/gravel shoulder friction (Indian rural/unstructured)
    params.mu_current = params.mu_nominal; % Current road friction

    %% Actuator Constraints & Limits
    % Steering limits
    params.delta_max = deg2rad(32);       % Max front wheel steering angle (+/- 32 deg = ~0.558 rad)
    params.delta_min = -params.delta_max;
    params.delta_rate_max = deg2rad(38);  % Max steering rate (+/- 38 deg/s = ~0.663 rad/s)
    params.tau_steer = 0.08;              % Steering actuator first-order time lag (seconds)

    % Longitudinal acceleration / deceleration limits
    params.a_max = 3.5;          % Maximum acceleration limit (m/s^2) for passenger comfort
    params.a_min = -6.5;         % Maximum regular braking limit (m/s^2)
    params.a_emergency = -8.5;   % Emergency collision avoidance braking limit (m/s^2)
    params.jerk_max = 4.0;       % Maximum longitudinal jerk limit (m/s^3)
    params.tau_throttle = 0.12;  % Throttle powertrain lag (seconds)
    params.tau_brake = 0.06;     % Hydraulic/brake booster response lag (seconds)

    % Velocity operating envelope
    params.v_min = 0.0;          % Minimum forward velocity (m/s)
    params.v_max = 28.0;         % Maximum operating speed (~100 km/h) (m/s)

    %% Environmental Constants
    params.g = 9.81;             % Gravitational acceleration (m/s^2)
    params.air_density = 1.225;  % Air density (kg/m^3)
    params.Cd = 0.34;            % Aerodynamic drag coefficient
    params.frontal_area = 2.2;   % Frontal projected area (m^2)
    params.rolling_res = 0.015;  % Tire rolling resistance coefficient

    %% Normal Loads (Static equilibrium)
    params.Fz_f_static = (params.m * params.g * params.Lr) / params.L;
    params.Fz_r_static = (params.m * params.g * params.Lf) / params.L;
end
