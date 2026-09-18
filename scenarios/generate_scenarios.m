% GENERATE_SCENARIOS - Generates test trajectories matching Yashwanth's Trajectory interface
% SIH Problem Statement 26037: Adaptive Path Planning and Collision Avoidance on Unstructured Indian Roads
%
% Output: trajectory struct:
%   time         : 1xN array of timestamps (s)
%   x            : 1xN global X coordinates (m)
%   y            : 1xN global Y coordinates (m)
%   yaw          : 1xN global heading angles (rad)
%   velocity     : 1xN reference speeds (m/s)
%   acceleration : 1xN reference accelerations (m/s^2)
%   curvature    : 1xN road curvature kappa (1/m)

function scenarios = generate_scenarios()
    scenarios = struct();

    %% Scenario 1: Pothole / Obstacle Quick Dodge (Indian Urban/Rural Road)
    % A 120m stretch where the vehicle swerves 1.5m to the right to dodge a pothole/debris at X=40m
    % and returns to center at X=80m, travelling at ~10 m/s (36 km/h).
    dt = 0.05;
    t_end = 12.0;
    t1 = 0:dt:t_end;
    v1_ref = 10.0; % m/s
    x1 = v1_ref * t1;
    
    % Smooth sigmoid/gaussian-derivative lateral shift
    y1 = zeros(size(x1));
    for i = 1:length(x1)
        xi = x1(i);
        if xi >= 25 && xi <= 55
            % Swerve out by 1.5 m
            y1(i) = 1.5 * 0.5 * (1 - cos(pi * (xi - 25) / 30));
        elseif xi > 55 && xi <= 85
            % Swerve back to original lane
            y1(i) = 1.5 * 0.5 * (1 + cos(pi * (xi - 55) / 30));
        else
            y1(i) = 0.0;
        end
    end
    scenarios.pothole_dodge = compute_trajectory_derivatives(t1, x1, y1, v1_ref * ones(size(t1)));
    scenarios.pothole_dodge.name = 'Pothole / Obstacle Avoidance Swerve';

    %% Scenario 2: Ghat Road Hairpin Mountain Curve
    % High curvature mountain bend (radius = 18m, 180 degree hairpin), speed slows from 12 m/s to 6 m/s
    t2 = 0:dt:18.0;
    N2 = length(t2);
    x2 = zeros(1, N2);
    y2 = zeros(1, N2);
    v2 = zeros(1, N2);
    
    % Straight entry (0 to 30m) -> Hairpin turn -> Straight exit
    R_hairpin = 18.0; % 18m radius
    arc_length = 0;
    for i = 1:N2
        ti = t2(i);
        if ti <= 3.0
            v2(i) = 11.0;
            x2(i) = 11.0 * ti;
            y2(i) = 0.0;
        elseif ti <= 11.0
            % Decelerating and cornering
            tau = (ti - 3.0) / 8.0;
            v2(i) = 11.0 - 5.5 * tau; % Decelerate to 5.5 m/s (~20 km/h)
            angle = tau * pi; % 0 to pi
            x2(i) = 33.0 + R_hairpin * sin(angle);
            y2(i) = R_hairpin * (1 - cos(angle));
        else
            % Straight exit
            t_exit = ti - 11.0;
            v2(i) = 5.5 + min(4.5, 0.8 * t_exit);
            x2(i) = 33.0 - (5.5 * t_exit + 0.5 * 0.8 * t_exit^2);
            y2(i) = 2 * R_hairpin;
        end
    end
    scenarios.ghat_curve = compute_trajectory_derivatives(t2, x2, y2, v2);
    scenarios.ghat_curve.name = 'Ghat Road Hairpin Curve';

    %% Scenario 3: Double Lane Change (ISO 3888-2 Severe Obstacle Avoidance)
    t3 = 0:dt:10.0;
    v3_ref = 14.0; % ~50 km/h
    x3 = v3_ref * t3;
    y3 = zeros(size(x3));
    for i = 1:length(x3)
        xi = x3(i);
        if xi >= 20 && xi <= 45
            y3(i) = 3.5 * 0.5 * (1 - cos(pi * (xi - 20) / 25));
        elseif xi > 45 && xi <= 65
            y3(i) = 3.5;
        elseif xi > 65 && xi <= 90
            y3(i) = 3.5 * 0.5 * (1 + cos(pi * (xi - 65) / 25));
        else
            y3(i) = 0.0;
        end
    end
    scenarios.double_lane_change = compute_trajectory_derivatives(t3, x3, y3, v3_ref * ones(size(t3)));
    scenarios.double_lane_change.name = 'ISO 3888-2 Double Lane Change';

    %% Scenario 4: Straight Cruise with Emergency Brake Trigger (Cow / Jaywalker)
    t4 = 0:dt:8.0;
    v4_ref = 12.0; % ~43 km/h
    x4 = v4_ref * t4;
    y4 = zeros(size(x4));
    scenarios.emergency_stop = compute_trajectory_derivatives(t4, x4, y4, v4_ref * ones(size(t4)));
    scenarios.emergency_stop.name = 'Emergency Braking Collision Avoidance';
    % Special flag for scenario runner: trigger safety emergency at t = 3.5s
    scenarios.emergency_stop.emergency_time = 3.5;
    scenarios.emergency_stop.emergency_reason = 'UNEXPECTED_ANIMAL_CROSSING';
end

%% Helper: Compute geometric yaw, curvature, and acceleration from path
function traj = compute_trajectory_derivatives(t, x, y, v)
    N = length(t);
    dt = mean(diff(t));

    % Numerical derivatives for heading (yaw)
    dx = gradient(x, dt);
    dy = gradient(y, dt);
    yaw = atan2(dy, dx);
    yaw = unwrap(yaw);

    % Second derivatives for curvature: kappa = (dx*d2y - dy*d2x) / (dx^2 + dy^2)^(3/2)
    ddx = gradient(dx, dt);
    ddy = gradient(dy, dt);
    speed_sq = dx.^2 + dy.^2;
    curvature = (dx .* ddy - dy .* ddx) ./ max(1e-4, speed_sq.^(1.5));

    % Acceleration along path
    accel = gradient(v, dt);

    traj = struct();
    traj.time = t;
    traj.x = x;
    traj.y = y;
    traj.yaw = wrapToPi(yaw);
    traj.velocity = v;
    traj.acceleration = accel;
    traj.curvature = curvature;
end
