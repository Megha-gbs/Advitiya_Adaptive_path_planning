%% localization_manager.m — Unified Localization Interface
% Hanish Localization Subsystem
% Status: Integration layer — combines EKF + GPS-denied fallback + occupancy map
%
% PURPOSE:
%   Single entry point for the planner (Yashwanth). Manages mode switching
%   between GNSS+IMU fusion (primary) and a GPS-denied fallback mode
%   (labeled 'LiDAR-odometry'). Publishes a unified pose + covariance +
%   mode + confidence.
%
% IMPORTANT — fallback mode is NOT LiDAR-corrected localization:
%   The 'LiDAR-odometry' mode currently performs IMU dead reckoning only
%   (see section 4, "Mode-specific update"). The LiDAR `scan` input is
%   used exclusively for LiDAR-assisted occupancy-map building (section
%   5); it does not correct the pose estimate in any mode. Do not report
%   this as "LiDAR correcting the pose" -- it is IMU dead reckoning with
%   LiDAR-assisted mapping running alongside it.
%
% USAGE:
%   Call this function once per timestep with new sensor data.
%   See test_localization_manager.m for a complete driver example.
%
% NOTE: Simulation-only. See report for limitations.

function [pose, map] = localization_manager(t, imu, gps, scan)

    %% Persistent state (survives across calls)
    persistent ekf_state P_est last_gps_time mode_current
    persistent map_obj stepSize
    persistent gnss_lost_count gnss_recovered_count
    persistent initialised

    %% Configuration constants
    dt               = 0.01;      % 100 Hz IMU
    GPS_TIMEOUT      = 1.0;       % seconds — treat GPS as lost after this
    GPS_RECOVERY_MIN = 3;         % consecutive good fixes to re-acquire
    mapWidth         = 20;
    mapHeight        = 20;
    resolution       = 0.1;
    maxRange         = 10;

    %% 1. Initialisation (first call)
    if isempty(initialised) || ~initialised
        if ~isempty(gps)
            x0 = [gps(1); gps(2); 0; 0];
        else
            x0 = [0; 0; 0; 0];
        end

        ekf_state       = x0;
        P_est           = diag([1.5^2, 1.5^2, 100, 100]);
        last_gps_time   = t;
        mode_current    = 'GNSS+IMU';

        map_obj         = occupancyMap(mapWidth, mapHeight, 1/resolution);
        map_obj.GridLocationInWorld = [0 0];
        stepSize        = resolution;

        gnss_lost_count      = 0;
        gnss_recovered_count = 0;
        initialised          = true;
    end

    %% 2. Determine mode
    % NOTE: GPS arrives slower than the 100 Hz IMU loop, so `gps` is only
    % non-empty on the ticks a fix actually lands. Mode selection must be
    % based on RECENCY of the last fix (last_gps_time), not on whether a
    % fix happens to exist on this exact tick -- otherwise the mode flaps
    % between RECOVERY/GNSS+IMU and LiDAR-odometry every tick that a fix
    % simply hasn't arrived yet, even with continuous GPS coverage.
    if ~isempty(gps)
        last_gps_time        = t;
        gnss_recovered_count = gnss_recovered_count + 1;
    end

    gps_available = (t - last_gps_time) < GPS_TIMEOUT;

    if gps_available
        gnss_lost_count = 0;

        if gnss_recovered_count >= GPS_RECOVERY_MIN
            mode_current = 'GNSS+IMU';
        else
            mode_current = 'RECOVERY';
        end
    else
        gnss_lost_count      = gnss_lost_count + 1;
        gnss_recovered_count = 0;
        mode_current         = 'LiDAR-odometry';
    end

    %% 3. State prediction (always, using IMU)
    F = [1 0 dt 0;
         0 1 0 dt;
         0 0 1  0;
         0 0 0  1];

    B = [0.5*dt^2 0;
         0 0.5*dt^2;
         dt 0;
         0 dt];

    sigma_a = 0.05;
    Q = sigma_a^2 * [dt^4/4, 0,       dt^3/2, 0;
                     0,      dt^4/4,  0,      dt^3/2;
                     dt^3/2, 0,       dt^2,   0;
                     0,      dt^3/2,  0,      dt^2];

    x_pred = F * ekf_state + B * imu(1:2)';
    P_pred = F * P_est * F' + Q;

    %% 4. Mode-specific update
    if strcmp(mode_current, 'GNSS+IMU') || strcmp(mode_current, 'RECOVERY')
        if ~isempty(gps)
            H = [1 0 0 0; 0 1 0 0];
            R = diag([1.5^2, 1.5^2]);

            z = gps(1:2)';
            y = z - H * x_pred;
            S = H * P_pred * H' + R;
            K = P_pred * H' / S;

            ekf_state = x_pred + K * y;
            P_est     = (eye(4) - K * H) * P_pred;
        else
            ekf_state = x_pred;
            P_est     = P_pred;
        end
    else
        % LiDAR-odometry fallback (stub — uses IMU dead reckoning)
        ekf_state = x_pred;
        P_est     = P_pred;
    end

    %% 5. Occupancy map update (if LiDAR scan available)
    if ~isempty(scan) && ~isempty(scan.Ranges)
        px  = ekf_state(1);
        py  = ekf_state(2);
        yaw = atan2(ekf_state(4), ekf_state(3));

        angles = scan.Angles;
        ranges = scan.Ranges;

        for b = 1:numel(ranges)
            r = ranges(b);
            if r <= 0 || ~isfinite(r) || r > maxRange
                continue;
            end

            rayAngle = yaw + angles(b);
            numSteps = floor(r / stepSize);

            for i = 1:numSteps
                cx = px + i*stepSize*cos(rayAngle);
                cy = py + i*stepSize*sin(rayAngle);
                col = round(cx * map_obj.Resolution) + 1;
                row = round(cy * map_obj.Resolution) + 1;

                if col < 1 || col > map_obj.GridSize(2) || ...
                   row < 1 || row > map_obj.GridSize(1)
                    break;
                end

                occ = map_obj.getOccupancy([cx, cy]);
                map_obj.setOccupancy([cx, cy], max(0, occ - 0.05));
            end

            ex = px + r*cos(rayAngle);
            ey = py + r*sin(rayAngle);
            col = round(ex * map_obj.Resolution) + 1;
            row = round(ey * map_obj.Resolution) + 1;

            if col >= 1 && col <= map_obj.GridSize(2) && ...
               row >= 1 && row <= map_obj.GridSize(1)
                occ = map_obj.getOccupancy([ex, ey]);
                map_obj.setOccupancy([ex, ey], min(1, occ + 0.4));
            end
        end
    end

    %% 6. Package output
    pose = struct();
    pose.timestamp   = t;
    pose.position    = ekf_state(1:2);
    pose.velocity    = ekf_state(3:4);
    % NOTE: pose.yaw is COURSE / velocity-direction heading (direction of
    % travel derived from the estimated velocity vector), not an
    % independently estimated vehicle attitude from a gyro/IMU orientation
    % filter. Appropriate for this 2D kinematic prototype; do not describe
    % it as IMU gyro attitude estimation.
    pose.yaw         = atan2(ekf_state(4), ekf_state(3));
    pose.cov_pos     = P_est(1:2, 1:2);
    pose.cov_vel     = P_est(3:4, 3:4);
    pose.mode        = mode_current;

    trace_pos = trace(P_est(1:2, 1:2));
    pose.confidence = 1 / (1 + trace_pos);

    map = map_obj;
end