%% Test driver for localization_manager
% Runs the EKF with a simulated GPS outage in the middle
% Demonstrates mode switching GNSS+IMU → LiDAR-odometry → GNSS+IMU

clear; clc; close all; rng(42);

%% Simulation parameters
dt = 0.01;
T  = 30;
N  = T/dt;
t  = (0:N-1)'*dt;

%% Ground truth (circular path)
R     = 10;
omega = 0.2;
trueX  = R*sin(omega*t);
trueY  = R*cos(omega*t);
trueVx = R*omega*cos(omega*t);
trueVy = -R*omega*sin(omega*t);

%% Sensor simulation
imuNoise = 0.05;
ax_true = zeros(N,1);
ay_true = zeros(N,1);
for k = 2:N
    ax_true(k) = (trueVx(k) - trueVx(k-1))/dt;
    ay_true(k) = (trueVy(k) - trueVy(k-1))/dt;
end

gpsIdx = 1:20:N;
gpsNoise = 1.5;
gpsAvailable = true(N,1);
gpsAvailable(1000:1500) = false;   % GPS outage (t=10 to t=15)

scanInterval = 10;
angles = linspace(-pi, pi, 360)';

%% Preallocate logging arrays
poseHist_pos      = zeros(N, 2);
poseHist_vel      = zeros(N, 2);
poseHist_conf     = zeros(N, 1);
poseHist_mode     = cell(N, 1);
poseHist_yaw      = zeros(N, 1);
poseHist_covTrace = zeros(N, 1);   % trace(cov_pos) -- scalar position-uncertainty proxy
poseHist_covVelTr = zeros(N, 1);   % trace(cov_vel) -- scalar velocity-uncertainty proxy

mapFinal = [];

fprintf('Running localization_manager over %d timesteps...\n', N);

%% Main loop
for k = 1:N
    imu_k = [ax_true(k) + imuNoise*randn, ay_true(k) + imuNoise*randn];

    if gpsAvailable(k) && any(k == gpsIdx)
        gps_k = [trueX(k) + gpsNoise*randn, trueY(k) + gpsNoise*randn];
    else
        gps_k = [];
    end

    if mod(k, scanInterval) == 0
        % NOTE: this is a synthetic placeholder scan (flat ~5 m range +
        % noise), NOT ray-cast against a real obstacle map. It exercises
        % the manager's scan-ingestion/map-update code path only -- it
        % does not validate map geometric accuracy. See lidar_slam_demo.m
        % for ray-cast scans against actual obstacles.
        ranges_k = 5 + 0.5*randn(360,1);
        ranges_k(ranges_k < 0.1) = 0.1;
        scan_k = lidarScan(ranges_k, angles);
    else
        scan_k = [];
    end

    [pose, mapFinal] = localization_manager(t(k), imu_k, gps_k, scan_k);

    poseHist_pos(k,:)    = pose.position;
    poseHist_vel(k,:)    = pose.velocity;
    poseHist_conf(k)     = pose.confidence;
    poseHist_mode{k}     = pose.mode;
    poseHist_yaw(k)      = pose.yaw;
    poseHist_covTrace(k) = trace(pose.cov_pos);
    poseHist_covVelTr(k) = trace(pose.cov_vel);

    if mod(k, 500) == 0
        fprintf('  t=%.1f s, mode=%s, conf=%.3f\n', ...
            t(k), pose.mode, pose.confidence);
    end
end

fprintf('Done.\n');

%% Extract for plotting
posArr  = poseHist_pos;
confArr = poseHist_conf;
modeStr = poseHist_mode;

modeNum = zeros(N,1);
for k = 1:N
    switch modeStr{k}
        case 'GNSS+IMU',       modeNum(k) = 1;
        case 'LiDAR-odometry', modeNum(k) = 2;
        case 'RECOVERY',       modeNum(k) = 3;
    end
end

%% Compute errors
posError = vecnorm(posArr - [trueX, trueY], 2, 2);

% pose.yaw is atan2(vy, vx) (heading = direction of travel), so compare
% against the same quantity computed from ground-truth velocity.
trueYaw  = atan2(trueVy, trueVx);
yawError = abs(wrapToPi(poseHist_yaw - trueYaw));

% Verify cov_vel against actual velocity error (ground truth is known
% for this synthetic circular path), rather than leaving it unverified.
velError = vecnorm(poseHist_vel - [trueVx, trueVy], 2, 2);

%% Plot
figure('Name','Localization Manager Test','Color','w', ...
    'Position',[100 100 1500 700]);

subplot(2,3,1);
plot(trueX, trueY, 'b-', 'LineWidth', 2); hold on;
plot(posArr(:,1), posArr(:,2), 'r--', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
legend('Truth','Manager output','Location','best');
title('Trajectory');

subplot(2,3,2);
plot(t, posError, 'r-', 'LineWidth', 1.5); hold on;
xline(10, 'k--', 'LineWidth', 1.5);
xline(15, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Position error (m)');
title(sprintf('Error (mean = %.3f m)', mean(posError)));
legend('Error','GPS outage','Location','best');

subplot(2,3,3);
plot(t, rad2deg(yawError), 'm-', 'LineWidth', 1.5); hold on;
xline(10, 'k--', 'LineWidth', 1);
xline(15, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)'); ylabel('Yaw error (deg)');
title(sprintf('Yaw error (mean = %.2f deg)', rad2deg(mean(yawError))));

subplot(2,3,4);
plot(t, modeNum, 'k-', 'LineWidth', 1.5);
grid on; ylim([0.5 3.5]);
yticks([1 2 3]);
yticklabels({'GNSS+IMU','LiDAR-odom','Recovery'});
xlabel('Time (s)');
title('Localization mode over time');

subplot(2,3,5);
plot(t, confArr, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Confidence');
title('Planner-facing confidence');

subplot(2,3,6);
plot(t, poseHist_covTrace, 'b-', 'LineWidth', 1.5); hold on;
xline(10, 'k--', 'LineWidth', 1);
xline(15, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)'); ylabel('trace(cov_{pos}) (m^2)');
title('Position-uncertainty proxy (feeds confidence)');

sgtitle('Localization Manager — Mode Switching Under GPS Outage');

%% Map visualization (exercises the `map` output, previously unused)
figure('Name','Localization Manager — Accumulated Occupancy Map','Color','w');
show(mapFinal);
hold on;
plot(posArr(:,1), posArr(:,2), 'r-', 'LineWidth', 1);
title({'Occupancy map built from estimated poses (all modes)', ...
    '(scan content is synthetic placeholder data -- map geometry not validated)'});

%% Summary
fprintf('\n=== Localization Manager Test ===\n');
fprintf('Mean position error:   %.3f m\n', mean(posError));
fprintf('Max position error:    %.3f m\n', max(posError));
fprintf('Mean yaw error:        %.4f rad (%.2f deg)\n', ...
    mean(yawError), rad2deg(mean(yawError)));
fprintf('Error during GPS outage (t=10-15):  %.3f m\n', ...
    mean(posError(t >= 10 & t <= 15)));
fprintf('Error outside outage:                %.3f m\n', ...
    mean(posError(t < 10 | t > 15)));
fprintf('GPS outage:            t = 10.0 to 15.0 s\n');
fprintf('Mean position-cov trace (uncertainty proxy): %.4f m^2\n', ...
    mean(poseHist_covTrace));
fprintf('Mean velocity error:    %.3f m/s\n', mean(velError));
fprintf('Mean velocity-cov trace (uncertainty proxy): %.4f (m/s)^2\n', ...
    mean(poseHist_covVelTr));
fprintf('Final occupancy map:   %d x %d cells\n', ...
    mapFinal.GridSize(1), mapFinal.GridSize(2));
fprintf('Mode switches observed:\n');
prevMode = modeStr{1};
fprintf('  t=%.2f → %s\n', t(1), prevMode);
for k = 2:N
    if ~strcmp(modeStr{k}, prevMode)
        fprintf('  t=%.2f → %s\n', t(k), modeStr{k});
        prevMode = modeStr{k};
    end
end

fprintf('\nDone. Check figure windows.\n');
fprintf(['Ready for planning handoff: pose.{position,velocity,yaw,' ...
    'cov_pos,cov_vel,mode,confidence}, map.\n']);