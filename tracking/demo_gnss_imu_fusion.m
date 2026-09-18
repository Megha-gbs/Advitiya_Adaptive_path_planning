%% Demo: GNSS + IMU Fusion with insEKF
% Hanish - Localization Subsystem
% Demonstrates the core localization concept for the AV project

clear; clc; close all;
rng(42);  % Reproducible

fprintf('=== GNSS + IMU Fusion Demo ===\n\n');

%% 1. Simulation parameters
fs_imu = 100;      % IMU sample rate (Hz)
fs_gps = 1;        % GPS sample rate (Hz)
T = 20;            % Duration (seconds)

t_imu = (0:1/fs_imu:T-1/fs_imu)';
N_imu = numel(t_imu);

%% 2. Ground truth trajectory (circle with slight climb)
omega = 0.2;  % rad/s
R = 10;       % circle radius (m)
truePos = [R*sin(omega*t_imu), R*cos(omega*t_imu), 0.5*t_imu];
trueVel = [R*omega*cos(omega*t_imu), -R*omega*sin(omega*t_imu), 0.5*ones(N_imu,1)];

%% 3. Simulate GPS measurements (noisy, 1 Hz)
t_gps = (0:1/fs_gps:T)';
idx_gps = min(round(t_gps*fs_imu)+1, N_imu);
gpsPos = truePos(idx_gps,:) + 1.5*randn(numel(idx_gps),3);
gpsVel = trueVel(idx_gps,:) + 0.1*randn(numel(idx_gps),3);

%% 4. Simulate IMU measurements (noisy accelerometer + gyro)
accelBias = 0.02*ones(1,3);
gyroBias  = 0.001*ones(1,3);
accelNoise = 0.05;
gyroNoise  = 0.002;

% Numerical derivatives to get "true" acceleration
trueAcc = [diff(trueVel(:,1))/ (1/fs_imu); 0], ...
    [diff(trueVel(:,2))/ (1/fs_imu); 0], ...
    [diff(trueVel(:,3))/ (1/fs_imu); 0];
trueAcc = [gradient(trueVel(:,1), 1/fs_imu), ...
    gradient(trueVel(:,2), 1/fs_imu), ...
    gradient(trueVel(:,3), 1/fs_imu)];

% Assume gyro measures turn rate around Z
trueGyro = [zeros(N_imu,2), omega*ones(N_imu,1)];

accelMeas = trueAcc + accelBias + accelNoise*randn(N_imu,3);
gyroMeas  = trueGyro + gyroBias + gyroNoise*randn(N_imu,3);

fprintf('Simulated %d IMU samples (%.1f s)\n', N_imu, T);
fprintf('Simulated %d GPS samples\n\n', numel(idx_gps));

%% 5. Visualize
figure('Name','GNSS + IMU Fusion Demo','Color','w','Position',[100 100 900 600]);

subplot(2,2,1);
plot3(truePos(:,1), truePos(:,2), truePos(:,3), 'b-', 'LineWidth', 2); hold on;
scatter3(gpsPos(:,1), gpsPos(:,2), gpsPos(:,3), 30, 'r', 'filled', ...
    'MarkerFaceAlpha', 0.5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('True trajectory','GPS (noisy)','Location','best');
title('Trajectory (Top View)');
view(2);

subplot(2,2,2);
plot(t_imu, truePos(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t_gps, gpsPos(:,3), 'r.', 'MarkerSize', 10);
grid on;
xlabel('Time (s)'); ylabel('Z (m)');
legend('True Z','GPS Z','Location','best');
title('Altitude');

subplot(2,2,3);
plot(t_imu, accelMeas(:,1), 'r', 'LineWidth', 0.8); hold on;
plot(t_imu, trueAcc(:,1), 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Accel X (m/s^2)');
legend('Measured (noisy)','True','Location','best');
title('IMU Accelerometer X');

subplot(2,2,4);
plot(t_imu, gyroMeas(:,3)*180/pi, 'r', 'LineWidth', 0.8); hold on;
plot(t_imu, trueGyro(:,3)*180/pi, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Gyro Z (deg/s)');
legend('Measured (noisy)','True','Location','best');
title('IMU Gyroscope Z');

sgtitle('GNSS + IMU Sensor Simulation (Before Fusion)');

fprintf('Demo complete. Check the figure window.\n');
fprintf('\nNext step: Add the EKF predict/correct loop to fuse these sensors.\n');
