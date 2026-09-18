%% Demo: 3D GNSS + IMU Fusion with insEKF
% Hanish - Localization Subsystem
% Produces full 3D pose: [x, y, z, roll, pitch, yaw, vx, vy, vz]
% Output format matches project interface to Yashwanth (planning)
%% EXPERIMENTAL — SHELVED 2026-09-16
% Attempt to use insEKF for 3D GNSS/IMU fusion.
% Status: DIVERGES (mean position error ~42 m, improvement 0.06x).
%
% Root cause: insEKF + insMotionPose expects body-frame IMU measurements
% (specific force, angular velocity) rotated by the filter's orientation
% estimate. Our simulation provides navigation-frame measurements.
% Fixing requires full 3D body-frame kinematics — not worth the effort
% for a local Cartesian simulation.
%
% Superseded by: ekf_3d_baseline.m (generic EKF, 6-state, 3.89x improvement).
% Kept for reference in case we move to real sensor data with LLA coordinates.
%
% =====================================================================

%% Demo: 3D GNSS + IMU Fusion with insEKF

clear; clc; close all;
rng(42);

%% 1. Parameters
fs_imu = 100;          % IMU rate (Hz)
fs_gps = 5;            % GPS rate (Hz)
T = 20;                % Duration (s)
dt = 1/fs_imu;
N = T*fs_imu;
t = (0:N-1)'*dt;

%% 2. Ground truth trajectory (straight-ish, zero yaw for simplicity)
R = 10;
omega = 0;             % zero rotation → body frame = global frame
vz_true = 0.5;
truePos = [R*sin(0.1*t), R*cos(0.1*t), vz_true*t];
trueVel = [R*0.1*cos(0.1*t), -R*0.1*sin(0.1*t), vz_true*ones(N,1)];
trueAcc = [-R*0.1^2*sin(0.1*t), -R*0.1^2*cos(0.1*t), zeros(N,1)];
trueYaw = zeros(N,1);

%% 3. Create INS sensor models (for insEKF)
accSensor     = insAccelerometer;
gyroSensor    = insGyroscope;
gpsSensorObj  = insGPS;
gpsSensorObj.ReferenceLocation = [0 0 0];   % local frame origin at LLA [0 0 0]

%% 4. Create insEKF with sensors and motion model
filter = insEKF(accSensor, gyroSensor, gpsSensorObj, insMotionPose);
disp('insEKF initialized with Accelerometer, Gyroscope, GPS');

%% 5. Initialize filter state
stateparts(filter, 'Position',    truePos(1,:) + 1.5*randn(1,3));
stateparts(filter, 'Velocity',    [0 0 0]);
stateparts(filter, 'Orientation', [1 0 0 0]);   % [w x y z]

% Initial covariance (large for velocity since unknown at start)
statecovparts(filter, 'Position', 1.5^2 * eye(3));
statecovparts(filter, 'Velocity', 100 * eye(3));

%% 6. Initialize logging
xHist = zeros(N, 9);   % [x y z roll pitch yaw vx vy vz]
p0 = stateparts(filter, 'Position');
v0 = stateparts(filter, 'Velocity');
q0 = quaternion(stateparts(filter, 'Orientation'));
eul0 = euler(q0, 'ZYX', 'frame');
xHist(1,:) = [p0, eul0(3), eul0(2), eul0(1), v0];

gpsIdx = round(1 : fs_imu/fs_gps : N);
gpsCounter = 1;

fprintf('Running insEKF over %d samples...\n', N);

%% 7. Main fusion loop
for k = 2:N
    % --- IMU measurement: specific force (gravity + linear accel) ---
    gravity = [0; 0; 9.81];                                 % NED: gravity points down
    accelMeas = (trueAcc(k,:)' + gravity + 0.05*randn(3,1))';
    gyroMeas  = 0.002*randn(1,3);                           % zero rotation

    % --- Predict and fuse IMU ---
    predict(filter, dt);
    fuse(filter, accSensor, accelMeas, (0.05^2) * eye(3));
    fuse(filter, gyroSensor, gyroMeas,  (0.002^2) * eye(3));

    % --- GPS update (convert local XYZ → LLA) ---
    if gpsCounter <= numel(gpsIdx) && k == gpsIdx(gpsCounter)
        % GPS measurement in LOCAL NED coordinates (meters)
        localPos = truePos(k,:) + 1.5*randn(1,3);

        % Correct the position state directly
        posIdx = stateinfo(filter, 'Position');   % [8 9 10] for insMotionPose
        correct(filter, posIdx, localPos, (1.5^2) * eye(3));

        gpsCounter = gpsCounter + 1;
    end

    % --- Log state ---
    p = stateparts(filter, 'Position');
    v = stateparts(filter, 'Velocity');
    q = quaternion(stateparts(filter, 'Orientation'));
    eul = euler(q, 'ZYX', 'frame');
    xHist(k,:) = [p, eul(3), eul(2), eul(1), v];
end

fprintf('Fusion complete.\n');

%% 8. Compute errors
posError_ekf = vecnorm(xHist(:,1:3) - truePos, 2, 2);
velError_ekf = vecnorm(xHist(:,7:9) - trueVel, 2, 2);
yawError_ekf = abs(wrapToPi(xHist(:,6)' - trueYaw'));

% GPS-only error at GPS sample times
gpsPosNoisy = truePos(gpsIdx,:) + 1.5*randn(numel(gpsIdx),3);
posError_gps = vecnorm(gpsPosNoisy - truePos(gpsIdx,:), 2, 2);

fprintf('\n=== insEKF 3D Fusion Results ===\n');
fprintf('Mean GPS position error:    %.3f m\n', mean(posError_gps));
fprintf('Mean EKF position error:    %.3f m\n', mean(posError_ekf));
fprintf('Mean EKF velocity error:    %.3f m/s\n', mean(velError_ekf));
fprintf('Mean EKF yaw error:         %.3f rad (%.2f deg)\n', ...
    mean(yawError_ekf), rad2deg(mean(yawError_ekf)));
fprintf('Improvement factor:         %.2fx\n', ...
    mean(posError_gps)/mean(posError_ekf));

%% 9. Plots
figure('Name','insEKF 3D Fusion','Color','w','Position',[100 100 1200 700]);

subplot(2,3,1);
plot3(truePos(:,1), truePos(:,2), truePos(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(xHist(:,1), xHist(:,2), xHist(:,3), 'g-', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Truth','EKF','Location','best');
title('3D Trajectory');

subplot(2,3,2);
plot(t, posError_ekf, 'g-', 'LineWidth', 1.5); hold on;
plot(t(gpsIdx), posError_gps, 'r.', 'MarkerSize', 10);
grid on;
xlabel('Time (s)'); ylabel('Position error (m)');
legend('EKF','GPS','Location','best');
title('Position Error');

subplot(2,3,3);
plot(t, trueVel(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, xHist(:,7), 'g-', 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('Vx (m/s)');
legend('Truth','EKF','Location','best'); title('Velocity X');

subplot(2,3,4);
plot(t, rad2deg(trueYaw), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(xHist(:,6)), 'g-', 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('Yaw (deg)');
legend('Truth','EKF','Location','best'); title('Yaw (Heading)');

subplot(2,3,5);
plot(t, rad2deg(xHist(:,4)), 'r-', 'LineWidth', 1); hold on;
plot(t, rad2deg(xHist(:,5)), 'g-', 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('deg');
legend('Roll','Pitch','Location','best'); title('Roll & Pitch');

subplot(2,3,6);
plot(t, velError_ekf, 'g-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Velocity error (m/s)');
title('Velocity Error');

sgtitle('insEKF 3D Localization - Hanish');