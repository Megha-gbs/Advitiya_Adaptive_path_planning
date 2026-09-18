% =====================================================================
% 3D GNSS + IMU Fusion Baseline — Hanish Localization Subsystem
% Status: Extends validated 2D baseline (3.68x) to 3D
% Output: [x y z vx vy vz] — matches interface to Yashwanth (planning)
% Q construction: physically-derived from acceleration noise model
% Reference: Bar-Shalom, "Estimation with Applications to Tracking"
% =====================================================================
% NOTE: Simulation-only validation. GPS noise uses separate horizontal
% (1.5 m) and vertical (2.5 m) components. IMU input is navigation-frame
% acceleration, not body-frame specific force. See report for limitations.
clear; clc; close all; rng(42);

%% 1. Parameters
dt = 0.01;              % 100 Hz IMU
T = 30;
N = T/dt;
t = (0:N-1)'*dt;

%% 2. Ground truth — 3D trajectory (circle + climb)
R = 10;                 % circle radius (m)
omega = 0.2;            % rad/s
vz_true = 0.5;          % climb rate (m/s)

truePos = [R*sin(omega*t), R*cos(omega*t), vz_true*t];
trueVel = [R*omega*cos(omega*t), -R*omega*sin(omega*t), vz_true*ones(N,1)];
trueAcc = [-R*omega^2*sin(omega*t), -R*omega^2*cos(omega*t), zeros(N,1)];

%% 3. Sensors
% GPS at 5 Hz, 1.5 m noise (horizontal), 2.5 m (vertical)
gpsIdx = 1:20:N;
gpsNoiseXY = 1.5;
gpsNoiseZ  = 2.5;
gpsMeas = truePos(gpsIdx,:) + [gpsNoiseXY*randn(numel(gpsIdx),2), gpsNoiseZ*randn(numel(gpsIdx),1)];

% IMU at 100 Hz, 0.05 m/s^2 noise
imuNoise = 0.05;
imuMeas = trueAcc + imuNoise*randn(N,3);

%% 4. EKF Setup — 6-state: [x y z vx vy vz]'
n = 6;

F = [1 0 0 dt 0 0;
     0 1 0 0 dt 0;
     0 0 1 0 0 dt;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

B = [0.5*dt^2, 0,        0;
     0,        0.5*dt^2, 0;
     0,        0,        0.5*dt^2;
     dt,       0,        0;
     0,        dt,       0;
     0,        0,        dt];

H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

% ---- PHYSICS-DERIVED Q (3D) ----
sigma_a = imuNoise;
Q = sigma_a^2 * [dt^4/4, 0,      0,      dt^3/2, 0,      0;
                 0,      dt^4/4, 0,      0,      dt^3/2, 0;
                 0,      0,      dt^4/4, 0,      0,      dt^3/2;
                 dt^3/2, 0,      0,      dt^2,   0,      0;
                 0,      dt^3/2, 0,      0,      dt^2,   0;
                 0,      0,      dt^3/2, 0,      0,      dt^2];

R = diag([gpsNoiseXY^2, gpsNoiseXY^2, gpsNoiseZ^2]);

%% 5. Initialize state
xEst = [gpsMeas(1,1); gpsMeas(1,2); gpsMeas(1,3); 0; 0; 0];
PEst = diag([gpsNoiseXY^2, gpsNoiseXY^2, gpsNoiseZ^2, 100, 100, 100]);

%% 6. Run EKF
xHist = zeros(N, 6);
xHist(1,:) = xEst';
gpsCounter = 2;

for k = 2:N
    u = imuMeas(k,:)';
    xPred = F*xEst + B*u;
    PPred = F*PEst*F' + Q;

    if gpsCounter <= size(gpsMeas,1) && k == gpsIdx(gpsCounter)
        z = gpsMeas(gpsCounter,:)';
        y = z - H*xPred;
        S = H*PPred*H' + R;
        K = PPred*H'/S;
        xEst = xPred + K*y;
        PEst = (eye(n) - K*H)*PPred;
        gpsCounter = gpsCounter + 1;
    else
        xEst = xPred;
        PEst = PPred;
    end

    xHist(k,:) = xEst';
end

%% 7. Compute errors
posError_ekf = vecnorm(xHist(:,1:3) - truePos, 2, 2);
velError_ekf = vecnorm(xHist(:,4:6) - trueVel, 2, 2);
posError_gps = vecnorm(gpsMeas - truePos(gpsIdx,:), 2, 2);

fprintf('\n=== 3D GNSS+IMU Fusion Baseline ===\n');
fprintf('Q(1,1) = %.3e, Q(4,4) = %.3e\n', Q(1,1), Q(4,4));
fprintf('-------------------------------------------\n');
fprintf('Mean GPS position error:    %.3f m\n', mean(posError_gps));
fprintf('Mean EKF position error:    %.3f m\n', mean(posError_ekf));
fprintf('Mean EKF velocity error:    %.3f m/s\n', mean(velError_ekf));
fprintf('Max EKF position error:     %.3f m\n', max(posError_ekf));
fprintf('Improvement factor:         %.2fx\n', ...
    mean(posError_gps)/mean(posError_ekf));

%% 8. Plots
figure('Name','3D EKF Fusion Baseline','Color','w','Position',[100 100 1200 700]);

subplot(2,3,1);
plot3(truePos(:,1), truePos(:,2), truePos(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(xHist(:,1), xHist(:,2), xHist(:,3), 'g-', 'LineWidth', 1.5);
plot3(gpsMeas(:,1), gpsMeas(:,2), gpsMeas(:,3), 'r.', 'MarkerSize', 8);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Truth','EKF','GPS','Location','best');
title('3D Trajectory');

subplot(2,3,2);
plot(t, posError_ekf, 'g-', 'LineWidth', 1.5); hold on;
plot(t(gpsIdx), posError_gps, 'r.', 'MarkerSize', 10);
grid on; xlabel('Time (s)'); ylabel('Position error (m)');
legend('EKF','GPS','Location','best');
title('Position Error');

subplot(2,3,3);
plot(t, trueVel(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, xHist(:,4), 'g-', 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('Vx (m/s)');
legend('Truth','EKF','Location','best'); title('Velocity X');

subplot(2,3,4);
plot(t, trueVel(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, xHist(:,5), 'g-', 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('Vy (m/s)');
legend('Truth','EKF','Location','best'); title('Velocity Y');

subplot(2,3,5);
plot(t, trueVel(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t, xHist(:,6), 'g-', 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('Vz (m/s)');
legend('Truth','EKF','Location','best'); title('Velocity Z');

subplot(2,3,6);
plot(t, velError_ekf, 'g-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Velocity error (m/s)');
title(sprintf('Velocity Error (mean = %.3f m/s)', mean(velError_ekf)));

sgtitle('3D GNSS + IMU Fusion Baseline - Hanish');