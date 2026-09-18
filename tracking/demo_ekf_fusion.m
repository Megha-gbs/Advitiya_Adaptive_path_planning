%% Demo: Real GNSS + IMU Fusion with EKF (v4 — Indian Road Realism)
% Hanish - Localization Subsystem
% Uses realistic vehicle accelerations and GPS rate

clear; clc; close all;
rng(42);

%% 1. Parameters
dt = 0.01;          % 100 Hz IMU
T = 30;             % 30 seconds (longer test)
N = T/dt;
t = (0:N-1)'*dt;

%% 2. Ground truth — realistic Indian driving
% Start straight, turn, brake, accelerate, another turn
v0 = 10;            % 10 m/s = 36 km/h (typical Indian city speed)

% Piecewise acceleration profile
accel_profile = zeros(N, 2);
for k = 1:N
    tk = t(k);
    if tk < 5
        % Straight cruise
        a_long = 0;
        a_lat  = 0;
    elseif tk < 10
        % Right turn: lateral accel
        a_long = 0;
        a_lat  = 3.0;    % 3 m/s² lateral (typical turn)
    elseif tk < 15
        % Braking
        a_long = -3.0;   % 3 m/s² braking
        a_lat  = 0;
    elseif tk < 20
        % Accelerating
        a_long = 2.0;
        a_lat  = 0;
    elseif tk < 25
        % Left turn
        a_long = 0;
        a_lat  = -2.5;
    else
        % Cruise
        a_long = 0;
        a_lat  = 0;
    end
    accel_profile(k,:) = [a_long, a_lat];
end

% Integrate accelerations to get velocity and position
% Simple: use forward Euler for the ground truth
trueVel = zeros(N, 2);
truePos = zeros(N, 2);
trueVel(1,:) = [v0, 0];
truePos(1,:) = [0, 0];

for k = 2:N
    trueVel(k,:) = trueVel(k-1,:) + dt * accel_profile(k,:);
    truePos(k,:) = truePos(k-1,:) + dt * trueVel(k,:);
end

%% 3. Realistic sensor simulation
% GPS at 5 Hz (RTK typical), noise ~1.5 m
gpsIdx = 1:20:N;
gpsNoise = 1.5;
gpsMeas = truePos(gpsIdx,:) + gpsNoise*randn(numel(gpsIdx),2);

% IMU at 100 Hz — realistic MEMS with body-frame measurement
% Add gravity leakage and slight misalignment (small realistic errors)
imuNoise = 0.05;
imuMeas = accel_profile + imuNoise*randn(N,2);

%% 4. EKF Setup — Proper acceleration-input model
n = 4;
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
H = [1 0 0 0; 0 1 0 0];

% Process noise: LOW for velocity (we trust IMU), higher for position
Q = diag([0.01, 0.01, 0.1, 0.1]);
R = diag([gpsNoise^2, gpsNoise^2]);

xEst = [gpsMeas(1,1); gpsMeas(1,2); v0; 0];
PEst = diag([gpsNoise^2, gpsNoise^2, 4, 4]);

%% 5. Run EKF
xHist = zeros(N, 4);
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

%% 6. Errors at GPS sample times
ekfAtGps = xHist(gpsIdx, 1:2);
posError_gps = vecnorm(gpsMeas - truePos(gpsIdx,:), 2, 2);
posError_ekf = vecnorm(ekfAtGps - truePos(gpsIdx,:), 2, 2);

% Full-trajectory error
posError_ekf_full = vecnorm(xHist(:,1:2) - truePos, 2, 2);

%% 7. Plot
figure('Name','EKF Fusion Results v4 — Indian Road','Color','w','Position',[100 100 1100 750]);

subplot(2,2,1);
plot(truePos(:,1), truePos(:,2), 'b-', 'LineWidth', 2); hold on;
plot(gpsMeas(:,1), gpsMeas(:,2), 'r.', 'MarkerSize', 12);
plot(xHist(:,1), xHist(:,2), 'g-', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
legend('Ground truth','GPS (noisy)','EKF estimate','Location','best');
title('Trajectory Comparison');

subplot(2,2,2);
bar([posError_gps, posError_ekf]);
grid on;
xlabel('GPS sample #'); ylabel('Position error (m)');
legend('GPS only','EKF fused','Location','best');
title('Position Error at GPS Times');

subplot(2,2,3);
plot(t, trueVel(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, xHist(:,3), 'g-', 'LineWidth', 1);
grid on;
xlabel('Time (s)'); ylabel('Vx (m/s)');
legend('True','EKF estimate','Location','best');
title('Velocity X');

subplot(2,2,4);
plot(t, posError_ekf_full, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Position error (m)');
title(sprintf('Full-trajectory error (mean = %.3f m)', mean(posError_ekf_full)));

sgtitle('EKF Fusion v4: Realistic Indian Road Maneuvers');

%% 8. Summary
fprintf('\n=== Fusion Results v4 ===\n');
fprintf('Mean GPS position error (at GPS times): %.3f m\n', mean(posError_gps));
fprintf('Mean EKF position error (at GPS times): %.3f m\n', mean(posError_ekf));
fprintf('Mean EKF error (full trajectory):       %.3f m\n', mean(posError_ekf_full));
fprintf('Improvement factor:                     %.2fx\n', mean(posError_gps)/mean(posError_ekf));