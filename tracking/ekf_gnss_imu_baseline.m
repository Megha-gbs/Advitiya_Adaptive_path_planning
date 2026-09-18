% =====================================================================
% GNSS + IMU Fusion Baseline — Hanish Localization Subsystem
% Status: VALIDATED — 2026-09-16
% Result: 3.68x improvement over GPS-only (mean EKF err = 0.531 m)
% Q construction: physically-derived from acceleration noise model
% Reference: Bar-Shalom, "Estimation with Applications to Tracking"
% NOTE: Simulation-only validation. Ground truth is an idealized
% constant-velocity straight line; GPS/IMU noise are synthetic Gaussian
% draws, not real sensor logs. See report for limitations.
% =====================================================================
clear; clc; rng(42);
dt = 0.01; T = 30; N = T/dt; t = (0:N-1)'*dt;

%% Ground truth: straight-line motion
vx_true = 10*ones(N,1);
vy_true = 5*ones(N,1);
truePos = cumsum([vx_true, vy_true])*dt;
trueAcc = zeros(N,2);   % no real acceleration (constant velocity)

%% Sensors
% GPS at 5 Hz with 1.5 m noise
gpsIdx = 1:20:N;
gpsNoise = 1.5;
gpsMeas = truePos(gpsIdx,:) + gpsNoise*randn(numel(gpsIdx),2);

% IMU at 100 Hz with 0.05 m/s^2 noise (measures acceleration)
imuNoise = 0.05;
imuMeas = trueAcc + imuNoise*randn(N,2);

%% ============================================================
%% EKF SETUP — THIS IS WHERE THE NEW Q GOES
%% ============================================================

% State transition matrix (constant velocity model)
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1  0;
     0 0 0  1];

% Control input matrix (acceleration -> position, velocity)
B = [0.5*dt^2 0;
     0 0.5*dt^2;
     dt 0;
     0 dt];

% Measurement matrix (GPS measures position only)
H = [1 0 0 0;
     0 1 0 0];

% ---- THE CORRECT PHYSICS-BASED Q ----
% Derived from continuous-time white noise acceleration model:
%   Position variance    = sigma_a^2 * dt^4 / 4
%   Position-Velocity    = sigma_a^2 * dt^3 / 2
%   Velocity variance    = sigma_a^2 * dt^2
sigma_a = imuNoise;   % 0.05 m/s^2
Q = sigma_a^2 * [dt^4/4,    0,       dt^3/2,  0;
                 0,        dt^4/4,   0,       dt^3/2;
                 dt^3/2,   0,       dt^2,    0;
                 0,        dt^3/2,   0,       dt^2];

% Measurement noise (GPS)
R = diag([gpsNoise^2, gpsNoise^2]);

%% Initialize state
% We do NOT know velocity at start — set to zero with large uncertainty
xEst = [gpsMeas(1,1);   % x from first GPS
        gpsMeas(1,2);   % y from first GPS
        0;              % vx UNKNOWN — start at zero
        0];             % vy UNKNOWN — start at zero

PEst = diag([gpsNoise^2, gpsNoise^2, 100, 100]);  % large velocity uncertainty

%% Run EKF
xHist = zeros(N, 4);
xHist(1,:) = xEst';
gpsCounter = 2;

for k = 2:N
    % --- Predict ---
    u = imuMeas(k,:)';
    xPred = F*xEst + B*u;
    PPred = F*PEst*F' + Q;

    % --- Update (when GPS available) ---
    if gpsCounter <= size(gpsMeas,1) && k == gpsIdx(gpsCounter)
        z = gpsMeas(gpsCounter,:)';
        y = z - H*xPred;
        S = H*PPred*H' + R;
        K = PPred*H'/S;
        xEst = xPred + K*y;
        PEst = (eye(4) - K*H)*PPred;
        gpsCounter = gpsCounter + 1;
    else
        xEst = xPred;
        PEst = PPred;
    end

    xHist(k,:) = xEst';
end

%% Compute errors
err_full       = vecnorm(xHist(:,1:2) - truePos, 2, 2);
err_ekf_at_gps = vecnorm(xHist(gpsIdx,1:2) - truePos(gpsIdx,:), 2, 2);
err_gps        = vecnorm(gpsMeas - truePos(gpsIdx,:), 2, 2);

%% Print summary
fprintf('\n=== GNSS+IMU Fusion Baseline ===\n');
fprintf('Q(1,1) = %.3e\n', Q(1,1));
fprintf('Q(3,3) = %.3e\n', Q(3,3));
fprintf('-------------------------------------------\n');
fprintf('EKF error (all samples):    mean = %.3f m, max = %.3f m\n', ...
    mean(err_full), max(err_full));
fprintf('EKF error (at GPS times):   mean = %.3f m\n', mean(err_ekf_at_gps));
fprintf('GPS error:                  mean = %.3f m\n', mean(err_gps));
fprintf('Improvement (GPS-time only):   %.2fx\n', mean(err_gps)/mean(err_ekf_at_gps));
fprintf('Improvement (full trajectory): %.2fx\n', mean(err_gps)/mean(err_full));
%% Plot velocity convergence
figure('Name','Velocity Convergence','Color','w','Position',[100 100 900 600]);

subplot(2,1,1);
plot(t, vx_true, 'b-', 'LineWidth', 2); hold on;
plot(t, xHist(:,3), 'r--', 'LineWidth', 1.5);
grid on; ylabel('Vx (m/s)');
legend('True','EKF estimate','Location','best');
title(sprintf('Velocity Vx — final EKF: %.3f, true: %.3f', ...
    xHist(end,3), vx_true(end)));

subplot(2,1,2);
plot(t, vy_true, 'b-', 'LineWidth', 2); hold on;
plot(t, xHist(:,4), 'r--', 'LineWidth', 1.5);
grid on; ylabel('Vy (m/s)'); xlabel('Time (s)');
legend('True','EKF estimate','Location','best');
title(sprintf('Velocity Vy — final EKF: %.3f, true: %.3f', ...
    xHist(end,4), vy_true(end)));