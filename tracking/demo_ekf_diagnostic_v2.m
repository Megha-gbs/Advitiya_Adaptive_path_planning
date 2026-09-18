%% Diagnostic v2: Measure EKF error BETWEEN GPS updates
clear; clc; rng(42);
dt = 0.01; T = 30; N = T/dt; t = (0:N-1)'*dt;

vx_true = 10*ones(N,1);
vy_true = 5*ones(N,1);
truePos = cumsum([vx_true, vy_true])*dt;
trueAcc = zeros(N,2);

gpsIdx = 1:20:N;
gpsNoise = 1.5;
gpsMeas = truePos(gpsIdx,:) + gpsNoise*randn(numel(gpsIdx),2);

imuNoise = 0.05;
imuMeas = trueAcc + imuNoise*randn(N,2);

% Use best Q from previous test
q_scale = 0.001;
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
H = [1 0 0 0; 0 1 0 0];
Q = diag([q_scale, q_scale, q_scale*10, q_scale*10]);
R = diag([gpsNoise^2, gpsNoise^2]);

xEst = [gpsMeas(1,1); gpsMeas(1,2); 10; 5];
PEst = diag([gpsNoise^2, gpsNoise^2, 1, 1]);
xHist = zeros(N, 4); xHist(1,:) = xEst';
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
        PEst = (eye(4) - K*H)*PPred;
        gpsCounter = gpsCounter + 1;
    else
        xEst = xPred;
        PEst = PPred;
    end
    xHist(k,:) = xEst';
end

% Full trajectory error (all samples)
err_full = vecnorm(xHist(:,1:2) - truePos, 2, 2);
err_gps_full = nan(N,1);
err_gps_full(gpsIdx) = vecnorm(gpsMeas - truePos(gpsIdx,:), 2, 2);

fprintf('\n=== Diagnostic v2 ===\n');
fprintf('EKF error (all %d samples):    mean = %.3f m, max = %.3f m\n', ...
    N, mean(err_full), max(err_full));
fprintf('GPS error (at %d GPS times):   mean = %.3f m\n', ...
    numel(gpsIdx), mean(err_gps_full(~isnan(err_gps_full))));
fprintf('Ratio (GPS / EKF):             %.2fx\n', ...
    mean(err_gps_full(~isnan(err_gps_full))) / mean(err_full));