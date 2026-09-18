%% Demo: EKF Diagnostic — Q Value Sweep
% Hanish - Localization Subsystem
% Tests 5 different Q values to find the optimal tuning

clear; clc; rng(42);
dt = 0.01; T = 30; N = T/dt; t = (0:N-1)'*dt;

% Straight-line motion (no acceleration, simplest possible case)
vx_true = 10*ones(N,1);
vy_true = 5*ones(N,1);
truePos = cumsum([vx_true, vy_true])*dt;
trueVel = [vx_true, vy_true];
trueAcc = zeros(N,2);

% GPS at 5 Hz
gpsIdx = 1:20:N;
gpsNoise = 1.5;
gpsMeas = truePos(gpsIdx,:) + gpsNoise*randn(numel(gpsIdx),2);

% IMU at 100 Hz
imuNoise = 0.05;
imuMeas = trueAcc + imuNoise*randn(N,2);

% Sweep Q values
fprintf('\n Q_scale | GPS_err | EKF_err | Improvement\n');
fprintf('---------|---------|---------|-------------\n');

for q_scale = [0.001, 0.01, 0.1, 1, 10]
    F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
    H = [1 0 0 0; 0 1 0 0];
    Q = diag([q_scale, q_scale, q_scale*10, q_scale*10]);
    R = diag([gpsNoise^2, gpsNoise^2]);

    xEst = [gpsMeas(1,1); gpsMeas(1,2); 0; 0];
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

    ekfAtGps = xHist(gpsIdx, 1:2);
    err_gps = mean(vecnorm(gpsMeas - truePos(gpsIdx,:), 2, 2));
    err_ekf = mean(vecnorm(ekfAtGps - truePos(gpsIdx,:), 2, 2));
    fprintf(' %7.3f | %7.3f | %7.3f | %6.2fx\n', ...
        q_scale, err_gps, err_ekf, err_gps/err_ekf);
end

fprintf('\n');