function [predictedState, predictedCovariance] = ...
    kalmanPredict(state, covariance, dt)

%KALMANPREDICT Predicts object state using a constant-velocity model.
%
% State:
%   [x; y; vx; vy]
%
% Inputs:
%   state      - Current state [x; y; vx; vy]
%   covariance - Current state covariance (4x4)
%   dt         - Time step [s]
%
% Outputs:
%   predictedState      - Predicted state [x; y; vx; vy]
%   predictedCovariance - Predicted covariance (4x4)

    % State transition matrix
    A = [1  0  dt  0;
         0  1  0   dt;
         0  0  1   0;
         0  0  0   1];

    % Process noise
    accelerationNoise = 2.0;

    Q = [dt^4/4  0        dt^3/2  0;
         0       dt^4/4   0       dt^3/2;
         dt^3/2  0        dt^2    0;
         0       dt^3/2   0       dt^2] ...
         * accelerationNoise^2;

    % Prediction
    predictedState = A * state;

    predictedCovariance = ...
        A * covariance * A' + Q;

end