function [updatedState, updatedCovariance, innovation, ...
    innovationCovariance] = ...
    kalmanUpdate(predictedState, predictedCovariance, ...
    measurement, measurementNoise)

%KALMANUPDATE Updates a predicted state using a position measurement.
%
% State:
%   [x; y; vx; vy]
%
% Measurement:
%   [x; y]
%
% Outputs also include innovation information for
% uncertainty-aware association.

    % Measurement model
    H = [1 0 0 0;
         0 1 0 0];

    % Innovation
    innovation = ...
        measurement(:) - H * predictedState;

    % Innovation covariance
    innovationCovariance = ...
        H * predictedCovariance * H' + measurementNoise;

    % Kalman gain
    K = predictedCovariance * H' / ...
        innovationCovariance;

    % State update
    updatedState = ...
        predictedState + K * innovation;

    % Joseph-form covariance update
    I = eye(4);

    updatedCovariance = ...
        (I - K*H) * predictedCovariance * (I - K*H)' ...
        + K * measurementNoise * K';

end