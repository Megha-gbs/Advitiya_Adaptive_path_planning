function [distanceSquared, isValid] = ...
    mahalanobisDistance( ...
    measurement, predictedState, ...
    predictedCovariance, measurementNoise, threshold)

%MAHALANOBISDISTANCE Calculates uncertainty-aware measurement distance.
%
% Measurement:
%   [x y]
%
% State:
%   [x y vx vy]
%
% threshold:
%   Maximum allowed squared Mahalanobis distance.

    % Measurement model
    H = [1 0 0 0;
         0 1 0 0];

    % Innovation
    innovation = ...
        measurement(:) - H * predictedState;

    % Innovation covariance
    S = ...
        H * predictedCovariance * H' + ...
        measurementNoise;

    % Mahalanobis distance squared
    distanceSquared = ...
        innovation' * (S \ innovation);

    % Association decision
    isValid = ...
        distanceSquared <= threshold;

end