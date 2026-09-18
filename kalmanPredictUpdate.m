function [updatedState, updatedCovariance] = ...
    kalmanPredictUpdate(state, covariance, ...
    measurement, dt, measurementNoise)

%KALMANPREDICTUPDATE Performs Kalman prediction and measurement update.
%
% This function is retained for backward compatibility.
%
% State:
%   [x; y; vx; vy]
%
% Measurement:
%   [x; y]
%
% Inputs:
%   state
%   covariance
%   measurement
%   dt
%   measurementNoise
%
% Outputs:
%   updatedState
%   updatedCovariance

    %--------------------------------------------------------------
    % Prediction
    %--------------------------------------------------------------

    [predictedState, predictedCovariance] = ...
        kalmanPredict( ...
            state, ...
            covariance, ...
            dt);


    %--------------------------------------------------------------
    % Measurement update
    %--------------------------------------------------------------

    [updatedState, updatedCovariance] = ...
        kalmanUpdate( ...
            predictedState, ...
            predictedCovariance, ...
            measurement, ...
            measurementNoise);

end