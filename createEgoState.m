function egoState = createEgoState( ...
    timestamp, position, velocity, ...
    acceleration, orientation, angularVelocity, ...
    covariance, frame)

%CREATEEGOSTATE Creates a standardized ego-vehicle state.
%
% Inputs:
%   timestamp       - Time [s]
%   position        - Ego position [m]
%   velocity        - Ego velocity [m/s]
%   acceleration    - Ego acceleration [m/s^2]
%   orientation     - Ego orientation [rad]
%   an4gularVelocity - Ego angular velocity [rad/s]
%   covariance      - State uncertainty/covariance
%   frame           - Coordinate frame
%
% Output:
%   egoState        - Standardized ego-vehicle state structure

    egoState.timestamp       = timestamp;
    egoState.position        = position;
    egoState.velocity        = velocity;
    egoState.acceleration    = acceleration;
    egoState.orientation     = orientation;
    egoState.angularVelocity = angularVelocity;
    egoState.covariance      = covariance;
    egoState.frame           = string(frame);

end