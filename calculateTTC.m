function [ttc, closingSpeed, relativeDistance] = ...
    calculateTTC(egoState, track)

%CALCULATETTC Calculates Time To Collision (TTC).
%
% Inputs:
%   egoState - Ego vehicle state
%   track    - Tracked object state
%
% Outputs:
%   ttc             - Time To Collision [s]
%   closingSpeed    - Relative closing speed [m/s]
%   relativeDistance - Distance between ego and object [m]
%
% TTC is calculated only when the object is moving
% toward the ego vehicle.

    % ---------------------------------------------------------
    % Relative position
    % ---------------------------------------------------------

    relativePosition = ...
        track.position - egoState.position;

    % Distance between ego vehicle and object
    relativeDistance = norm(relativePosition);


    % ---------------------------------------------------------
    % Relative velocity
    % ---------------------------------------------------------

    relativeVelocity = ...
        track.velocity - egoState.velocity;


    % ---------------------------------------------------------
    % Calculate closing speed
    % ---------------------------------------------------------

    % Unit vector from ego vehicle toward object
    if relativeDistance > 0

        direction = ...
            relativePosition / relativeDistance;

    else

        direction = [0 0];

    end

    % Relative velocity along line of sight
    radialRelativeVelocity = ...
        dot(relativeVelocity, direction);

    % Positive closing speed means distance is decreasing
    closingSpeed = -radialRelativeVelocity;


    % ---------------------------------------------------------
    % Calculate TTC
    % ---------------------------------------------------------

    if closingSpeed > 0

        ttc = relativeDistance / closingSpeed;

    else

        % Object is not approaching ego vehicle
        ttc = inf;

    end

end