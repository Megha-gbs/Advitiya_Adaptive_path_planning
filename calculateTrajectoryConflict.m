function conflict = calculateTrajectoryConflict( ...
    egoState, track, predictionTime, timeStep, ...
    egoLength, egoWidth, objectSafetyMargin)

%CALCULATETRAJECTORYCONFLICT Checks predicted collision conflict.
%
% This function compares the predicted trajectory of an object
% against the future occupied region of the ego vehicle.
%
% Inputs:
%
%   egoState
%       Ego vehicle state.
%
%   track
%       Tracked object.
%
%   predictionTime
%       Prediction horizon [s].
%
%   timeStep
%       Prediction time step [s].
%
%   egoLength
%       Ego vehicle length [m].
%
%   egoWidth
%       Ego vehicle width [m].
%
%   objectSafetyMargin
%       Additional safety margin around object [m].
%
% Output:
%
%   conflict
%       Structure containing trajectory conflict information.
%
% Coordinate convention:
%
%   vehicle frame:
%
%       x = forward
%       y = lateral
%
% IMPORTANT:
%
% This is a geometric conflict detector.
% It does not by itself determine the final risk level.

    %--------------------------------------------------------------
    % Predict object trajectory
    %--------------------------------------------------------------

    predictedTrajectory = ...
        predictObjectTrajectory( ...
        track, ...
        predictionTime, ...
        timeStep);

    %--------------------------------------------------------------
    % Ego vehicle dimensions
    %--------------------------------------------------------------

    halfEgoLength = ...
        egoLength / 2;

    halfEgoWidth = ...
        egoWidth / 2;

    %--------------------------------------------------------------
    % Object safety boundary
    %--------------------------------------------------------------

    objectRadius = ...
        objectSafetyMargin;

    %--------------------------------------------------------------
    % Initialize outputs
    %--------------------------------------------------------------

    minimumDistance = inf;

    conflictDetected = false;

    conflictTime = inf;

    conflictPosition = [NaN NaN];

    %--------------------------------------------------------------
    % Compare object trajectory with ego occupied area
    %--------------------------------------------------------------

    for k = 1:size(predictedTrajectory.position,1)

        %----------------------------------------------------------
        % Object position relative to ego
        %----------------------------------------------------------

        objectPosition = ...
            predictedTrajectory.position(k,:);

        relativePosition = ...
            objectPosition - egoState.position;

        %----------------------------------------------------------
        % Ego future position
        %
        % For this first implementation we assume the ego vehicle
        % maintains its current velocity.
        %----------------------------------------------------------

        futureTime = ...
            predictedTrajectory.time(k);

        egoFuturePosition = ...
            egoState.position + ...
            egoState.velocity * futureTime;

        %----------------------------------------------------------
        % Relative object position at future time
        %----------------------------------------------------------

        relativeFuturePosition = ...
            objectPosition - egoFuturePosition;

        %----------------------------------------------------------
        % Longitudinal and lateral distances
        %----------------------------------------------------------

        longitudinalDistance = ...
            abs(relativeFuturePosition(1));

        lateralDistance = ...
            abs(relativeFuturePosition(2));

        %----------------------------------------------------------
        % Distance from ego occupied rectangle
        %
        % If inside the rectangle, distance is zero.
        %----------------------------------------------------------

        longitudinalClearance = ...
            max( ...
            0, ...
            longitudinalDistance - ...
            halfEgoLength - ...
            objectRadius);

        lateralClearance = ...
            max( ...
            0, ...
            lateralDistance - ...
            halfEgoWidth - ...
            objectRadius);

        distance = ...
            sqrt( ...
            longitudinalClearance^2 + ...
            lateralClearance^2);

        %----------------------------------------------------------
        % Track minimum distance
        %----------------------------------------------------------

        if distance < minimumDistance

            minimumDistance = ...
                distance;

        end

        %----------------------------------------------------------
        % Collision/conflict condition
        %----------------------------------------------------------

        longitudinalConflict = ...
            longitudinalDistance <= ...
            (halfEgoLength + objectRadius);

        lateralConflict = ...
            lateralDistance <= ...
            (halfEgoWidth + objectRadius);

        if longitudinalConflict && lateralConflict

            conflictDetected = true;

            conflictTime = ...
                predictedTrajectory.time(k);

            conflictPosition = ...
                objectPosition;

            break;

        end

    end

    %--------------------------------------------------------------
    % Estimate conflict severity
    %--------------------------------------------------------------

    if conflictDetected

        if conflictTime <= 1.0

            severity = "CRITICAL";

        elseif conflictTime <= 2.0

            severity = "HIGH";

        elseif conflictTime <= 4.0

            severity = "MEDIUM";

        else

            severity = "LOW";

        end

    else

        severity = "NONE";

    end

    %--------------------------------------------------------------
    % Output structure
    %--------------------------------------------------------------

    conflict.predictedTrajectory = ...
        predictedTrajectory;

    conflict.conflictDetected = ...
        conflictDetected;

    conflict.conflictTime = ...
        conflictTime;

    conflict.minimumDistance = ...
        minimumDistance;

    conflict.conflictPosition = ...
        conflictPosition;

    conflict.severity = ...
        severity;

    conflict.trackId = ...
        track.id;

    conflict.objectClass = ...
        track.class;

    conflict.timestamp = ...
        egoState.timestamp;

    conflict.frame = ...
        egoState.frame;

end