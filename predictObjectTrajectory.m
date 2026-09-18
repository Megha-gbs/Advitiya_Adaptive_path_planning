function predictedTrajectory = ...
    predictObjectTrajectory(track, predictionTime, timeStep)

%PREDICTOBJECTTRAJECTORY Predicts future object trajectory.
%
% State:
%   [x y vx vy]
%
% Motion model:
%   Constant velocity
%
% Inputs:
%   track
%       Current tracked object.
%
%   predictionTime
%       How far into the future to predict [s].
%
%   timeStep
%       Prediction time step [s].
%
% Output:
%   predictedTrajectory
%       Structure containing future positions and velocities.
%
% Coordinate frame:
%   vehicle frame
%
%   x = forward
%   y = lateral

    %--------------------------------------------------------------
    % Number of prediction points
    %--------------------------------------------------------------

    numberOfSteps = ...
        floor(predictionTime / timeStep) + 1;

    %--------------------------------------------------------------
    % Allocate
    %--------------------------------------------------------------

    predictedTrajectory.time = ...
        zeros(1, numberOfSteps);

    predictedTrajectory.position = ...
        zeros(numberOfSteps, 2);

    predictedTrajectory.velocity = ...
        zeros(numberOfSteps, 2);

    %--------------------------------------------------------------
    % Initial state
    %--------------------------------------------------------------

    initialPosition = ...
        track.position;

    initialVelocity = ...
        track.velocity;

    %--------------------------------------------------------------
    % Generate trajectory
    %--------------------------------------------------------------

    for k = 1:numberOfSteps

        t = (k - 1) * timeStep;

        % Constant velocity prediction
        position = ...
            initialPosition + ...
            initialVelocity * t;

        % Store
        predictedTrajectory.time(k) = t;

        predictedTrajectory.position(k,:) = ...
            position;

        predictedTrajectory.velocity(k,:) = ...
            initialVelocity;

    end

end