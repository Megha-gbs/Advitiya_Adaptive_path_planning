function [associations, unmatchedCamera, unmatchedLidar] = ...
    associateCameraLidar( ...
    cameraDetections, lidarDetections, ...
    timeTolerance, distanceThreshold)

%ASSOCIATECAMERALIDAR Uncertainty-aware camera-LiDAR association.
%
% This function associates camera and LiDAR detections using:
%
%   1. Timestamp compatibility
%   2. Object-class compatibility
%   3. Position-distance gating
%   4. Mahalanobis gating
%   5. Detection confidence
%   6. One-to-one matching
%
% The function assumes that camera and LiDAR detections have
% already been transformed into the same coordinate frame.
%
% Position:
%
%   [x y] [m]
%
% Coordinate convention:
%
%   x = forward
%   y = lateral
%
% Inputs:
%
%   cameraDetections
%       Camera detection structure array.
%
%   lidarDetections
%       LiDAR detection structure array.
%
%   timeTolerance
%       Maximum timestamp difference [s].
%
%   distanceThreshold
%       Maximum Euclidean position difference [m].
%
% Outputs:
%
%   associations
%       Matched camera-LiDAR pairs.
%
%   unmatchedCamera
%       Indices of unmatched camera detections.
%
%   unmatchedLidar
%       Indices of unmatched LiDAR detections.

    %--------------------------------------------------------------
    % Configuration
    %--------------------------------------------------------------

    % Camera position measurement uncertainty [m^2].
    cameraVariance = 1.0;

    % LiDAR position measurement uncertainty [m^2].
    lidarVariance = 0.09;

    % Combined measurement covariance.
    measurementVariance = ...
        cameraVariance + lidarVariance;

    measurementNoise = ...
        measurementVariance * eye(2);

    % Mahalanobis gate for 2-D position.
    mahalanobisThreshold = 9.21;

    %--------------------------------------------------------------
    % Initialize outputs
    %--------------------------------------------------------------

    associations = [];

    unmatchedCamera = [];

    unmatchedLidar = [];

    %--------------------------------------------------------------
    % Handle empty inputs
    %--------------------------------------------------------------

    if isempty(cameraDetections)

        unmatchedLidar = ...
            1:length(lidarDetections);

        return;

    end

    if isempty(lidarDetections)

        unmatchedCamera = ...
            1:length(cameraDetections);

        return;

    end

    %--------------------------------------------------------------
    % Track which LiDAR detections have already been used
    %--------------------------------------------------------------

    lidarUsed = ...
        false(1, length(lidarDetections));

    %--------------------------------------------------------------
    % Process each camera detection
    %--------------------------------------------------------------

    for i = 1:length(cameraDetections)

        bestLidarIndex = 0;

        bestAssociationScore = inf;

        bestDistance = inf;

        bestTimeDifference = inf;

        bestMahalanobisDistance = inf;

        %----------------------------------------------------------
        % Compare against every LiDAR detection
        %----------------------------------------------------------

        for j = 1:length(lidarDetections)

            %------------------------------------------------------
            % Prevent multiple camera detections from using
            % the same LiDAR detection.
            %------------------------------------------------------

            if lidarUsed(j)

                continue;

            end

            %------------------------------------------------------
            % Timestamp gating
            %------------------------------------------------------

            timeDifference = ...
                abs( ...
                cameraDetections(i).timestamp - ...
                lidarDetections(j).timestamp);

            if timeDifference > timeTolerance

                continue;

            end

            %------------------------------------------------------
            % Class compatibility
            %------------------------------------------------------

            cameraClass = ...
                string(cameraDetections(i).class);

            lidarClass = ...
                string(lidarDetections(j).class);

            if cameraClass ~= lidarClass

                continue;

            end

            %------------------------------------------------------
            % Frame compatibility
            %
            % The upgraded function does not silently transform
            % frames. Both detections must already be expressed
            % in the same frame.
            %------------------------------------------------------

            cameraFrame = ...
                string(cameraDetections(i).frame);

            lidarFrame = ...
                string(lidarDetections(j).frame);

            if cameraFrame ~= lidarFrame

                continue;

            end

            %------------------------------------------------------
            % Position difference
            %------------------------------------------------------

            positionDifference = ...
                cameraDetections(i).position(:) - ...
                lidarDetections(j).position(:);

            euclideanDistance = ...
                norm(positionDifference);

            %------------------------------------------------------
            % Simple distance gate
            %------------------------------------------------------

            if euclideanDistance > distanceThreshold

                continue;

            end

            %------------------------------------------------------
            % Mahalanobis distance
            %
            % Here the innovation is simply the difference between
            % camera and LiDAR position.
            %------------------------------------------------------

            distanceSquared = ...
                positionDifference' * ...
                (measurementNoise \ positionDifference);

            %------------------------------------------------------
            % Mahalanobis gate
            %------------------------------------------------------

            if distanceSquared > mahalanobisThreshold

                continue;

            end

            %------------------------------------------------------
            % Confidence
            %------------------------------------------------------

            cameraConfidence = ...
                cameraDetections(i).confidence;

            lidarConfidence = ...
                lidarDetections(j).confidence;

            combinedConfidence = ...
                0.5 * cameraConfidence + ...
                0.5 * lidarConfidence;

            %------------------------------------------------------
            % Normalize distance
            %------------------------------------------------------

            normalizedDistance = ...
                euclideanDistance / ...
                max(distanceThreshold, eps);

            %------------------------------------------------------
            % Normalize timestamp difference
            %------------------------------------------------------

            normalizedTime = ...
                timeDifference / ...
                max(timeTolerance, eps);

            %------------------------------------------------------
            % Association cost
            %
            % Lower is better.
            %
            % Position agreement is weighted strongly.
            % Confidence improves the association.
            %------------------------------------------------------

            associationScore = ...
                0.55 * normalizedDistance + ...
                0.20 * normalizedTime + ...
                0.15 * (distanceSquared / ...
                mahalanobisThreshold) + ...
                0.10 * (1 - combinedConfidence);

            %------------------------------------------------------
            % Select best LiDAR detection
            %------------------------------------------------------

            if associationScore < ...
                    bestAssociationScore

                bestAssociationScore = ...
                    associationScore;

                bestLidarIndex = j;

                bestDistance = ...
                    euclideanDistance;

                bestTimeDifference = ...
                    timeDifference;

                bestMahalanobisDistance = ...
                    sqrt(distanceSquared);

            end

        end

        %----------------------------------------------------------
        % Store successful association
        %----------------------------------------------------------

        if bestLidarIndex ~= 0

            match.cameraIndex = i;

            match.lidarIndex = ...
                bestLidarIndex;

            match.distance = ...
                bestDistance;

            match.timeDifference = ...
                bestTimeDifference;

            match.mahalanobisDistance = ...
                bestMahalanobisDistance;

            match.associationScore = ...
                bestAssociationScore;

            match.cameraConfidence = ...
                cameraDetections(i).confidence;

            match.lidarConfidence = ...
                lidarDetections(bestLidarIndex).confidence;

            match.fusedConfidence = ...
                0.5 * ...
                (cameraDetections(i).confidence + ...
                lidarDetections(bestLidarIndex).confidence);

            match.class = ...
                cameraDetections(i).class;

            match.frame = ...
                cameraDetections(i).frame;

            %------------------------------------------------------
            % Store association
            %------------------------------------------------------

            if isempty(associations)

                associations = match;

            else

                associations(end + 1) = match;

            end

            %------------------------------------------------------
            % Mark LiDAR measurement as used
            %------------------------------------------------------

            lidarUsed(bestLidarIndex) = true;

        else

            %------------------------------------------------------
            % No compatible LiDAR measurement
            %------------------------------------------------------

            unmatchedCamera(end + 1) = i;

        end

    end

    %--------------------------------------------------------------
    % Find unmatched LiDAR detections
    %--------------------------------------------------------------

    for j = 1:length(lidarDetections)

        if ~lidarUsed(j)

            unmatchedLidar(end + 1) = j;

        end

    end

end