function tracks = updateTracks(tracks, detections)

    % ============================================================
    % UNCERTAINTY-AWARE MULTI-OBJECT TRACKER
    %
    % State:
    %   [x; y; vx; vy]
    %
    % Measurement:
    %   [x; y]
    %
    % Features:
    %   - Kalman prediction
    %   - Kalman measurement update
    %   - Uncertainty-aware Mahalanobis gating
    %   - Adaptive measurement noise
    %   - Fused sensor uncertainty support
    %   - Track confidence management
    %   - Track persistence through missed detections
    %   - Automatic creation of new tracks
    %   - Stale-track removal
    %   - Sensor information storage
    % ============================================================


    % ------------------------------------------------------------
    % CONFIGURATION
    % ------------------------------------------------------------

    defaultPositionVariance = 1.0;

    mahalanobisThreshold = 9.21;

    maxMisses = 5;

    confidenceDecay = 0.90;

    confidenceUpdateOld = 0.70;

    confidenceUpdateDetection = 0.30;


    % ------------------------------------------------------------
    % CASE 1:
    % No detections available
    % ------------------------------------------------------------

    if isempty(detections)

        for i = 1:length(tracks)

            dt = getTrackDeltaTime(tracks(i));

            [predictedState, predictedCovariance] = ...
                kalmanPredict( ...
                tracks(i).state, ...
                tracks(i).covariance, ...
                dt);


            tracks(i).state = predictedState;

            tracks(i).covariance = predictedCovariance;

            tracks(i).position = ...
                predictedState(1:2)';

            tracks(i).velocity = ...
                predictedState(3:4)';

            tracks(i).acceleration = [0 0];

            tracks(i).misses = ...
                tracks(i).misses + 1;

            tracks(i).age = ...
                tracks(i).age + 1;

            tracks(i).confidence = ...
                tracks(i).confidence * confidenceDecay;


            % Store propagated uncertainty

            tracks(i).positionVariance = ...
                trace(predictedCovariance(1:2,1:2)) / 2;

            tracks(i).velocityVariance = ...
                trace(predictedCovariance(3:4,3:4)) / 2;

            tracks(i).positionStd = ...
                sqrt(max(tracks(i).positionVariance, 0));

            tracks(i).velocityStd = ...
                sqrt(max(tracks(i).velocityVariance, 0));

        end


        tracks = ...
            removeStaleTracks(tracks, maxMisses);

        return;

    end


    % ------------------------------------------------------------
    % CASE 2:
    % No existing tracks
    % ------------------------------------------------------------

    if isempty(tracks)

        tracks = ...
            createTracksFromDetections(detections);

        return;

    end


    % ------------------------------------------------------------
    % PREDICT ALL EXISTING TRACKS
    % ------------------------------------------------------------

    predictedStates = ...
        cell(1, length(tracks));

    predictedCovariances = ...
        cell(1, length(tracks));


    for i = 1:length(tracks)

        dt = ...
            getTrackDeltaTime(tracks(i));


        [predictedState, predictedCovariance] = ...
            kalmanPredict( ...
            tracks(i).state, ...
            tracks(i).covariance, ...
            dt);


        predictedStates{i} = ...
            predictedState;

        predictedCovariances{i} = ...
            predictedCovariance;

    end


    % ------------------------------------------------------------
    % ASSOCIATION BOOKKEEPING
    % ------------------------------------------------------------

    detectionUsed = ...
        false(1, length(detections));

    trackUpdated = ...
        false(1, length(tracks));


    % ------------------------------------------------------------
    % TRACK-TO-DETECTION ASSOCIATION
    % ------------------------------------------------------------

    for i = 1:length(tracks)

        bestDetection = 0;

        bestDistance = inf;


        for j = 1:length(detections)

            if detectionUsed(j)

                continue;

            end


            % ----------------------------------------------------
            % CLASS CHECK
            % ----------------------------------------------------

            trackClass = ...
                string(tracks(i).class);

            detectionClass = ...
                string(detections(j).class);


            if trackClass ~= detectionClass

                continue;

            end


            % ----------------------------------------------------
            % GET POSITION VARIANCE
            % ----------------------------------------------------

            positionVariance = ...
                getPositionVariance( ...
                detections(j), ...
                defaultPositionVariance);


            % ----------------------------------------------------
            % ADAPTIVE MEASUREMENT NOISE
            % ----------------------------------------------------

            measurementNoise = ...
                positionVariance * eye(2);


            % ----------------------------------------------------
            % MAHALANOBIS GATING
            % ----------------------------------------------------

            [distanceSquared, isValid] = ...
                mahalanobisDistance( ...
                detections(j).position, ...
                predictedStates{i}, ...
                predictedCovariances{i}, ...
                measurementNoise, ...
                mahalanobisThreshold);


            if ~isValid

                continue;

            end


            % Convert squared distance to distance

            distance = ...
                sqrt(distanceSquared);


            % Select closest valid measurement

            if distance < bestDistance

                bestDistance = distance;

                bestDetection = j;

            end

        end


        % --------------------------------------------------------
        % VALID ASSOCIATION FOUND
        % --------------------------------------------------------

        if bestDetection ~= 0

            detection = ...
                detections(bestDetection);


            % ----------------------------------------------------
            % Detection uncertainty
            % ----------------------------------------------------

            positionVariance = ...
                getPositionVariance( ...
                detection, ...
                defaultPositionVariance);


            measurementNoise = ...
                positionVariance * eye(2);


            % ----------------------------------------------------
            % KALMAN UPDATE
            % ----------------------------------------------------

            [updatedState, updatedCovariance, ...
                innovation, innovationCovariance] = ...
                kalmanUpdate( ...
                predictedStates{i}, ...
                predictedCovariances{i}, ...
                detection.position, ...
                measurementNoise);


            % Save old velocity before update

            oldVelocity = ...
                tracks(i).velocity;


            % ----------------------------------------------------
            % UPDATE STATE
            % ----------------------------------------------------

            tracks(i).state = ...
                updatedState;

            tracks(i).covariance = ...
                updatedCovariance;

            tracks(i).position = ...
                updatedState(1:2)';

            tracks(i).velocity = ...
                updatedState(3:4)';


            % ----------------------------------------------------
            % ACCELERATION ESTIMATION
            % ----------------------------------------------------

            dt = ...
                getTrackDeltaTime(tracks(i));


            if dt > 0

                tracks(i).acceleration = ...
                    (tracks(i).velocity - oldVelocity) / dt;

            else

                tracks(i).acceleration = [0 0];

            end


            % ----------------------------------------------------
            % CONFIDENCE UPDATE
            % ----------------------------------------------------

            detectionConfidence = ...
                max(0, min(1, detection.confidence));


            tracks(i).confidence = ...
                confidenceUpdateOld * ...
                tracks(i).confidence + ...
                confidenceUpdateDetection * ...
                detectionConfidence;


            % ----------------------------------------------------
            % TRACK STATISTICS
            % ----------------------------------------------------

            tracks(i).hits = ...
                tracks(i).hits + 1;

            tracks(i).misses = 0;

            tracks(i).age = ...
                tracks(i).age + 1;


            % ----------------------------------------------------
            % TRACK UNCERTAINTY
            % ----------------------------------------------------

            tracks(i).positionVariance = ...
                trace(updatedCovariance(1:2,1:2)) / 2;

            tracks(i).velocityVariance = ...
                trace(updatedCovariance(3:4,3:4)) / 2;


            tracks(i).positionStd = ...
                sqrt(max( ...
                tracks(i).positionVariance, 0));


            tracks(i).velocityStd = ...
                sqrt(max( ...
                tracks(i).velocityVariance, 0));


            % ----------------------------------------------------
            % KALMAN INNOVATION
            % ----------------------------------------------------

            tracks(i).innovation = ...
                innovation;

            tracks(i).innovationCovariance = ...
                innovationCovariance;


            % ----------------------------------------------------
            % ASSOCIATION QUALITY
            % ----------------------------------------------------

            tracks(i).associationDistance = ...
                bestDistance;


            % ----------------------------------------------------
            % SENSOR INFORMATION
            % ----------------------------------------------------

            if isfield(detection, "sensorCount")

                tracks(i).sensorCount = ...
                    detection.sensorCount;

            else

                tracks(i).sensorCount = 1;

            end


            if isfield(detection, "sensors")

                tracks(i).sensors = ...
                    detection.sensors;

            else

                tracks(i).sensors = ...
                    string(detection.sensorType);

            end


            % ----------------------------------------------------
            % Mark detection as used
            % ----------------------------------------------------

            detectionUsed(bestDetection) = true;

            trackUpdated(i) = true;

        end

    end


    % ------------------------------------------------------------
    % HANDLE UNMATCHED TRACKS
    % ------------------------------------------------------------

    for i = 1:length(tracks)

        if ~trackUpdated(i)

            predictedState = ...
                predictedStates{i};

            predictedCovariance = ...
                predictedCovariances{i};


            tracks(i).state = ...
                predictedState;

            tracks(i).covariance = ...
                predictedCovariance;

            tracks(i).position = ...
                predictedState(1:2)';

            tracks(i).velocity = ...
                predictedState(3:4)';


            tracks(i).misses = ...
                tracks(i).misses + 1;

            tracks(i).age = ...
                tracks(i).age + 1;


            tracks(i).confidence = ...
                tracks(i).confidence * ...
                confidenceDecay;


            % Propagated uncertainty

            tracks(i).positionVariance = ...
                trace(predictedCovariance(1:2,1:2)) / 2;

            tracks(i).velocityVariance = ...
                trace(predictedCovariance(3:4,3:4)) / 2;


            tracks(i).positionStd = ...
                sqrt(max( ...
                tracks(i).positionVariance, 0));


            tracks(i).velocityStd = ...
                sqrt(max( ...
                tracks(i).velocityVariance, 0));

        end

    end


    % ------------------------------------------------------------
    % CREATE TRACKS FROM UNMATCHED DETECTIONS
    % ------------------------------------------------------------

    unmatchedDetections = ...
        find(~detectionUsed);


    for k = 1:length(unmatchedDetections)

        detectionIndex = ...
            unmatchedDetections(k);


        newTrack = ...
            createTrackFromDetection( ...
            detections(detectionIndex));


        if isempty(tracks)

            tracks = newTrack;

        else

            tracks(end + 1) = newTrack;

        end

    end


    % ------------------------------------------------------------
    % REMOVE STALE TRACKS
    % ------------------------------------------------------------

    tracks = ...
        removeStaleTracks( ...
        tracks, ...
        maxMisses);

end



% ================================================================
% HELPER FUNCTION
% Get time difference for a track
% ================================================================

function dt = getTrackDeltaTime(track)

    if isfield(track, "timestamp")

        if isfield(track, "lastTimestamp")

            dt = ...
                track.timestamp - ...
                track.lastTimestamp;

        else

            dt = 0.1;

        end

    else

        dt = 0.1;

    end


    if ~isfinite(dt) || dt <= 0

        dt = 0.1;

    end

end



% ================================================================
% HELPER FUNCTION
% Get detection position variance
% ================================================================

function variance = getPositionVariance( ...
    detection, defaultVariance)

    if isfield(detection, "positionVariance")

        variance = ...
            detection.positionVariance;

    else

        variance = ...
            defaultVariance;

    end


    if ~isfinite(variance) || variance <= 0

        variance = ...
            defaultVariance;

    end

end



% ================================================================
% HELPER FUNCTION
% Create tracks from detections
% ================================================================

function tracks = ...
    createTracksFromDetections(detections)

    % IMPORTANT:
    % Initialize as an empty STRUCT, not [].
    % This prevents MATLAB from treating tracks as a double array.

    tracks = struct([]);


    for i = 1:length(detections)

        newTrack = ...
            createTrackFromDetection( ...
            detections(i));


        if isempty(tracks)

            tracks = newTrack;

        else

            tracks(end + 1) = newTrack;

        end

    end

end



% ================================================================
% HELPER FUNCTION
% Create a single track
% ================================================================

function track = ...
    createTrackFromDetection(detection)


    % ------------------------------------------------------------
    % Initial state
    % ------------------------------------------------------------

    state = [ ...
        detection.position(1);
        detection.position(2);
        detection.velocity(1);
        detection.velocity(2)];


    % ------------------------------------------------------------
    % Initial position uncertainty
    % ------------------------------------------------------------

    positionVariance = ...
        getPositionVariance( ...
        detection, ...
        1.0);


    % ------------------------------------------------------------
    % Initial covariance
    %
    % [position x]
    % [position y]
    % [velocity x]
    % [velocity y]
    % ------------------------------------------------------------

    covariance = diag([ ...
        positionVariance;
        positionVariance;
        4.0;
        4.0]);


    % ------------------------------------------------------------
    % Basic track information
    % ------------------------------------------------------------

    track.id = ...
        randi(1000000);

    track.timestamp = ...
        detection.timestamp;

    track.lastTimestamp = ...
        detection.timestamp;

    track.class = ...
        string(detection.class);

    track.position = ...
        detection.position;

    track.velocity = ...
        detection.velocity;

    track.acceleration = ...
        [0 0];

    track.confidence = ...
        max(0, min(1, detection.confidence));

    track.frame = ...
        string(detection.frame);


    % ------------------------------------------------------------
    % Kalman state
    % ------------------------------------------------------------

    track.state = ...
        state;

    track.covariance = ...
        covariance;


    % ------------------------------------------------------------
    % Track statistics
    % ------------------------------------------------------------

    track.hits = 1;

    track.misses = 0;

    track.age = 1;


    % ------------------------------------------------------------
    % Uncertainty information
    % ------------------------------------------------------------

    track.positionVariance = ...
        positionVariance;

    track.velocityVariance = ...
        4.0;

    track.positionStd = ...
        sqrt(positionVariance);

    track.velocityStd = ...
        2.0;


    % ------------------------------------------------------------
    % Association information
    % ------------------------------------------------------------

    track.associationDistance = ...
        NaN;


    track.innovation = ...
        [NaN; NaN];


    track.innovationCovariance = ...
        NaN(2,2);


    % ------------------------------------------------------------
    % Sensor information
    % ------------------------------------------------------------

    if isfield(detection, "sensorCount")

        track.sensorCount = ...
            detection.sensorCount;

    else

        track.sensorCount = 1;

    end


    if isfield(detection, "sensors")

        track.sensors = ...
            detection.sensors;

    else

        track.sensors = ...
            string(detection.sensorType);

    end

end



% ================================================================
% HELPER FUNCTION
% Remove stale tracks
% ================================================================

function tracks = ...
    removeStaleTracks(tracks, maxMisses)


    if isempty(tracks)

        return;

    end


    keep = ...
        true(1, length(tracks));


    for i = 1:length(tracks)

        if tracks(i).misses > maxMisses

            keep(i) = false;

        end

    end


    tracks = ...
        tracks(keep);

end