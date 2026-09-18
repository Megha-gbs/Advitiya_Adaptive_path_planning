function [tracks, audioAssociations] = ...
    associateAudioTracks(tracks, audioEvents, egoState, ...
    timeTolerance, angleThreshold)

%ASSOCIATEAUDIOTRACKS Associates audio events with tracked objects.
%
% Audio is treated as contextual evidence.
%
% Pipeline:
%
%   Audio Event
%       |
%       v
%   Timestamp Check
%       |
%       v
%   Track Bearing Calculation
%       |
%       v
%   DoA Comparison
%       |
%       v
%   Audio-Track Association
%       |
%       v
%   Audio Confidence / Risk Cue
%
% Inputs:
%
%   tracks
%       Current tracked objects.
%
%   audioEvents
%       Structure array created using createAudioEvent().
%
%   egoState
%       Current ego vehicle state.
%
%   timeTolerance
%       Maximum allowed timestamp difference [s].
%
%   angleThreshold
%       Maximum angular difference for association [rad].
%
% Outputs:
%
%   tracks
%       Updated tracks containing audio information.
%
%   audioAssociations
%       Associations between audio events and tracks.
%
% Coordinate convention:
%
%   Track positions are assumed to be in the vehicle frame:
%
%       x = forward
%       y = lateral
%
%   Audio DoA convention:
%
%       0 rad = forward
%       positive angle = positive vehicle-y direction
%
% IMPORTANT:
%
% Audio alone does NOT create an object track.
% It only provides additional evidence for an existing track.

    %--------------------------------------------------------------
    % Initialize
    %--------------------------------------------------------------

    audioAssociations = [];

    %--------------------------------------------------------------
    % If there are no tracks or audio events
    %--------------------------------------------------------------

    if isempty(tracks) || isempty(audioEvents)

        return;

    end

    %--------------------------------------------------------------
    % Add audio fields to tracks if they do not already exist
    %--------------------------------------------------------------

    for i = 1:length(tracks)

        tracks(i).audioEvent = "";
        tracks(i).audioConfidence = 0;
        tracks(i).audioDoA = NaN;
        tracks(i).audioAngleError = NaN;
        tracks(i).audioAssociationConfidence = 0;
        tracks(i).audioTimestamp = NaN;
        tracks(i).audioRiskCue = 0;

    end

    %--------------------------------------------------------------
    % Process every audio event
    %--------------------------------------------------------------

    for a = 1:length(audioEvents)

        audioTimestamp = audioEvents(a).timestamp;

        audioEventType = ...
            string(audioEvents(a).event);

        audioDoA = ...
            audioEvents(a).doa;

        audioConfidence = ...
            audioEvents(a).confidence;

        %----------------------------------------------------------
        % Ignore invalid audio confidence
        %----------------------------------------------------------

        if audioConfidence <= 0

            continue;

        end

        %----------------------------------------------------------
        % Find the best matching track
        %----------------------------------------------------------

        bestTrack = 0;

        bestScore = 0;

        bestAngleError = inf;

        bestBearing = NaN;

        %----------------------------------------------------------
        % Compare audio event with every track
        %----------------------------------------------------------

        for t = 1:length(tracks)

            %------------------------------------------------------
            % Timestamp compatibility
            %------------------------------------------------------

            timeDifference = ...
                abs(tracks(t).timestamp - audioTimestamp);

            if timeDifference > timeTolerance

                continue;

            end

            %------------------------------------------------------
            % Relative position of track from ego vehicle
            %------------------------------------------------------

            relativePosition = ...
                tracks(t).position - egoState.position;

            %------------------------------------------------------
            % Bearing of object relative to ego vehicle
            %
            % x = forward
            % y = lateral
            %------------------------------------------------------

            bearing = atan2( ...
                relativePosition(2), ...
                relativePosition(1));

            %------------------------------------------------------
            % Audio DoA and object bearing difference
            %------------------------------------------------------

            angleDifference = ...
                wrapToPi(audioDoA - bearing);

            angleError = ...
                abs(angleDifference);

            %------------------------------------------------------
            % Angular gating
            %------------------------------------------------------

            if angleError > angleThreshold

                continue;

            end

            %------------------------------------------------------
            % Angular consistency score
            %
            % 1.0 = perfect angular agreement
            % 0.0 = poor agreement
            %------------------------------------------------------

            angularScore = ...
                exp(-0.5 * ...
                (angleError / angleThreshold)^2);

            %------------------------------------------------------
            % Timestamp score
            %------------------------------------------------------

            if timeTolerance > 0

                timestampScore = ...
                    exp(-0.5 * ...
                    (timeDifference / timeTolerance)^2);

            else

                timestampScore = 1;

            end

            %------------------------------------------------------
            % Track confidence
            %------------------------------------------------------

            trackConfidence = ...
                tracks(t).confidence;

            %------------------------------------------------------
            % Combined audio-track association confidence
            %------------------------------------------------------

            associationConfidence = ...
                audioConfidence * ...
                angularScore * ...
                timestampScore * ...
                trackConfidence;

            %------------------------------------------------------
            % Keep best association
            %------------------------------------------------------

            if associationConfidence > bestScore

                bestScore = ...
                    associationConfidence;

                bestTrack = t;

                bestAngleError = ...
                    angleError;

                bestBearing = ...
                    bearing;

            end

        end

        %----------------------------------------------------------
        % Store successful association
        %----------------------------------------------------------

        if bestTrack ~= 0

            %------------------------------------------------------
            % Association information
            %------------------------------------------------------

            association.audioIndex = a;

            association.trackIndex = bestTrack;

            association.trackId = ...
                tracks(bestTrack).id;

            association.event = ...
                audioEventType;

            association.audioDoA = ...
                audioDoA;

            association.trackBearing = ...
                bestBearing;

            association.angleError = ...
                bestAngleError;

            association.audioConfidence = ...
                audioConfidence;

            association.associationConfidence = ...
                bestScore;

            association.timestamp = ...
                audioTimestamp;

            %------------------------------------------------------
            % Add to output
            %------------------------------------------------------

            if isempty(audioAssociations)

                audioAssociations = ...
                    association;

            else

                audioAssociations(end + 1) = ...
                    association;

            end

            %------------------------------------------------------
            % Update track with audio evidence
            %------------------------------------------------------

            tracks(bestTrack).audioEvent = ...
                audioEventType;

            tracks(bestTrack).audioConfidence = ...
                audioConfidence;

            tracks(bestTrack).audioDoA = ...
                audioDoA;

            tracks(bestTrack).audioAngleError = ...
                bestAngleError;

            tracks(bestTrack).audioAssociationConfidence = ...
                bestScore;

            tracks(bestTrack).audioTimestamp = ...
                audioTimestamp;

            %------------------------------------------------------
            % Audio risk cue
            %
            % Strong audio association gives an additional cue.
            %
            % It does NOT directly declare the object dangerous.
            % Geometry and motion remain necessary.
            %------------------------------------------------------

            tracks(bestTrack).audioRiskCue = ...
                bestScore;

        end

    end

end