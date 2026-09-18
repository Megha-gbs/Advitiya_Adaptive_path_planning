function fusedDetection = fuseDetections(detections)

%FUSEDETECTIONS Uncertainty- and confidence-weighted sensor fusion.
%
% Supported sensors:
%
%   camera
%   lidar
%   radar
%
% Each detection must contain:
%
%   timestamp
%   sensorType
%   sensorId
%   class
%   position
%   velocity
%   confidence
%   frame
%
% The function combines measurements from multiple sensors
% using confidence- and sensor-dependent weights.
%
% Position and velocity are fused independently.
%
% Coordinate convention:
%
%   x = forward
%   y = lateral
%
% IMPORTANT:
%
% All detections supplied to this function must already be
% expressed in the same coordinate frame.

    %--------------------------------------------------------------
    % Validate input
    %--------------------------------------------------------------

    if isempty(detections)

        fusedDetection = [];

        return;

    end

    %--------------------------------------------------------------
    % Sensor uncertainty configuration
    %
    % These are initial simulation values.
    % They should later be replaced by values obtained from
    % your actual sensor specifications/calibration.
    %--------------------------------------------------------------

    positionVariance.camera = 1.00;
    positionVariance.lidar  = 0.09;
    positionVariance.radar  = 0.25;

    velocityVariance.camera = 2.25;
    velocityVariance.lidar  = 0.64;
    velocityVariance.radar  = 0.25;

    %--------------------------------------------------------------
    % Select reference information
    %--------------------------------------------------------------

    referenceClass = ...
        string(detections(1).class);

    referenceFrame = ...
        string(detections(1).frame);

    referenceTimestamp = ...
        detections(1).timestamp;

    %--------------------------------------------------------------
    % Keep only compatible detections
    %--------------------------------------------------------------

    validIndex = true(1, length(detections));

    for i = 1:length(detections)

        currentClass = ...
            string(detections(i).class);

        currentFrame = ...
            string(detections(i).frame);

        if currentClass ~= referenceClass

            validIndex(i) = false;

        elseif currentFrame ~= referenceFrame

            validIndex(i) = false;

        end

    end

    detections = ...
        detections(validIndex);

    %--------------------------------------------------------------
    % If no compatible detections remain
    %--------------------------------------------------------------

    if isempty(detections)

        fusedDetection = [];

        return;

    end

    %--------------------------------------------------------------
    % Initialize weighted sums
    %--------------------------------------------------------------

    positionWeightedSum = [0 0];

    velocityWeightedSum = [0 0];

    totalPositionWeight = 0;

    totalVelocityWeight = 0;

    confidenceWeightedSum = 0;

    totalConfidenceWeight = 0;

    %--------------------------------------------------------------
    % Store sensor information
    %--------------------------------------------------------------

    sensorNames = strings(1, length(detections));

    sensorWeights = zeros(1, length(detections));

    %--------------------------------------------------------------
    % Process each sensor measurement
    %--------------------------------------------------------------

    for i = 1:length(detections)

        sensorType = ...
            lower(string(detections(i).sensorType));

        confidence = ...
            max(0, min(1, detections(i).confidence));

        %----------------------------------------------------------
        % Determine sensor variance
        %----------------------------------------------------------

        switch sensorType

            case "camera"

                positionVar = ...
                    positionVariance.camera;

                velocityVar = ...
                    velocityVariance.camera;

            case "lidar"

                positionVar = ...
                    positionVariance.lidar;

                velocityVar = ...
                    velocityVariance.lidar;

            case "radar"

                positionVar = ...
                    positionVariance.radar;

                velocityVar = ...
                    velocityVariance.radar;

            otherwise

                % Unknown sensor.
                %
                % Give it conservative uncertainty.

                positionVar = 4.0;

                velocityVar = 4.0;

        end

        %----------------------------------------------------------
        % Confidence-adjusted precision
        %
        % Higher confidence + lower variance = larger weight.
        %----------------------------------------------------------

        positionWeight = ...
            confidence / ...
            max(positionVar, eps);

        velocityWeight = ...
            confidence / ...
            max(velocityVar, eps);

        %----------------------------------------------------------
        % Position fusion
        %----------------------------------------------------------

        position = ...
            detections(i).position(:)';

        positionWeightedSum = ...
            positionWeightedSum + ...
            positionWeight * position;

        totalPositionWeight = ...
            totalPositionWeight + ...
            positionWeight;

        %----------------------------------------------------------
        % Velocity fusion
        %----------------------------------------------------------

        velocity = ...
            detections(i).velocity(:)';

        velocityWeightedSum = ...
            velocityWeightedSum + ...
            velocityWeight * velocity;

        totalVelocityWeight = ...
            totalVelocityWeight + ...
            velocityWeight;

        %----------------------------------------------------------
        % Confidence fusion
        %----------------------------------------------------------

        confidenceWeightedSum = ...
            confidenceWeightedSum + ...
            confidence * positionWeight;

        totalConfidenceWeight = ...
            totalConfidenceWeight + ...
            positionWeight;

        %----------------------------------------------------------
        % Store sensor information
        %----------------------------------------------------------

        sensorNames(i) = ...
            sensorType;

        sensorWeights(i) = ...
            positionWeight;

    end

    %--------------------------------------------------------------
    % Calculate fused position
    %--------------------------------------------------------------

    if totalPositionWeight > 0

        fusedPosition = ...
            positionWeightedSum / ...
            totalPositionWeight;

    else

        fusedPosition = ...
            detections(1).position;

    end

    %--------------------------------------------------------------
    % Calculate fused velocity
    %--------------------------------------------------------------

    if totalVelocityWeight > 0

        fusedVelocity = ...
            velocityWeightedSum / ...
            totalVelocityWeight;

    else

        fusedVelocity = ...
            detections(1).velocity;

    end

    %--------------------------------------------------------------
    % Calculate fused confidence
    %--------------------------------------------------------------

    if totalConfidenceWeight > 0

        fusedConfidence = ...
            confidenceWeightedSum / ...
            totalConfidenceWeight;

    else

        fusedConfidence = ...
            detections(1).confidence;

    end

    %--------------------------------------------------------------
    % Calculate fused uncertainty
    %
    % For independent measurements, combined precision is:
    %
    %   precision = sum(weights)
    %
    % Therefore:
    %
    %   variance = 1 / sum(weights)
    %
    %--------------------------------------------------------------

    fusedPositionVariance = ...
        1 / max(totalPositionWeight, eps);

    fusedVelocityVariance = ...
        1 / max(totalVelocityWeight, eps);

    %--------------------------------------------------------------
    % Sensor count
    %--------------------------------------------------------------

    uniqueSensors = ...
        unique(sensorNames);

    sensorCount = ...
        length(uniqueSensors);

    %--------------------------------------------------------------
    % Build output
    %--------------------------------------------------------------

    fusedDetection.timestamp = ...
        referenceTimestamp;

    fusedDetection.sensorType = ...
        "fused";

    fusedDetection.sensorId = ...
        "camera_lidar_radar";

    fusedDetection.class = ...
        referenceClass;

    fusedDetection.position = ...
        fusedPosition;

    fusedDetection.velocity = ...
        fusedVelocity;

    fusedDetection.confidence = ...
        fusedConfidence;

    fusedDetection.frame = ...
        referenceFrame;

    fusedDetection.sensorCount = ...
        sensorCount;

    fusedDetection.sensors = ...
        uniqueSensors;

    fusedDetection.sensorWeights = ...
        sensorWeights;

    fusedDetection.positionVariance = ...
        fusedPositionVariance;

    fusedDetection.velocityVariance = ...
        fusedVelocityVariance;

    fusedDetection.positionStd = ...
        sqrt(fusedPositionVariance);

    fusedDetection.velocityStd = ...
        sqrt(fusedVelocityVariance);

end