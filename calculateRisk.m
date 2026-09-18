function riskState = calculateRisk(egoState, track)

%CALCULATERISK Calculates collision-risk information for a tracked object.
%
% Inputs:
%   egoState - Ego vehicle state
%   track    - Tracked object state
%
% Output:
%   riskState - Standardized risk information

    % ---------------------------------------------------------
    % Calculate TTC and closing speed
    % ---------------------------------------------------------

    [ttc, closingSpeed, relativeDistance] = ...
        calculateTTC(egoState, track);


    % ---------------------------------------------------------
    % Calculate minimum clearance
    % ---------------------------------------------------------

    clearance = calculateMinimumClearance(egoState, track);


    % ---------------------------------------------------------
    % Determine risk level
    % ---------------------------------------------------------

    if ttc <= 2.0 || clearance <= 5.0

        riskLevel = "HIGH";

    elseif ttc <= 5.0 || clearance <= 10.0

        riskLevel = "MEDIUM";

    else

        riskLevel = "LOW";

    end


    % ---------------------------------------------------------
    % Reduce risk when object is not approaching
    % ---------------------------------------------------------

    if isinf(ttc)

        riskLevel = "LOW";

    end


    % ---------------------------------------------------------
    % Create RiskState
    % ---------------------------------------------------------

    riskState.timestamp = track.timestamp;

    riskState.trackId = track.id;

    riskState.objectClass = track.class;

    riskState.ttc = ttc;

    riskState.closingSpeed = closingSpeed;

    riskState.relativeDistance = relativeDistance;

    riskState.minimumClearance = clearance;

    riskState.trackConfidence = track.confidence;

    riskState.riskLevel = riskLevel;

    riskState.frame = track.frame;

end