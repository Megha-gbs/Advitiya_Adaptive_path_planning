function clearance = calculateMinimumClearance(egoState, track)

%CALCULATEMINIMUMCLEARANCE Calculates distance between
% ego vehicle and tracked object.
%
% Inputs:
%   egoState - Ego vehicle state
%   track    - Tracked object state
%
% Output:
%   clearance - Euclidean clearance distance [m]

    % Relative position
    relativePosition = ...
        track.position - egoState.position;

    % Euclidean distance
    clearance = norm(relativePosition);

end