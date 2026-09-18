function [isValid, timeDifference] = validateTimestamp( ...
    timestamp1, timestamp2, tolerance)

%VALIDATETIMESTAMP Checks whether two sensor timestamps
% are close enough for sensor fusion.
%
% Inputs:
%   timestamp1 - First sensor timestamp [s]
%   timestamp2 - Second sensor timestamp [s]
%   tolerance  - Maximum allowed difference [s]
%
% Outputs:
%   isValid        - true if timestamps are synchronized
%   timeDifference - Absolute timestamp difference [s]

    timeDifference = abs(timestamp1 - timestamp2);

    isValid = timeDifference <= tolerance;

end