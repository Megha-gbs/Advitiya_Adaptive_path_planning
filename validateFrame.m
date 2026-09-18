function [isValid, transformedPosition, message] = ...
    validateFrame(position, sourceFrame, targetFrame, transform)

%VALIDATEFRAME Validates and transforms a 2-D position between frames.
%
% This function performs two operations:
%
%   1. Checks whether a transformation between the source and
%      target coordinate frames is available.
%
%   2. Transforms the supplied position into the target frame.
%
% Position format:
%
%   [x y]       [m]
%
% Coordinate convention:
%
%   x = forward
%   y = lateral
%
% Inputs:
%
%   position
%       Position in source frame [x y].
%
%   sourceFrame
%       Name of source coordinate frame.
%
%   targetFrame
%       Name of target coordinate frame.
%
%   transform
%       Transformation structure.
%
%       Required fields:
%
%           transform.R
%           2x2 rotation matrix
%
%           transform.translation
%           1x2 translation vector [m]
%
% Output:
%
%   isValid
%       True if the frame transformation is valid.
%
%   transformedPosition
%       Position expressed in target frame.
%
%   message
%       Description of the operation.
%
% Transformation:
%
%   p_target = R * p_source + t
%
% where:
%
%   R = rotation matrix
%   t = translation vector

    %--------------------------------------------------------------
    % Convert frame names to strings
    %--------------------------------------------------------------

    sourceFrame = string(sourceFrame);
    targetFrame = string(targetFrame);

    %--------------------------------------------------------------
    % Initialize output
    %--------------------------------------------------------------

    isValid = false;

    transformedPosition = [NaN NaN];

    message = "";

    %--------------------------------------------------------------
    % Validate position
    %--------------------------------------------------------------

    if isempty(position) || length(position) ~= 2

        message = ...
            "Position must contain exactly two elements.";

        return;

    end

    position = ...
        position(:)';

    %--------------------------------------------------------------
    % Case 1:
    % Same coordinate frame
    %--------------------------------------------------------------

    if sourceFrame == targetFrame

        isValid = true;

        transformedPosition = ...
            position;

        message = ...
            "Source and target frames are identical.";

        return;

    end

    %--------------------------------------------------------------
    % Check transformation structure
    %--------------------------------------------------------------

    if isempty(transform)

        message = ...
            "Transformation is required for different frames.";

        return;

    end

    %--------------------------------------------------------------
    % Check rotation matrix
    %--------------------------------------------------------------

    if ~isfield(transform, "R")

        message = ...
            "Transformation does not contain rotation matrix R.";

        return;

    end

    R = transform.R;

    if ~isequal(size(R), [2 2])

        message = ...
            "Rotation matrix R must be 2x2.";

        return;

    end

    %--------------------------------------------------------------
    % Check translation
    %--------------------------------------------------------------

    if ~isfield(transform, "translation")

        message = ...
            "Transformation does not contain translation.";

        return;

    end

    translation = ...
        transform.translation(:)';

    if length(translation) ~= 2

        message = ...
            "Translation must contain exactly two elements.";

        return;

    end

    %--------------------------------------------------------------
    % Validate rotation matrix
    %--------------------------------------------------------------

    determinantR = det(R);

    orthogonalityError = ...
        norm(R' * R - eye(2), "fro");

    if abs(determinantR - 1) > 1e-6 || ...
            orthogonalityError > 1e-6

        message = ...
            "Invalid rotation matrix.";

        return;

    end

    %--------------------------------------------------------------
    % Transform position
    %--------------------------------------------------------------

    transformedPosition = ...
        (R * position(:))' + translation;

    %--------------------------------------------------------------
    % Successful transformation
    %--------------------------------------------------------------

    isValid = true;

    message = ...
        "Position successfully transformed from " + ...
        sourceFrame + ...
        " to " + ...
        targetFrame + ".";

end