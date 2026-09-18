% VALIDATE_INTERFACES - Verifies compliance of input structs from Hanish, Yashwanth, and Safety
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function [isValid, errMsg] = validate_interfaces(trajectory, egoState, safetyStatus)
    isValid = true;
    errMsg = '';

    %% 1. Validate Trajectory (from Yashwanth)
    % Minimum required fields: time, x, y, yaw, velocity, acceleration, curvature
    reqTrajFields = {'time', 'x', 'y', 'yaw', 'velocity', 'acceleration', 'curvature'};
    for i = 1:length(reqTrajFields)
        f = reqTrajFields{i};
        if ~isfield(trajectory, f)
            isValid = false;
            errMsg = sprintf('Trajectory missing required field: %s', f);
            return;
        end
    end
    N = length(trajectory.time);
    if length(trajectory.x) ~= N || length(trajectory.y) ~= N || ...
       length(trajectory.yaw) ~= N || length(trajectory.velocity) ~= N || ...
       length(trajectory.acceleration) ~= N || length(trajectory.curvature) ~= N
        isValid = false;
        errMsg = 'Trajectory dimension mismatch: arrays time, x, y, yaw, velocity, acceleration, curvature must be equal length.';
        return;
    end

    %% 2. Validate EgoState (from Hanish)
    % Minimum required fields: timestamp, pose ([x, y, z]), velocity ([vx, vy, vz]), heading (yaw), covariance
    reqEgoFields = {'timestamp', 'pose', 'velocity', 'heading', 'covariance'};
    for i = 1:length(reqEgoFields)
        f = reqEgoFields{i};
        if ~isfield(egoState, f)
            isValid = false;
            errMsg = sprintf('EgoState missing required field: %s', f);
            return;
        end
    end
    if length(egoState.pose) < 2
        isValid = false;
        errMsg = 'EgoState.pose must contain at least [x, y].';
        return;
    end

    %% 3. Validate SafetyStatus (from Safety Supervisor)
    % Minimum required fields: safe, TTC, clearance, reason, emergency_flag
    reqSafetyFields = {'safe', 'TTC', 'clearance', 'reason', 'emergency_flag'};
    for i = 1:length(reqSafetyFields)
        f = reqSafetyFields{i};
        if ~isfield(safetyStatus, f)
            isValid = false;
            errMsg = sprintf('SafetyStatus missing required field: %s', f);
            return;
        end
    end
end
