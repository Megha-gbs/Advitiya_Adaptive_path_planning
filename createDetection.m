function detection = createDetection( ...
    timestamp, sensorType, sensorId, objectClass, ...
    position, velocity, confidence, frame)

%CREATEDETECTION Creates a standardized object detection.
%
% Inputs:
%   timestamp   - Detection timestamp [s]
%   sensorType  - camera / lidar / radar
%   sensorId    - Unique sensor identifier
%   objectClass - Object category
%   position    - Object position [m]
%   velocity    - Object velocity [m/s]
%   confidence  - Detection confidence [0,1]
%   frame       - Coordinate frame
%
% Output:
%   detection   - Standardized detection structure

    detection.timestamp   = timestamp;
    detection.sensorType  = string(sensorType);
    detection.sensorId    = string(sensorId);
    detection.class       = string(objectClass);

    detection.position    = position;
    detection.velocity    = velocity;

    detection.confidence  = confidence;
    detection.frame       = string(frame);

end