function track = createTrackState( ...
    trackId, timestamp, objectClass, ...
    position, velocity, acceleration, ...
    confidence, covariance, frame)

%CREATETRACKSTATE Creates a standardized tracked-object state.
%
% Inputs:
%   trackId       - Unique persistent track ID
%   timestamp     - Current time [s]
%   objectClass   - Object class
%   position      - Object position [m]
%   velocity      - Object velocity [m/s]
%   acceleration  - Object acceleration [m/s^2]
%   confidence    - Track confidence [0,1]
%   covariance    - State uncertainty/covariance
%   frame         - Coordinate frame
%
% Output:
%   track         - Standardized track state structure

    track.id = trackId;

    track.timestamp = timestamp;

    track.class = string(objectClass);

    track.position = position;

    track.velocity = velocity;

    track.acceleration = acceleration;

    track.confidence = confidence;

    track.covariance = covariance;

    track.frame = string(frame);

end