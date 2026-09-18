function audioEvent = createAudioEvent( ...
    timestamp, eventType, doa, confidence)

%CREATEAUDIOEVENT Creates a standardized audio event.
%
% Inputs:
%   timestamp  - Event timestamp [s]
%   eventType  - Type of audio event
%   doa        - Direction of arrival [rad]
%   confidence - Classification confidence [0,1]
%
% Output:
%   audioEvent - Standardized audio event structure

    audioEvent.timestamp  = timestamp;
    audioEvent.event      = string(eventType);
    audioEvent.doa        = doa;
    audioEvent.confidence = confidence;

end