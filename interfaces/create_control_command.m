% CREATE_CONTROL_COMMAND - Packs actuator commands to Simulator (Sireesha)
% SIH Problem Statement 26037: Vehicle Dynamics & Control
% Output: ControlCommand struct
%   timestamp : current simulation time (s)
%   steering  : front wheel steering angle (rad)
%   throttle  : normalized throttle command [0.0, 1.0]
%   brake     : normalized brake command [0.0, 1.0]
%   accel_cmd : raw demanded acceleration (m/s^2) for diagnostic verification

function cmd = create_control_command(timestamp, steering, throttle, brake, accel_cmd)
    cmd = struct();
    cmd.timestamp = timestamp;
    cmd.steering  = max(min(steering, deg2rad(32)), -deg2rad(32)); % safety clamp
    cmd.throttle  = max(0.0, min(1.0, throttle));
    cmd.brake     = max(0.0, min(1.0, brake));
    if nargin >= 5
        cmd.accel_cmd = accel_cmd;
    else
        cmd.accel_cmd = 0.0;
    end
end
