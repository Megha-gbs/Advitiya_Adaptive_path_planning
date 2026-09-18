% CREATE_VEHICLE_FEEDBACK - Generates telemetry and feasibility feedback to Yashwanth/Sireesha
% SIH Problem Statement 26037: Vehicle Dynamics & Control

function fb = create_vehicle_feedback(timestamp, ego_x, ego_y, ego_yaw, ego_v, ...
                                     cross_track_err, heading_err, speed_err, ...
                                     lat_accel, slip_angle_f, slip_angle_r, ...
                                     feasibility_code, message)
    fb = struct();
    fb.timestamp        = timestamp;
    fb.actual_pose      = [ego_x, ego_y, ego_yaw];
    fb.actual_velocity  = ego_v;
    fb.cross_track_err  = cross_track_err;
    fb.heading_err      = heading_err;
    fb.speed_err        = speed_err;
    fb.lat_accel        = lat_accel;
    fb.slip_angle_f     = slip_angle_f;
    fb.slip_angle_r     = slip_angle_r;
    fb.feasibility_code = feasibility_code; % 0: OK, 1: CURVATURE_EXCESS, 2: SLIP_LIMIT, 3: SATURATION, 4: REJECT_FALLBACK
    fb.message          = message;
end
