%% Occupancy Map from SLAM-Estimated Poses — Hanish Localization
% Status: Estimated-pose map construction demonstration
% Depends on: lidar_slam_result.mat (produced by lidar_slam_demo.m, the
% frozen v3 LiDAR + gyro baseline)
% Output: occupancy map + comparison against ground-truth map
%
% This addresses the code review's point that the original
% occupancy_map_demo.m used ground-truth poses. This version uses
% the LiDAR odometry's own pose estimates instead.
%
% CAVEAT — this is NOT an independent end-to-end validation: the LiDAR
% ranges used here are RE-SIMULATED from slamPoses (section 2, ray-cast
% against the known obstacle map at each estimated pose), not an
% independently captured sensor stream tied to the original trajectory.
% So this demonstrates map construction from estimated poses, but it
% does not independently validate the SLAM pose estimates themselves
% against a separately-generated sensor stream.

clear; clc; close all;

%% 1. Load SLAM result
if ~isfile('lidar_slam_result.mat')
    error(['lidar_slam_result.mat not found. Run lidar_slam_demo.m ' ...
           'first to produce the estimated poses.']);
end
load('lidar_slam_result.mat');   % contains slamMap, slamPoses, truePose, trueYaw

fprintf('Loaded SLAM result: %d estimated poses\n', size(slamPoses,1));

%% 2. Environment (same as other scripts)
mapWidth   = 20;
mapHeight  = 20;
resolution = 0.1;
numBeams   = 360;
maxRange   = 10;

obstacles = [
     2  2   4   6;
     2  2   8   3;
     6  2   7   8;
    10  5  14   6;
    12 10  16  14;
     5 12   9  15;
    16  3  18   5;
];

% Re-simulate the LiDAR scans (need them for the map build)
angles = linspace(-pi, pi, numBeams)';
numSamples = size(slamPoses,1);
ranges = zeros(numSamples, numBeams);

for k = 1:numSamples
    pose = slamPoses(k,1:2);
    yaw  = slamPoses(k,3);

    for b = 1:numBeams
        rayAngle = yaw + angles(b);
        dx = cos(rayAngle);
        dy = sin(rayAngle);

        r = maxRange;
        for o = 1:size(obstacles,1)
            xmin = obstacles(o,1); ymin = obstacles(o,2);
            xmax = obstacles(o,3); ymax = obstacles(o,4);
            tHit = rayBoxIntersect(pose(1), pose(2), dx, dy, ...
                                    xmin, ymin, xmax, ymax);
            if ~isnan(tHit) && tHit < r
                r = tHit;
            end
        end

        rNoisy = r + 0.02*randn;
        if ~isfinite(rNoisy) || rNoisy < 0
            rNoisy = maxRange;
        end
        ranges(k,b) = min(maxRange, rNoisy);
    end
end

%% 3. Build map from ESTIMATED poses (this is the whole point)
fprintf('Building map from ESTIMATED SLAM poses...\n');

map_est = occupancyMap(mapWidth, mapHeight, 1/resolution);
map_est.GridLocationInWorld = [0 0];
occMat_est = 0.5 * ones(map_est.GridSize);
stepSize = resolution;

for k = 1:numSamples
    px  = slamPoses(k,1);
    py  = slamPoses(k,2);
    yaw = slamPoses(k,3);

    for b = 1:numBeams
        r = ranges(k,b);
        if r <= 0 || ~isfinite(r), continue; end

        rayAngle = yaw + angles(b);
        numSteps = floor(r / stepSize);

        for i = 1:numSteps
            cx = px + i*stepSize*cos(rayAngle);
            cy = py + i*stepSize*sin(rayAngle);
            col = round(cx * map_est.Resolution) + 1;
            row = round(cy * map_est.Resolution) + 1;

            if col < 1 || col > map_est.GridSize(2) || ...
               row < 1 || row > map_est.GridSize(1)
                break;
            end
            occMat_est(row, col) = max(0, occMat_est(row, col) - 0.05);
        end

        ex = px + r*cos(rayAngle);
        ey = py + r*sin(rayAngle);
        col = round(ex * map_est.Resolution) + 1;
        row = round(ey * map_est.Resolution) + 1;

        if col >= 1 && col <= map_est.GridSize(2) && ...
           row >= 1 && row <= map_est.GridSize(1)
            occMat_est(row, col) = min(1, occMat_est(row, col) + 0.4);
        end
    end
end

map_est.setOccupancy(occMat_est);

%% 4. Compare ground-truth map vs estimated-pose map
figure('Name','Map from SLAM poses vs ground truth','Color','w', ...
    'Position',[100 100 1400 600]);

subplot(1,2,1);
show(map_est);
hold on;
plot(slamPoses(:,1), slamPoses(:,2), 'r-', 'LineWidth', 1.5);
title('Map built from ESTIMATED SLAM poses');
xlabel('X (m)'); ylabel('Y (m)');

subplot(1,2,2);
show(map_est);
hold on;
% Overlay ground-truth obstacles
for o = 1:size(obstacles,1)
    rectangle('Position', [obstacles(o,1), obstacles(o,2), ...
        obstacles(o,3)-obstacles(o,1), obstacles(o,4)-obstacles(o,2)], ...
        'EdgeColor','r','LineWidth',2);
end
plot(truePose(:,1), truePose(:,2), 'b--', 'LineWidth', 1.5);
plot(slamPoses(:,1), slamPoses(:,2), 'r-', 'LineWidth', 1.5);
title('Estimated-pose map vs ground-truth obstacles');
xlabel('X (m)'); ylabel('Y (m)');
legend('','Obstacles (truth)','True traj','Est traj','Location','bestoutside');

sgtitle('End-to-End Validation: Occupancy Map from Estimated Poses');

%% 5. Summary
occ = occupancyMatrix(map_est);
fprintf('\n=== Map from ESTIMATED Poses ===\n');
fprintf('Grid size:             %d x %d\n', map_est.GridSize(1), map_est.GridSize(2));
fprintf('Occupied cells:        %d (%.2f%%)\n', ...
    sum(occ(:) > 0.65), 100*sum(occ(:) > 0.65)/numel(occ));
fprintf('Free cells:            %d (%.2f%%)\n', ...
    sum(occ(:) < 0.35), 100*sum(occ(:) < 0.35)/numel(occ));
fprintf('Unknown cells:         %d (%.2f%%)\n', ...
    sum(occ(:) >= 0.35 & occ(:) <= 0.65), ...
    100*sum(occ(:) >= 0.35 & occ(:) <= 0.65)/numel(occ));
fprintf('-------------------------------------------\n');
fprintf('Compare visually above: map quality using estimated poses.\n');

save('local_occupancy_map_from_slam.mat', 'map_est');
fprintf('\nMap saved to local_occupancy_map_from_slam.mat\n');

%% Helper
function t = rayBoxIntersect(px, py, dx, dy, xmin, ymin, xmax, ymax)
    tmin = -inf;
    tmax = inf;

    if abs(dx) < 1e-9
        if px < xmin || px > xmax, t = NaN; return; end
    else
        t1 = (xmin - px)/dx;
        t2 = (xmax - px)/dx;
        if t1 > t2, [t1, t2] = deal(t2, t1); end
        tmin = max(tmin, t1);
        tmax = min(tmax, t2);
    end

    if abs(dy) < 1e-9
        if py < ymin || py > ymax, t = NaN; return; end
    else
        t1 = (ymin - py)/dy;
        t2 = (ymax - py)/dy;
        if t1 > t2, [t1, t2] = deal(t2, t1); end
        tmin = max(tmin, t1);
        tmax = min(tmax, t2);
    end

    if tmax < tmin || tmax < 0
        t = NaN;
    else
        t = max(tmin, 0);
    end
end