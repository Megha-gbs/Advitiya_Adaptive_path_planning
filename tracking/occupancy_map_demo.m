%% Occupancy Map Demo — Hanish Localization Subsystem
% Status: Builds a 2D occupancy grid from simulated LiDAR scans
% Output: occupancyMap object representing drivable/non-drivable space
% Interface: pose + map → Yashwanth (planning)
% Note: uses manual ray-marching instead of insertRay for reliability
% NOTE: This demo builds the occupancy map using ground-truth poses.
% It demonstrates map construction from LiDAR, but not map quality
% when using estimated localization. See occupancy_map_from_slam.m
% for the end-to-end version using SLAM-estimated poses.
clear; clc; close all; rng(42);

%% 1. Environment definition
mapWidth   = 20;    % meters
mapHeight  = 20;    % meters
resolution = 0.1;   % 10 cm per cell

% Obstacles as [x_min, y_min, x_max, y_max]
obstacles = [
     2  2   4   6;
     2  2   8   3;
     6  2   7   8;
    10  5  14   6;
    12 10  16  14;
     5 12   9  15;
    16  3  18   5;
];

%% 2. Ground truth trajectory (square loop, inward from walls)
trajWaypoints = [8 8; 15 8; 15 15; 8 15; 8 8];
numSamples = 200;

segLen   = sqrt(sum(diff(trajWaypoints).^2, 2));
cumLen   = [0; cumsum(segLen)];
totalLen = cumLen(end);
s = linspace(0, totalLen, numSamples);

trueX = interp1(cumLen, trajWaypoints(:,1), s(:), 'linear');
trueY = interp1(cumLen, trajWaypoints(:,2), s(:), 'linear');
truePose = [trueX(:), trueY(:)];

trueYaw = atan2(gradient(trueY(:)), gradient(trueX(:)));

%% 3. LiDAR simulation
numBeams   = 360;
maxRange   = 10;
lidarNoise = 0.02;

angles = linspace(-pi, pi, numBeams)';
ranges = zeros(numSamples, numBeams);

fprintf('Simulating %d LiDAR scans with %d beams each...\n', ...
    numSamples, numBeams);

for k = 1:numSamples
    pose = truePose(k,:);
    yaw  = trueYaw(k);

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

        rNoisy = r + lidarNoise*randn;
        if ~isfinite(rNoisy) || rNoisy < 0
            rNoisy = maxRange;
        end
        ranges(k,b) = min(maxRange, rNoisy);
    end
end

fprintf('Simulated %d scans successfully.\n', numSamples);

%% 4. Build occupancy map via manual ray-marching
map = occupancyMap(mapWidth, mapHeight, 1/resolution);
map.GridLocationInWorld = [0 0];

fprintf('Building occupancy map...\n');

occMat = 0.5 * ones(map.GridSize);   % start all cells unknown

stepSize = resolution;               % 10 cm march step

for k = 1:numSamples
    px  = truePose(k,1);
    py  = truePose(k,2);
    yaw = trueYaw(k);

    for b = 1:numBeams
        r = ranges(k,b);
        if r <= 0 || ~isfinite(r)
            continue;
        end

        rayAngle = yaw + angles(b);
        numSteps = floor(r / stepSize);

        % Mark free cells along the ray
        for i = 1:numSteps
            cx = px + i*stepSize*cos(rayAngle);
            cy = py + i*stepSize*sin(rayAngle);

            col = round(cx * map.Resolution) + 1;
            row = round(cy * map.Resolution) + 1;

            if col < 1 || col > map.GridSize(2) || ...
               row < 1 || row > map.GridSize(1)
                break;
            end

            occMat(row, col) = max(0, occMat(row, col) - 0.05);
        end

        % Mark the endpoint as occupied
        ex = px + r*cos(rayAngle);
        ey = py + r*sin(rayAngle);

        col = round(ex * map.Resolution) + 1;
        row = round(ey * map.Resolution) + 1;

        if col >= 1 && col <= map.GridSize(2) && ...
           row >= 1 && row <= map.GridSize(1)
            occMat(row, col) = min(1, occMat(row, col) + 0.4);
        end
    end
end

% Push the occupancy matrix back into the map object
map.setOccupancy(occMat);

fprintf('Map built: %d x %d cells at %.2f m/cell\n', ...
    map.GridSize(1), map.GridSize(2), 1/map.Resolution);

%% 5. Visualize
figure('Name','Occupancy Map — Hanish Localization','Color','w', ...
    'Position',[100 100 1200 600]);

subplot(1,2,1);
show(map);
hold on;
plot(truePose(:,1), truePose(:,2), 'g-', 'LineWidth', 2);
plot(truePose(1,1), truePose(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor','g');
plot(truePose(end,1), truePose(end,2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor','r');
title('Occupancy Map (built from LiDAR)');
xlabel('X (m)'); ylabel('Y (m)');
legend('Map','Trajectory','Start','End','Location','bestoutside');

subplot(1,2,2);
imagesc([0 mapWidth], [0 mapHeight], 1-occupancyMatrix(map));
set(gca,'YDir','normal'); colormap(gray); hold on;
for o = 1:size(obstacles,1)
    rectangle('Position', [obstacles(o,1), obstacles(o,2), ...
        obstacles(o,3)-obstacles(o,1), obstacles(o,4)-obstacles(o,2)], ...
        'EdgeColor','r','LineWidth',2);
end
plot(truePose(:,1), truePose(:,2), 'g-', 'LineWidth', 2);
title('Map vs Ground Truth Obstacles');
xlabel('X (m)'); ylabel('Y (m)');
axis equal; axis([0 mapWidth 0 mapHeight]);

sgtitle('Occupancy Map — Hanish Localization Subsystem');

%% 6. Save map
save('local_occupancy_map.mat', 'map');
fprintf('\nMap saved to local_occupancy_map.mat\n');

%% 7. Summary
occ = occupancyMatrix(map);
fprintf('\n=== Occupancy Map Summary ===\n');
fprintf('Grid size:             %d x %d\n', map.GridSize(1), map.GridSize(2));
fprintf('Resolution:            %.3f m/cell\n', 1/map.Resolution);
fprintf('Occupied cells:        %d (%.2f%%)\n', ...
    sum(occ(:) > 0.65), 100*sum(occ(:) > 0.65)/numel(occ));
fprintf('Free cells:            %d (%.2f%%)\n', ...
    sum(occ(:) < 0.35), 100*sum(occ(:) < 0.35)/numel(occ));
fprintf('Unknown cells:         %d (%.2f%%)\n', ...
    sum(occ(:) >= 0.35 & occ(:) <= 0.65), ...
    100*sum(occ(:) >= 0.35 & occ(:) <= 0.65)/numel(occ));
fprintf('-------------------------------------------\n');
fprintf('Ready for planning handoff.\n');

%% ============================================================
%% Helper: Ray-box intersection (axis-aligned bounding box)
%% ============================================================
function t = rayBoxIntersect(px, py, dx, dy, xmin, ymin, xmax, ymax)
    tmin = -inf;
    tmax = inf;

    % X slab
    if abs(dx) < 1e-9
        if px < xmin || px > xmax, t = NaN; return; end
    else
        t1 = (xmin - px)/dx;
        t2 = (xmax - px)/dx;
        if t1 > t2, [t1, t2] = deal(t2, t1); end
        tmin = max(tmin, t1);
        tmax = min(tmax, t2);
    end

    % Y slab
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