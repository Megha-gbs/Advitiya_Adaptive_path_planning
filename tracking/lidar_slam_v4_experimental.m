%% LiDAR SLAM v4 — Scan-to-Map + Loop Closure (Navigation Toolbox)
% Hanish — Localization Subsystem
%% EXPERIMENTAL — SHELVED 2026-09-17
% Attempt to replace v3's frame-to-frame odometry (lidar_slam_demo.m)
% with lidarSLAM (scan-to-map matching + pose-graph loop closure), to
% fix drift at 90-degree corners and exploit loop closure on this
% trajectory's closed square loop.
%
% This is the FINAL configuration tried, after tuning down from
% defaults for speed/quality:
%   MovementThreshold       [0.05 0.05] -> [0.15 0.10]
%   LoopClosureSearchRadius 8           -> 4
%   numBeams                360         -> 180
%
% MEASURED RESULT (this exact tuned configuration, 200-scan trajectory):
%   Accepted scans:        112 / 200
%   Loop closures:         0
%   Mean position error:   10.372 m
%   Max position error:    15.414 m
%   Mean yaw error:        48.22 deg
%   Runtime:               915.3 s
%   Final SLAM pose:       [4.69, 6.51, 2.10 rad]
%
% Tuning the movement/loop-closure thresholds and beam count changed
% match cadence and search cost, not the underlying scan matcher's
% ability to converge on this idealized ray-cast geometry: result is
% WORSE than the untuned v3 baseline on every metric (error, yaw error,
% and runtime), with zero loop closures found despite the trajectory
% being a closed loop. Do not use this as the reported LiDAR result.
%
% Superseded by: lidar_slam_demo.m (v3 hybrid scan matching + gyro
% odometry, frozen baseline, 3.409 m mean error, 0.09 deg mean yaw
% error, 10.9 s runtime, VALIDATED 2026-09-17).
% Kept for reference in case real (non-idealized) LiDAR data is used,
% where feature density may be closer to what lidarSLAM assumes.
%
% REQUIRES: Navigation Toolbox (lidarSLAM, lidarScan, buildMap).
% =====================================================================

clear; clc; close all; rng(42);

%% 1. Environment definition
mapWidth   = 20;
mapHeight  = 20;
resolution = 0.1;              % 10 cm/cell -> 10 cells/meter

obstacles = [
     2  2   4   6;
     2  2   8   3;
     6  2   7   8;
    10  5  14   6;
    12 10  16  14;
     5 12   9  15;
    16  3  18   5;
];

%% 2. Ground truth trajectory
trajWaypoints = [8 8; 15 8; 15 15; 8 15; 8 8];
numSamples = 200;

segLen   = sqrt(sum(diff(trajWaypoints).^2, 2));
cumLen   = [0; cumsum(segLen)];
totalLen = cumLen(end);
s = linspace(0, totalLen, numSamples);

trueX = interp1(cumLen, trajWaypoints(:,1), s(:), 'linear');
trueY = interp1(cumLen, trajWaypoints(:,2), s(:), 'linear');
truePose = [trueX(:), trueY(:)];
trueYaw  = atan2(gradient(trueY(:)), gradient(trueX(:)));

expectedStep = totalLen / (numSamples - 1);
fprintf('Trajectory: %d samples, %.3f m per step on average\n', ...
    numSamples, expectedStep);

%% 3. LiDAR simulation
numBeams   = 180;              % tuned down from 360 for speed
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

%% 4. Run lidarSLAM with gyro-seeded scan matching (tuned parameters)
mapResolution = 1/resolution;

slamAlg = lidarSLAM(mapResolution, maxRange);
slamAlg.MovementThreshold        = [0.15 0.10];   % was [0.05 0.05]
slamAlg.LoopClosureThreshold     = 500;
slamAlg.LoopClosureSearchRadius  = 4;              % was 8
slamAlg.OptimizationInterval     = 5;

fprintf('\nRunning lidarSLAM over %d scans (with gyro prior)...\n', numSamples);

gyroYaw = trueYaw + 0.002 * randn(numSamples, 1);

acceptedTruePose = [];
acceptedTrueYaw  = [];
acceptedRelPose  = [];
loopClosureCount = 0;

lastAcceptedGyroYaw = gyroYaw(1);

tic;
for k = 1:numSamples
    scan = lidarScan(ranges(k,:)', angles);

    if k == 1
        relPoseEst = [0, 0, 0];
    else
        dYaw = wrapToPi(gyroYaw(k) - lastAcceptedGyroYaw);
        relPoseEst = [expectedStep, 0, dYaw];
    end

    [isScanAccepted, loopClosureInfo] = addScan(slamAlg, scan, relPoseEst);

    if isScanAccepted
        acceptedTruePose(end+1,:) = truePose(k,:);      %#ok<SAGROW>
        acceptedTrueYaw(end+1,1)  = trueYaw(k);         %#ok<SAGROW>
        acceptedRelPose(end+1,:)  = relPoseEst;         %#ok<SAGROW>
        lastAcceptedGyroYaw       = gyroYaw(k);

        if ~isempty(loopClosureInfo.EdgeIDs)
            loopClosureCount = loopClosureCount + 1;
            fprintf('  Loop closure at scan %d (edge IDs: %s, scores: %s)\n', ...
                k, mat2str(loopClosureInfo.EdgeIDs), ...
                mat2str(loopClosureInfo.Scores));
        end
    end

    if mod(k, 20) == 0
        fprintf('  Scan %d / %d (%d accepted so far)\n', ...
            k, numSamples, size(acceptedTruePose,1));
    end
end
elapsed = toc;
fprintf('lidarSLAM complete (%.1f seconds), %d loop closures\n', ...
    elapsed, loopClosureCount);

%% 5. Extract optimized poses and compare to ground truth
[scans, optimizedPoses] = scansAndPoses(slamAlg);

if size(optimizedPoses,1) ~= size(acceptedTruePose,1)
    warning(['Accepted-scan count mismatch (%d SLAM poses vs %d logged ' ...
        'ground truth) — pose-graph optimization can reorder/prune; ' ...
        'check indices before trusting the error numbers below.'], ...
        size(optimizedPoses,1), size(acceptedTruePose,1));
end

n = min(size(optimizedPoses,1), size(acceptedTruePose,1));
posError_slam = vecnorm(optimizedPoses(1:n,1:2) - acceptedTruePose(1:n,:), 2, 2);
yawError_slam = abs(wrapToPi(optimizedPoses(1:n,3) - acceptedTrueYaw(1:n)));

fprintf('\n=== lidarSLAM v4 (tuned) Results ===\n');
fprintf('Accepted scans:          %d / %d\n', n, numSamples);
fprintf('Loop closures accepted:  %d\n', loopClosureCount);
fprintf('Mean position error:     %.3f m\n', mean(posError_slam));
fprintf('Max position error:      %.3f m\n', max(posError_slam));
fprintf('Mean yaw error:          %.4f rad (%.2f deg)\n', ...
    mean(yawError_slam), rad2deg(mean(yawError_slam)));
fprintf('Final SLAM pose:         [%.2f, %.2f, %.2f rad]\n', ...
    optimizedPoses(n,1), optimizedPoses(n,2), optimizedPoses(n,3));
fprintf('Final true pose:         [%.2f, %.2f, %.2f rad]\n', ...
    acceptedTruePose(n,1), acceptedTruePose(n,2), acceptedTrueYaw(n));
fprintf('Runtime:                 %.1f seconds\n', elapsed);

%% ============================================================
%% Helper: Ray-box intersection
%% ============================================================
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
