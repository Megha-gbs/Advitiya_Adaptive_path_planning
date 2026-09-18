% Load the result
load('lidar_slam_result.mat');

% Check yaw drift
yaw_error = slamPoses(:,3) - trueYaw;
figure;
plot(1:200, rad2deg(slamPoses(:,3)), 'r-', 'LineWidth', 1.5); hold on;
plot(1:200, rad2deg(trueYaw), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Scan index'); ylabel('Yaw (deg)');
legend('SLAM yaw','True yaw');
title('Yaw drift over time');