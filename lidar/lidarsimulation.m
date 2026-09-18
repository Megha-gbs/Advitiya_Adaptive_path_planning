clc;
clear;
close all;

dataPath = "C:\Users\mohitkumar\OneDrive\Documents\Indian AV\Mohith\objectdetection\Dataset\IDD_subset";

load(fullfile(dataPath,"YOLOv4_IDD_Final.mat"),"detectorCompleted");

imageFolder = fullfile(dataPath,"val","images");
imageFiles = dir(fullfile(imageFolder,"*.jpg"));

cameraImage = imread(fullfile(imageFolder,imageFiles(1).name));

[bboxes,scores,labels] = detect( ...
    detectorCompleted,cameraImage,Threshold=0.1);

[x,y] = meshgrid(-20:0.2:50,0:0.2:60);
z = zeros(size(x));

roadPoints = [x(:) y(:) z(:)];

vehicle1 = [ ...
    -2 + 2*rand(1500,1), ...
    15 + 4*rand(1500,1), ...
    0.5 + 1.5*rand(1500,1)];

vehicle2 = [ ...
    3 + 2*rand(1500,1), ...
    30 + 4*rand(1500,1), ...
    0.5 + 1.5*rand(1500,1)];

vehicle3 = [ ...
    -6 + 2*rand(1200,1), ...
    45 + 4*rand(1200,1), ...
    0.5 + 1.5*rand(1200,1)];

points = [roadPoints; vehicle1; vehicle2; vehicle3];

ptCloud = pointCloud(points);

cameraOutput = insertObjectAnnotation( ...
    cameraImage,"rectangle",bboxes, ...
    string(labels) + " " + string(round(scores,2)));

figure;

subplot(1,2,1);
imshow(cameraOutput);
title("Camera + YOLOv4");

subplot(1,2,2);
pcshow(ptCloud);
xlabel("X (m)");
ylabel("Y (m)");
zlabel("Z (m)");
title("LiDAR Point Cloud");

fprintf("Camera detections: %d\n",size(bboxes,1));
fprintf("LiDAR points: %d\n",ptCloud.Count);