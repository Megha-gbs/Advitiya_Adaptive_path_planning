clc;
clear;
close all;

dataPath = "C:\Users\mohitkumar\OneDrive\Documents\Indian AV\Mohith\objectdetection\Dataset\IDD_subset";

load(fullfile(dataPath,"YOLOv4_IDD_Final.mat"),"detectorCompleted");

imageFolder = fullfile(dataPath,"val","images");
imageFiles = dir(fullfile(imageFolder,"*.jpg"));

figure;

for k = 1:10
    cameraImage = imread(fullfile(imageFolder,imageFiles(k).name));

    [bboxes,scores,labels] = detect( ...
        detectorCompleted,cameraImage,Threshold=0.1);

    outputImage = insertObjectAnnotation( ...
        cameraImage,"rectangle",bboxes, ...
        string(labels) + " " + string(round(scores,2)));

    imshow(outputImage);
    title("Camera Frame " + string(k) + ...
        " | Objects Detected: " + string(size(bboxes,1)));

    drawnow;
    pause(0.5);
end