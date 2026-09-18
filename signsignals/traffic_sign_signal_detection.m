clc;
clear;
close all;

dataPath = "C:\Users\mohitkumar\OneDrive\Documents\Indian AV\Mohith\objectdetection\Dataset\IDD_subset";

load(fullfile(dataPath,"YOLOv4_IDD_Final.mat"),"detectorCompleted");

imageFolder = fullfile(dataPath,"train","images");
imageFiles = dir(fullfile(imageFolder,"*.jpg"));

totalImages = numel(imageFiles);
imagesWithSigns = 0;
imagesWithLights = 0;
totalSigns = 0;
totalLights = 0;

for k = 1:totalImages

    I = imread(fullfile(imageFolder,imageFiles(k).name));

    [~,scores,labels] = detect( ...
        detectorCompleted,I,Threshold=0.1);

    signCount = sum(labels == "trafficSign");
    lightCount = sum(labels == "trafficLight");

    if signCount > 0
        imagesWithSigns = imagesWithSigns + 1;
    end

    if lightCount > 0
        imagesWithLights = imagesWithLights + 1;
    end

    totalSigns = totalSigns + signCount;
    totalLights = totalLights + lightCount;

    if mod(k,100) == 0
        fprintf("Processed %d / %d images\n",k,totalImages);
    end
end

fprintf("\n--- TRAFFIC SIGN & SIGNAL RESULTS ---\n");
fprintf("Total images: %d\n",totalImages);
fprintf("Images with traffic signs: %d\n",imagesWithSigns);
fprintf("Images with traffic lights: %d\n",imagesWithLights);
fprintf("Total traffic sign detections: %d\n",totalSigns);
fprintf("Total traffic light detections: %d\n",totalLights);