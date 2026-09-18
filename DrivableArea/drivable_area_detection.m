clc;
clear;
close all;

dataPath = "C:\Users\mohitkumar\OneDrive\Documents\Indian AV\Mohith\objectdetection\Dataset\IDD_subset";

load(fullfile(dataPath,"YOLOv4_IDD_Final.mat"),"detectorCompleted");

imageFolder = fullfile(dataPath,"val","images");
imageFiles = dir(fullfile(imageFolder,"*.jpg"));

figure;

for k = 1:10

    I = imread(fullfile(imageFolder,imageFiles(k).name));

    [height,width,~] = size(I);

    [bboxes,scores,labels] = detect( ...
        detectorCompleted,I,Threshold=0.1);

    hsvImage = rgb2hsv(I);

    sat = hsvImage(:,:,2);
    val = hsvImage(:,:,3);

    mask = false(height,width);

    mask(round(height*0.48):end,:) = true;

    roadColor = sat < 0.55 & val > 0.08;

    mask = mask & roadColor;

    roi = poly2mask( ...
        [round(width*0.02) round(width*0.42) round(width*0.58) round(width*0.98)], ...
        [height round(height*0.48) round(height*0.48) height], ...
        height,width);

    mask = mask & roi;

    objectMask = false(height,width);

    for i = 1:size(bboxes,1)

        x1 = max(1,round(bboxes(i,1)));
        y1 = max(1,round(bboxes(i,2)));

        x2 = min(width,round(bboxes(i,1)+bboxes(i,3)));
        y2 = min(height,round(bboxes(i,2)+bboxes(i,4)));

        objectMask(y1:y2,x1:x2) = true;

    end

    mask(objectMask) = false;

    mask = imopen(mask,strel("disk",5));
    mask = imclose(mask,strel("disk",15));
    mask = imfill(mask,"holes");

    overlay = I;

    overlay(:,:,1) = uint8(double(I(:,:,1))*0.5 + double(mask)*120);
    overlay(:,:,2) = uint8(double(I(:,:,2))*0.5 + double(mask)*180);
    overlay(:,:,3) = uint8(double(I(:,:,3))*0.5);

    imshow(overlay);

    title("YOLO-Assisted Drivable Area | Frame " + string(k));

    drawnow;
    pause(0.5);

end