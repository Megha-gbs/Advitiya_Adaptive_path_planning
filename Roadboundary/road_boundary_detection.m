clc;
clear;
close all;

dataPath = "C:\Users\mohitkumar\OneDrive\Documents\Indian AV\Mohith\objectdetection\Dataset\IDD_subset";

imageFolder = fullfile(dataPath,"val","images");
imageFiles = dir(fullfile(imageFolder,"*.jpg"));

I = imread(fullfile(imageFolder,imageFiles(1).name));

[height,width,~] = size(I);

labImage = rgb2lab(I);

bottomRegion = labImage( ...
    round(height*0.75):height, ...
    round(width*0.30):round(width*0.70), :);

L = bottomRegion(:,:,1);
A = bottomRegion(:,:,2);
B = bottomRegion(:,:,3);

roadL = median(L(:));
roadA = median(A(:));
roadB = median(B(:));

distance = sqrt( ...
    (labImage(:,:,1)-roadL).^2 + ...
    (labImage(:,:,2)-roadA).^2 + ...
    (labImage(:,:,3)-roadB).^2);

roadMask = distance < 28;

roadMask(1:round(height*0.45),:) = false;

roadMask = imopen(roadMask,strel("disk",5));
roadMask = imclose(roadMask,strel("disk",20));
roadMask = imfill(roadMask,"holes");

connected = bwconncomp(roadMask);

roadComponent = false(height,width);

for k = 1:connected.NumObjects

    pixels = connected.PixelIdxList{k};

    if any(pixels == sub2ind([height width],height,round(width/2)))

        roadComponent(pixels) = true;

    end

end

roadMask = roadComponent;

leftBoundary = nan(height,1);
rightBoundary = nan(height,1);

for y = round(height*0.45):height

    x = find(roadMask(y,:));

    if ~isempty(x)

        leftBoundary(y) = min(x);
        rightBoundary(y) = max(x);

    end

end

validRows = find(~isnan(leftBoundary) & ~isnan(rightBoundary));

outputImage = I;

if numel(validRows) > 20

    y1 = validRows(1);
    y2 = validRows(end);

    leftX = leftBoundary(validRows);
    rightX = rightBoundary(validRows);

    leftFit = polyfit(validRows,leftX,2);
    rightFit = polyfit(validRows,rightX,2);

    yPlot = linspace(y1,y2,100)';

    leftPlot = polyval(leftFit,yPlot);
    rightPlot = polyval(rightFit,yPlot);

    for k = 1:length(yPlot)-1

        outputImage = insertShape( ...
            outputImage, ...
            "Line", ...
            [leftPlot(k) yPlot(k) leftPlot(k+1) yPlot(k+1)], ...
            "Color","yellow", ...
            "LineWidth",5);

        outputImage = insertShape( ...
            outputImage, ...
            "Line", ...
            [rightPlot(k) yPlot(k) rightPlot(k+1) yPlot(k+1)], ...
            "Color","yellow", ...
            "LineWidth",5);

    end

    boundaryFound = true;

else

    boundaryFound = false;

end

figure("Name","Road Boundary Detection");

subplot(1,3,1);
imshow(I);
title("Original Image");

subplot(1,3,2);
imshow(roadMask);
title("Estimated Road Surface");

subplot(1,3,3);
imshow(outputImage);
title("Estimated Road Boundaries");

fprintf("\n--- ROAD BOUNDARY RESULT ---\n");
fprintf("Boundary detected: %d\n",boundaryFound);