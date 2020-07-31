function [stats,xPos,yPos] = getPosColour(image, colour)
    %colour: red=1,blue=2,yellow=3
    blocks = imread(image);
    figure(1);
    conveyerMask = getConveyerMask(blocks);
    % show conveyer mask
    imshow(conveyerMask)
    % Convert RGB image to chosen color space
    I = blocks;
    if colour == 1
        % Thresholds for channel red colour
        channel1 = [255.000,255.000];
        channel2 = [0.000,5.000];
        channel3 = [0.000,5.000];
    end
    if colour == 2
        % Thresholds for channel blue colour
        channel1 = [0.000,5.000];
        channel2 = [0.000,5.000];
        channel3 = [255.000,255.000];
    end
    if colour == 3
        % Thresholds for channel blue colour
        channel1 = [255.000,255.000];
        channel2 = [255.000,255.000];
        channel3 = [0.000,5.000];
    end
    % Create mask based on chosen histogram thresholds
    BW = (I(:,:,1) >= channel1(1) ) & (I(:,:,1) <= channel1(2)) & ...
            (I(:,:,2) >= channel2(1) ) & (I(:,:,2) <= channel2(2)) & ...
            (I(:,:,3) >= channel3(1) ) & (I(:,:,3) <= channel3(2));
    % remove objects not on conveyer
    BW = BW & conveyerMask;
    figure(2);
    subplot(2,1,1);
    % display original RGB image
    imshow(blocks);
    axis on;
    subplot(2,1,2);
    % display black/white mask of red objects
    imshow(BW);
    BW = bwlabel(BW,4);
    stats = regionprops(BW, 'BoundingBox','Centroid','area');
    % area of each object
    area = cat(1,stats.Area);
    % co ordinates of each object
    pos = cat(1,stats.Centroid);
    % remove objects smaller than 5 area
    j = 1;
    for i = 1:length(area)
        if area(i,1) < 5
            pos(j,:) = [];
            j = j - 1;
        end
        j = j + 1;
    end
    xPos = pos(:,1);
    yPos = pos(:,2);
    hold on
    % display centroids
    plot(xPos,yPos,'b*');
    axis on;
    %[H,T,R] = hough(BW,'RhoResolution',0.5,'Theta',-90:0.5:89.5);
end

function mask = getConveyerMask(image)
% Convert RGB image to chosen color space
I = rgb2ycbcr(image);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 25.000;
channel1Max = 43.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 119.000;
channel2Max = 151.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 112.000;
channel3Max = 255.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
mask = sliderBW;

SE = strel('disk',6);
mask = imdilate(mask,SE);
mask = imfill(mask,'holes');
mask = imerode(mask,SE);
% Initialize output masked image based on input image.
%maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
%maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end