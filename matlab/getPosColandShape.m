function [xPos,yPos] = getPosColandShape(image, colour, shape)
    % colour: red=1, blue=2, yellow=3
    % shape: rectangle = 1, circle = 2, triangle = 3, any = 4
    blocks = image; %imread(image);
    
    conveyerMask = getConveyerMask(blocks);
    
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
    
    if shape == 1
        area_min = 1400;
        area_max = 1600;
        perimeter_min = 140;
        perimeter_max = 155;
    end
    if shape == 2
        area_min = 1100;
        area_max = 1300;
        perimeter_min = 95;
        perimeter_max = 130;
    end
    if shape == 3
        area_min = 1800;
        area_max = 2100;
        perimeter_min = 190;
        perimeter_max = 205;
    end
    if shape == 4
        area_min = 0;
        area_max = 6000;
        perimeter_min = 0;
        perimeter_max = 1000;
    end
        
    if colour ~= 0	
        BW = (I(:,:,1) >= channel1(1) ) & (I(:,:,1) <= channel1(2)) & ...	
                (I(:,:,2) >= channel2(1) ) & (I(:,:,2) <= channel2(2)) & ...	
                (I(:,:,3) >= channel3(1) ) & (I(:,:,3) <= channel3(2));	
        % remove objects not on conveyer	
        BW = BW & conveyerMask;	
    else	
        xPos = [-1];	
        yPos = [-1];	
        return;	
    end
    
    stats = regionprops(BW, 'BoundingBox','Centroid','area');
    blockstats = regionprops('table', BW, 'all');
    p = blockstats.Perimeter
    accepted = blockstats.Area >= area_min & blockstats.Area <= area_max & blockstats.Perimeter >= perimeter_min & blockstats.Perimeter <= perimeter_max;
    centroids = cat(1, blockstats.Centroid);
    
    row = 1;
    for i = find(accepted).'
        if centroids(i,1) ~= 0
            xPos(row,1) = round(centroids(i,1));
            yPos(row,1) = round(centroids(i,2));
            row = row + 1;
        else
            break;
        end
    end
    
    if row == 1
        xPos = [-1];
        yPos = [-1];
    end
%     figure(2);
%     imshow(BW);
%     hold on
%     axis on
%     plot(xPos,yPos,'b*');
%     hold off
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
