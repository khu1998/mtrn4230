function [posX, posY] = ShapePosition(image,shape)
    % shape: rectangle = 1, circle = 2, triangle = 3
    base_img = imread(image);
    gray_base_img = rgb2gray(base_img);
    posX = [];
    posY = [];
    
    if shape == 1
       perimeter_min = 100;
       perimeter_max = 110;
    end
    if shape == 2
       perimeter_min = 100;
       perimeter_max = 110;
    end
    if shape == 3
       perimeter_min = 100;
       perimeter_max = 110;
    end

    subplot(1,2,1);
    imshow(base_img);
    title('Original RGB');
    
    I = rgb2hsv(base_img);
    hChannel = I(:, :, 3);
    hChannel = hChannel >= 0.713 & hChannel <= 1;
    
    [labeledImage numberOfObjects] = bwlabel(hChannel);
    
    blobMeasurements = regionprops('table', labeledImage, gray_base_img, 'all');
    accepted = blobMeasurements.Perimeter >= perimeter_min & blobMeasurements.Perimeter <= perimeter_max;
    maskedRgbImage = bsxfun(@times, base_img, cast(hChannel, 'like', base_img));
    centroids = cat(1, blobMeasurements.Centroid);
    subplot(1,2,2);
    imshow(maskedRgbImage);
    title('Shape Detection from Original RGB');
    hold(imgca,'on')
    
    row = 1;
    for i = find(accepted).'
        plot(imgca,centroids(i,1), centroids(i,2), 'black*');
        if centroids(i,1) ~= 0
            posX(row,1) = centroids(i,1);
            posY(row,1) = centroids(i,2);
            row = row + 1;
        else
            break
        end
    end
end