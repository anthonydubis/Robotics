function hw5_doortracker_team_04( R )
    close all;
    
    img_path = 'ex/im10.png';
    [area, cent, hue, img_sz] = get_initial_object_info(img_path);
end

% Used to get the initial object information given the user selection
function [area, cent, hue, img_sz] = get_initial_object_info(img_path)
    img = imread(img_path);
    hsv = rgb2hsv(img);
    sz = size(img);
    img_sz = sz(:,1:2);
    
    % Get a point on the object from the user
    figure; imshow(img);
    pt = round(ginput(1));
    
%     rgb_val = zeros(1,3);
%     rgb_val(1) = img(pt(2),pt(1),1);
%     rgb_val(2) = img(pt(2),pt(1),2);
%     rgb_val(3) = img(pt(2),pt(1),3);
%     process_image_for_rgb(img, rgb_val);
    
    % Threshold to find the object
    hue = hsv(pt(2),pt(1),1);
    figure; imshow(hsv(:,:,1) < .3);
    % figure; imshow(hsv(:,:,2));
    % figure; imshow(hsv(:,:,3));
    BW = process_image_for_hue(hsv, hue);
    figure; imshow(BW);
end

function processed = process_image_for_hue(hsv, hue)
    range = 0.06;

    % Create BW image with 1s where the image has the hue within range of
    % the user's specification and the saturation/value is not close to 0
    processed = hsv(:,:,1) > hue-range & hsv(:,:,1) < hue+range ...
        & hsv(:,:,2) > .3 & hsv(:,:,3) > .15;
    
    % Remove noise
    processed = bwareaopen(processed, 100);
end

function processed = process_image_for_rgb(img, rgb)
    r_range = rgb(1) * .3;
    g_range = rgb(2) * .3;
    b_range = rgb(3) * .3;
    
    processed = img(:,:,1) > rgb(1) - r_range & img(:,:,1) < rgb(1) + r_range ...
        & img(:,:,2) > rgb(2) - g_range & img(:,:,2) < rgb(2) + g_range ...
        & img(:,:,3) > rgb(3) - b_range & img(:,:,3) < rgb(3) + g_range;
    
    processed = bwareaopen(processed, 100);
    
    imshow(processed);
end