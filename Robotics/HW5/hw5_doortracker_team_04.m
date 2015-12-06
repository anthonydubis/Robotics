function hw5_doortracker_team_04( R )
    close all;
    
    img_path = 'ex/im14.png';
    % sample_image(img_path);
    [area, cent, hue, img_sz] = get_initial_object_info(img_path);
end

function sample_image(img_path)
    img = imread(img_path);
    hsv = rgb2hsv(img);
    figure; imshow(img);
    pts = ginput;
    pts = round(pts);
    
    for i=1:length(pts)
        p = pts(i,:);
        hsv_vals = [hsv(p(2),p(1),1) hsv(p(2),p(1),2) hsv(p(2),p(1),3)]
    end
end

% Used to get the initial object information given the user selection
function [area, cent, hue, img_sz] = get_initial_object_info(img_path)
    img = imread(img_path);
    hsv = rgb2hsv(img);
    sz = size(img);
    img_sz = sz(:,1:2);
    
    % Get a point on the object from the user
    figure; imshow(img);
    
    % Threshold to find the object
    BW = process_image_for_doors(hsv);
    figure; imshow(BW);
end

function processed = process_image_for_doors(hsv)
    % Create BW image with 1s where the image has the hue within range of
    % the user's specification and the saturation/value is not close to 0
    processed = hsv(:,:,1) > .56 & hsv(:,:,1) < .7 & hsv(:,:,2) > .25 ...
        & hsv(:,:,2) < .44 & hsv(:,:,3) > .21 & hsv(:,:,3) < .48;
    
    % Remove noise
    processed = bwareaopen(processed, 300);
end