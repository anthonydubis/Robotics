function hw5_part1_team_04(R)
    close all; % Closes any images you had open previously
    
    img_path = 'http://192.168.0.102/snapshot.cgi?user=admin&pwd=&resolution=10&rate=0';
    img_path = 'stop.jpg'; % Using test image for now

    [t_area, t_pos, hue] = get_initial_object_info(img_path);
    
    while true
        [n_area, n_pos] = get_object_info(img_path, hue);
    end
end

% Used to get the initial object information given the user selection
function [area, pos, hue] = get_initial_object_info(img_path)
    img = imread(img_path);
    hsv = rgb2hsv(img);
    
    % Get a point on the object from the user
    imshow(img);
    pt = round(ginput(1));
    
    % Threshold to find the object
    hue = hsv(pt(2),pt(1),1);
    BW = process_image_for_hue(hsv, hue);
    imshow(BW);
    
    object = get_largest_object_stats(BW);
    
    area = object.FilledArea;
    pos = object.Centroid;
end

% Used to get info about the object that matches the given hue
function [size, pos] = get_object_info(img_path, hue)
end

function processed = process_image_for_hue(hsv, hue)
    range = 0.06;

    processed = hsv(:,:,1) > hue-range & hsv(:,:,1) < hue+range ...
        & hsv(:,:,2) > .05 & hsv(:,:,3) > .05;
    
    % Remove noise
    processed = bwareaopen(processed, 20);
end

function object = get_largest_object_stats(BW)
    stats = regionprops(BW, 'Centroid', 'FilledArea');
    
    object = stats(1);
    for i=1:length(stats)
        if stats(i).FilledArea > object.FilledArea
            object = stats(i);
        end
    end
end