function hw5_part1_team_04(R)
    close all;
    img_path = 'http://192.168.0.102/snapshot.cgi?user=admin&pwd=&resolution=10&rate=0';
    img_path = 'cone.jpg';

    [t_size, color, t_pos] = get_object_info(img_path);

end

function [t_size, t_hue, t_pos] = get_object_info(img_path)
    img = imread(img_path);
    hsv = rgb2hsv(img);
    sz = size(img);
    
    % Get a point on the object from the user
    imshow(img);
    pt = round(ginput(1));
    
    % Threshold to find the object
    t_hue = hsv(pt(2),pt(1),1);
    processed = process_image_for_hue(hsv, t_hue);
    imshow(processed);
    
    t_size = 0;
    t_pos = 0;
end

function processed = process_image_for_hue(hsv, hue)
    range = 0.03;

    processed = hsv(:,:,1) > hue-range & hsv(:,:,1) < hue+range ...
        & hsv(:,:,2) > .25 & hsv(:,:,3) > .25;
    
    % Remove noise
    processed = bwareaopen(processed, 20);
end