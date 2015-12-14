%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 5 - Color Tracker
%
% Team number: 4
% Team leader: Anthony Dubis (ajd2194)
% Team members: Lilly Wang (lfw2114), Samir Mathrani (sm3619)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function hw5_colortracker_team_04(R)
    close all; % Closes any images you had open previously
    
    img_path = 'http://192.168.0.101/snapshot.cgi?user=admin&pwd=&resolution=10&rate=0';

    [t_area, t_cent, hue, img_sz] = get_initial_object_info(img_path);
    
    while true
        % Given the hue we got above, get information on the largest object
        % with that hue in the latest picture
        [found, n_area, n_cent] = get_object_info(img_path, hue);
        
        % If the object left view, skip this image
        if ~found
            continue;
        end
        
        if should_realign(t_cent, n_cent, img_sz)
            realign(R, t_cent, n_cent, img_sz);
        else
            keep_distance(R, t_area, n_area);
            pause(0.1);
        end
    end
end

% Keeps the robot within a fixed distance of the object, according to some
% threshold.
function keep_distance(R, t_area, n_area)
    percentage = n_area / t_area;
    if percentage < .85
        SetFwdVelRadiusRoomba(R, 0.2, inf);
    elseif percentage > 1.15
        SetFwdVelRadiusRoomba(R, -0.2, inf);
    else
        SetFwdVelRadiusRoomba(R, 0, inf);
    end
end

% Returns true if the robot needs to realign itself
% t_cent is the target centroid
% n_cent is the new centroid
% img_sz is the image resolution (ex. 320 x 480)
function realign = should_realign(t_cent, n_cent, img_sz)
    realign = false;
    
    degrees = degrees_misaligned(t_cent, n_cent, img_sz);
    if abs(degrees) > 15
        realign = true;
    end
end

function realign(R, t_cent, n_cent, img_sz)
    degrees = degrees_misaligned(t_cent, n_cent, img_sz);
    turnAngle(R, 0.1, -degrees);
end

% Returns the degrees current centroid is off from the target centroid
% t_cent is the target centroid (x,y with the top left being the origin)
% n_cent is the new centroid (x,y with the top left being the origin)
% img_sz is the image resolution (height x width in MATLAB)
% Per the camera, the field of view is 67 degrees
function degrees = degrees_misaligned(t_cent, n_cent, img_sz)
    field_of_view = 67;
    img_width = img_sz(2);
    pixels_per_degree = img_width / field_of_view;
    diff = n_cent(1) - t_cent(1);
    degrees = diff / pixels_per_degree;
end

% Used to get the initial object information given the user selection
function [area, cent, hue, img_sz] = get_initial_object_info(img_path)
    img = imread(img_path);
    hsv = rgb2hsv(img);
    sz = size(img);
    img_sz = sz(:,1:2);
    
    % Get a point on the object from the user
    imshow(img);
    pt = round(ginput(1));
    
    % Threshold to find the object
    hue = hsv(pt(2),pt(1),1);
    BW = process_image_for_hue(hsv, hue);
    imshow(BW);
    
    object = get_largest_object_stats(BW);
    
    area = object.FilledArea;
    cent = [(img_sz(2)/2) (img_sz(1)/2)];
end

% Used to get info about the object that matches the given hue
function [found, area, cent] = get_object_info(img_path, hue)
    img = imread(img_path);
    hsv = rgb2hsv(img);
    
    BW = process_image_for_hue(hsv, hue);
    imshow(BW);
    
    object = get_largest_object_stats(BW);
    
    if isempty(object)
        found = false;
        return;
    end
    
    found = true;
    area = object.FilledArea;
    cent = object.Centroid;
end

function processed = process_image_for_hue(hsv, hue)
    range = 0.06;

    % Create BW image with 1s where the image has the hue within range of
    % the specified huge and the saturation/value is not close to 0
    processed = hsv(:,:,1) > hue-range & hsv(:,:,1) < hue+range ...
        & hsv(:,:,2) > .05 & hsv(:,:,3) > .05;
    
    % Remove noise
    processed = bwareaopen(processed, 20);
end

function object = get_largest_object_stats(BW)
    stats = regionprops(BW, 'Centroid', 'FilledArea');
    
    if isempty(stats)
        object = 0;
        return;
    end
    
    object = stats(1);
    for i=1:length(stats)
        if stats(i).FilledArea > object.FilledArea
            object = stats(i);
        end
    end
end