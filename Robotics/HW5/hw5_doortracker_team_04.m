function hw5_doortracker_team_04( R )
    close all;
    
    % Replace this with the camera image path
    img_path = 'ex/im14.png';
    turn = 0;
    
    while true
        while true
            img = imread(img_path);
            turn = should_approach_door(img);
            if turn
                % Stopping can likely be removed since we're going to go
                % forward a bit more anyway, but do it for debugging
                SetFwdVelRadiusRoomba(R, 0, inf);
                pause(1);
                break;
            end
            drive_down_hallway(R, img);
            pause(0.1);
        end

        move_to_front_of_door(R);
        turn_ninety_degrees(R, turn);

        img = imread(img_path);
        [door, found] = get_door(img);
        if ~found
            % Somehow, you lost the door. Straight out and continue down the
            % hall again.
            turn_ninety_degrees(R, turn * -1);
            continue;
        end
        
        center_on_object(R, door, img);
        knock_knock(R);
        img = imread(img_path);
        move_inside_upon_opening(R, img);
        break;
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf);
    % [area, cent, hue, img_sz] = get_initial_object_info(img_path);
end

function move_inside_upon_opening(R, img)
end

function knock_knock(R)
end

function center_on_object(R, door, img)
end

function [door, found] = get_door(img)
end

function turn_ninety_degrees(R, turn)
end

% Called when a door is nearing the robot to move the robot in front of it
function move_to_front_of_door(R)
    DistanceSensorRoomba(R);
    m_to_door_front = 8;
    
    while m_to_door_front > 0
        SetFwdVelRadiusRoomba(R, .1, inf);
        pause(0.1);
        m_to_door_front = m_to_door_front - DistanceSensorRoomba(R);
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf);
    pause(1);
end

% Looks for doors that are nearing the edge of the robot's field of view.
% If one is found, turn is set to -1 if the door is on the left, or 1 if
% the door is one the right. If turn remains 0, then the robot is not near
% any doors.
function turn = should_approach_door(img)
    turn = 0;
end

% Keep the robot going straight down the middle of the hallway
function drive_down_hallway(R, img)
    SetFwdVelRadiusRoomba(R, 0.1, inf);
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

% Helper function to sample the HSV at various points on image
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