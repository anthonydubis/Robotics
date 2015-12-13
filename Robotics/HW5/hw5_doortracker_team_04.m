function hw5_doortracker_team_04( R )
    close all;
    
    %img_path = 'ex/im14.png';
    img_path = 'http://192.168.0.101/snapshot.cgi?user=admin&pwd=&resolution=10&rate=0';
    
    turn = 0;
    
    while true
        % Find a door to approach
        while true
            img = imread(img_path);
            turn = should_approach_door(img);
            if turn
                % Stopping can likely be removed since we're going to go
                % forward a bit more anyway, but do it for debugging
                SetFwdVelRadiusRoomba(R, 0, inf);
                pause(1);
                
                % DEBUG POINT - There is a door on the left or right most
                % side of our field of view.
                %return;
                
                break;
            end
            drive_down_hallway(R, img);
            pause(0.1);
        end

        move_to_front_of_door(R);
        
        % DEBUG POINT - We should now be in front of the door, but still
        % facing down the hallway. Look in move_to_front_of_door() to play
        % with the distance we travel here (it's a guess)
        %return;
        
        turn_ninety_degrees(R, turn);
        
        img = imread(img_path);
        [door_cent, found] = get_door_cent(img);
        if found
            center_on_object(R, door_cent, img);
        end
        
        knock_knock(R);
        
        % DEBUG POINT - We should have just bumped up against the door
        % twice, then beeped.
        %return;
        
        move_inside_upon_opening(R, img_path);
        
        % DEBUG POINT - Once the door is open, the robot should move into
        % the room.
        
        break;
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf);
end

% Assumes we're facing an open doorway
function move_inside(R)
    DistanceSensorRoomba(R);
    m_to_move = 1.5;
    
    while m_to_move > 0
        SetFwdVelRadiusRoomba(R, .1, inf);
        pause(0.1);
        m_to_move = m_to_move - DistanceSensorRoomba(R);
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf);
    pause(1);
end

% Assumes the robot is facing the door, which is captured in the image, and
% is waiting for it to open
function move_inside_upon_opening(R, img_path)
    open = false;
    
    img = imread(img_path);
    sz = size(img);
    img_sz = sz(1) * sz(2);
    base_gray = rgb2gray(img);
    
    % Do nothing while the door is still in our face
    while ~open
        curr_gray = rgb2gray(imread(img_path));
        
        diff = base_gray - curr_gray;
        diff = abs(diff);
        diff = im2bw(diff, 0.20);
        
        s = sum(sum(diff));
        if (s / img_sz > .20)
            open = true;
        end
    end
    
    move_inside(R);
end

function move_to_bump(R)
    while true
        [br, bl, ~, ~, ~, bf] = BumpsWheelDropsSensorsRoomba(R);
        contact = br || bl || bf;
        
        if contact 
            break;
        end
        
        SetFwdVelRadiusRoomba(R, 0.1, inf);
        pause(0.1);
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf);
    pause(1);
end

function back_up(R)
    DistanceSensorRoomba(R);
    m_to_back_up = -.25;
    
    while m_to_back_up < 0
        SetFwdVelRadiusRoomba(R, -.1, inf);
        pause(0.1);
        m_to_back_up = m_to_back_up - DistanceSensorRoomba(R);
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf);
    pause(1);
end

% At this point, we should be centered on the door
function knock_knock(R)
    % Reset sensors
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(R);
    
    move_to_bump(R);
    back_up(R);
    move_to_bump(R);
    back_up(R);
    BeepRoomba(R);
end

% Returns the degrees current centroid is off from the target centroid
% t_cent is the target centroid (x,y with the top left being the origin)
% n_cent is the new centroid (x,y with the top left being the origin)
% img_sz is the image resolution (height x width in MATLAB)
% Per the camera, the field of view is 67 degrees
function degrees = degrees_misaligned(t_x, n_x, img_sz)
    field_of_view = 67;
    img_width = img_sz(2);
    pixels_per_degree = img_width / field_of_view;
    diff = n_x - t_x;
    degrees = diff / pixels_per_degree;
end

function center_on_object(R, door_cent, img)
    img_sz = size(img);
    degrees = degrees_misaligned(img_sz(2)/2, door_cent(1), img_sz);
    turnAngle(R, 0.1, -degrees);
end

function [door_cent, found] = get_door_cent(img)
    door_cent = [0 0];
    found = false;
    
    BW = process_hsv_image_for_doors(rgb2hsv(img));
    stats = regionprops(BW, 'FilledArea', 'BoundingBox');
    
    max_area = 0;
    for i=1:length(stats)
        obj = stats(i);

        if obj.FilledArea > max_area
            max_area = obj.FilledArea;
            bb = obj.BoundingBox;
            door_cent = [(bb(1)+bb(3)/2) (bb(2)+bb(4)/2)];
            found = true;
        end
    end
end

% If turn is -1, turn left. If turn is 1, turn left. Otherwise, do nothing.
function turn_ninety_degrees(R, turn)
    if turn == -1
        turnAngle(R, 0.1, 90)
    elseif turn == 1
        turnAngle(R, 0.1, -90)
    end
end

% Called when a door is nearing the robot to move the robot in front of it.
% m_to_door_front is initially set to a distance we think we need to travel
% to get us in front of the door.
function move_to_front_of_door(R)
    DistanceSensorRoomba(R);
    m_to_door_front = 3.2;
    
    while m_to_door_front > 0
        SetFwdVelRadiusRoomba(R, .1, inf);
        pause(0.1);
        m_to_door_front = m_to_door_front - DistanceSensorRoomba(R);
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf);
    pause(1);
end

% Looks for doors that are nearing the edge of the robot's field of view,
% either the left or the right most sides, according to some threshold.
% If one is found, turn is set to -1 if the door is on the left, or 1 if
% the door is one the right. If turn remains 0, then the robot is not near
% any doors.
function turn = should_approach_door(img)
    turn = 0;

    img_sz = size(img);
    BW = process_hsv_image_for_doors(rgb2hsv(img));
    stats = regionprops(BW, 'BoundingBox', 'Orientation');
    thresh = 5;
    
    % Look for the first object that we find that is on the edge of our
    % field of vision. These objects must be oriented up and down, like a
    % door, rather than side-to-side.
    for i=1:length(stats)
        obj = stats(i);
        % Doors should have orientations close to -90 or +90 because they
        % are up and down. Other objects that aren't doors but still appear
        % in the process image due to having a similar HSV will likely have
        % orientations close to 0 (side-to-side)
        if abs(obj.Orientation) > 30
            bb = obj.BoundingBox;
            if bb(1) < thresh
                turn = -1;
                break;
            elseif (bb(1) + bb(3)) > (img_sz(2) - thresh)
                turn = 1;
                break;
            end;
        end
    end
end

% Keep the robot going straight down the middle of the hallway
function drive_down_hallway(R, img)
    SetFwdVelRadiusRoomba(R, 0.1, inf);
end

% Accepts the HSV image and returns a BW image with white objects being
% potential doors
function processed = process_hsv_image_for_doors(hsv)
    % Create BW image with 1s where the image has the hue within range of
    % the user's specification and the saturation/value is not close to 0
    processed = hsv(:,:,1) > .56 & hsv(:,:,1) < .7 & hsv(:,:,2) > .25 ...
        & hsv(:,:,2) < .44 & hsv(:,:,3) > .21 & hsv(:,:,3) < .48;
    
    % Remove noise
    processed = bwareaopen(processed, 300);
    
    imshow(processed);
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