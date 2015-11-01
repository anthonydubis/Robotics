%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 1
%
% Team number: 4
% Team leader: Anthony Dubis (ajd2194)
% Team members: Lilly Wang (lfw2114), Samir Mathrani (sm3619)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw3_team_04( serPort )

% Constants
diameter = .335;
initialSpiralTurnRadius = 0.1;
spiralRadiusIncrement = 0.003;

% Start the timer
tStart= tic;
t_last_disc = tic;
maxDuration = 300;
last_disc_thresh = 60;

% The current position, angle, and last point of contact
pos = [0 0];
angle = 0;
lastPOC = [0 0];

% Variables for spiralling functionality
isSpiralling = true;
turnRadius = initialSpiralTurnRadius;

% Reset sensors
BumpsWheelDropsSensorsRoomba(serPort);
DistanceSensorRoomba(serPort);
AngleSensorRoomba(serPort);

% Assume starting size grid, 0 = unvisited, 1 = open, -1 = closed
map = zeros(2);

while toc(tStart) < maxDuration && toc(t_last_disc) < last_disc_thresh
    [bRight, bLeft, ~, ~, ~, bFront] = BumpsWheelDropsSensorsRoomba(serPort);
    contact = bRight || bLeft || bFront;

    % Add value to map matrix if necessary
    [map, wasUpdated] = updateMap(map, bLeft, bRight, bFront, pos, diameter, angle);
    if wasUpdated
        t_last_disc = tic;
    end
    
    if contact 
        % Hit an obstacle
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        isSpiralling = false;
        lastPOC = pos;
        randomTurn(serPort, bLeft);

    elseif isSpiralling 
        % Should continue spirallying
        SetFwdVelRadiusRoomba(serPort, 0.2, turnRadius);
        turnRadius = turnRadius + spiralRadiusIncrement;
        
        % Stop spiralling if our turn radius has grown too large
        if turnRadius > 1.9
            isSpiralling = false;
            lastPOC = pos;
        end
        
    else
        % Go straight
        SetFwdVelRadiusRoomba(serPort, 0.3, inf);
        
        % Begin spiralling after moving away from last POC
%         if distanceFromPoint(lastPOC, pos) > diameter * 8 && toc(t_last_disc) < 1.5
%             isSpiralling = true;
%             turnRadius = initialSpiralTurnRadius;
%         end
        
        % Begin spiralling when a certain amount of cells around the robot
        % are unvisited
        spiral = shouldSpiral(map, pos, diameter);
        if spiral
            isSpiralling = true;
            turnRadius = initialSpiralTurnRadius;
        end
    end
    
    pause(.1);
    
    % Update position based on the new orientation and distance travelled
    dist_travelled = DistanceSensorRoomba(serPort);
    angle = angle + AngleSensorRoomba(serPort);
    pos = updatedPosition(pos, dist_travelled, angle);
    
%     Debugging
%     display(pos);
%     display(angle);
%     display(map);    
end

HeatMap(-map); %so red means obstacle
SetFwdVelRadiusRoomba(serPort, 0, inf);

end

% The euclidean distance between p1 and p2
% @p1: [x y]
% @p2: [x y]
function dist = distanceFromPoint(p1, p2)
dist = norm(p1 - p2);
end

% Determines if the robot should start spiraling or not
% @map: the current coordinate space
% @pos: the position of the robot
% @diameter: the diameter of the robot
function spiral = shouldSpiral(map, pos, diameter)

pos_floor = floor(pos/diameter);
idx = pos_floor + length(map)/2 + 1;

n = 1;
num_unvisited = 0;
thresh = 3;
spiral = false;

while n <= thresh
    % Break if we will access indicies outside of the map
    if idx(1) - n < 1 || idx(2) - n < 1 || idx(1) + n > length(map) || idx(2) + n > length(map)
        break;
    end
    
    if map(idx(2), idx(1) - n) == 0
        num_unvisited = num_unvisited + 1;
    end
    if map(idx(2), idx(1) + n) == 0
        num_unvisited = num_unvisited + 1;
    end
    if map(idx(2) - n, idx(1)) == 0
        num_unvisited = num_unvisited + 1;
    end
    if map(idx(2) + n, idx(1)) == 0
        num_unvisited = num_unvisited + 1;
    end
    
    n = n + 1;
end

if num_unvisited >= (thresh * 4 * .70)
    spiral = true;
end

end

% Make a random turn
% @bLeft = true if the left bumper is in contact
function randomTurn(serPort, bLeft)

x = randi([30 160]);
if bLeft
    x = x * -1;
end
turnAngle(serPort, 0.2, x);     

end


% Returns the new position of the robot 
% @last_pos: the last position of the robot
% @dist_travelled: how far the robot has traveled since the last update
% @angle: the current orientation of the robot since beginning
function pos = updatedPosition(last_pos, dist_travelled, angle)

pos = [0 0];
pos(1) = last_pos(1) + dist_travelled * cos(angle);
pos(2) = last_pos(2) + dist_travelled * sin(angle);

end


% Assumes we are currently in contact with an obstacle
% Returns the estimated position of that obstacle
function poc = contactPosition(bLeft, bRight, bFront, pos, diameter, angle)

if bFront
    % Don't adjust the angle
elseif bLeft
    angle = angle + pi / 4;
elseif bRight
    angle = angle - pi / 4;
end

poc = updatedPosition(pos, diameter, angle);

end


% Updates map positions with appropriate value (-1 closed, 1 open)
% @map: the current coordinate space
% @bLeft, @bRight, @bFront: which bumper is pressed in.
% @pos: the position of the robot
% @diameter: the diameter of the robot
% @angle: the current orientation
% @wasUpdated: true if a value in map changes, false otherwise
%
% Once a grid cell is set to closed, it is closed forever.  However, a cell
% that has been set to open may later be closed if we run into an object in
% that position later.
function [map, wasUpdated] = updateMap(map, bLeft, bRight, bFront, pos, diameter, angle)

wasUpdated = false;
contact = bLeft || bRight || bFront;

if contact
    pos = contactPosition(bLeft, bRight, bFront, pos, diameter, angle);
end

pos_floor = floor(pos/diameter);
idx = pos_floor + length(map)/2 + 1;

if idx(1) < 1 || idx(2) < 1 || idx(1) > length(map) || idx(2) > length(map)
    map = doubleMap(map);
    idx = pos_floor + length(map)/2 + 1;
end

% Update if cell is open or unvisted
prev = map(idx(2), idx(1));
if (prev > -1)
    if contact
        map(idx(2), idx(1)) = -1;
    else
        map(idx(2), idx(1)) = 1;
    end
end

if prev ~= map(idx(2), idx(1))
    wasUpdated = true;
end

end


% Quadruples the size of the map, putting the original map in the center
function new_map = doubleMap(map)

s = length(map);
new_map = zeros(2*s);
new_map(s/2+1:3*s/2, s/2+1:3*s/2) = map;

end
