%==========================================================================
% Instructions:
% Each Segment 1-4 is an autonomous
% block of code that can be run indivudually
%==========================================================================

%==========================================================================
% 1. Basic Navigation
%==========================================================================
% SetFwdVelRadiusRoomba(serPort, 0.5, inf);     % Move Forward Full Speed
% SetFwdVelRadiusRoomba(serPort, -0.5, inf);    % Move Backward Full Speed
% turnAngle(serPort, 0.1, 90)                   % Turn Left
% turnAngle(serPort, 0.1, -90)                  % Turn Right
% SetFwdVelRadiusRoomba(serPort, 0, inf);       % Stop
%==========================================================================


%==========================================================================
% 2. Basic Navigation with Time Constraints
%==========================================================================    
% SetFwdVelRadiusRoomba(serPort, 0.5, inf);      % Move Forward
% pause(1)                                       % Pause for 1 second
% SetFwdVelRadiusRoomba(serPort, 0, inf);        % Stop
%==========================================================================

%==========================================================================
% 3. Read from Sensors
%==========================================================================
% [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers
% display(BumpLeft)                              % Display Left Bumper Value
% display(BumpRight)                             % Display Right Bumper Value
% display(BumpFront)                             % Display Front Bumper Value
% WallSensor = WallSensorReadRoomba(serPort);    % Read Wall Sensor, Requires WallsSensorReadRoomba file    
% display(WallSensor)                            % Display WallSensor Value
%==========================================================================

%==========================================================================
% 4. While Loop with Maximum Time and Distance Sensor
%==========================================================================

function hw3_team_04( serPort )

% Start the timer
tStart= tic;
maxDuration = 120;

% The current position and angle
pos = [0 0];
angle = 0;

% Constants
diameter = .335;

% Reset sensors
BumpsWheelDropsSensorsRoomba(serPort);
DistanceSensorRoomba(serPort);
AngleSensorRoomba(serPort);

% Assume starting size grid, 0 = unvisited, 1 = open, -1 = closed
map = zeros(10);

while toc(tStart) < maxDuration
    % Get sensor values
    [bumpRight, bumpLeft, ~, ~, ~, bumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    contact = bumpRight || bumpLeft || bumpFront;
    %wallSensor = WallSensorReadRoomba(serPort);

    % Add coordinate to map matrix
    map = updateMap(map, contact, pos, diameter);
    
    if contact
        minAngle = 0;
        maxAngle = 180;
        if bumpLeft
            minAngle = maxAngle * -1;
            maxAngle = 0;
        end
        x = randi([minAngle, maxAngle]);
        turnAngle(serPort, 0.2, x);
    end
    
    SetFwdVelRadiusRoomba(serPort, 0.3, inf);
    
    pause(.1);
    
    % Update position based on the new orientation and distance travelled
    dist_travelled = DistanceSensorRoomba(serPort);
    angle = angle + AngleSensorRoomba(serPort);
    pos = updatedPosition(pos, dist_travelled, angle);
    
    display(pos);
    display(angle);
    display(map);
    
end

fillObjects(map);
HeatMap(-map); %so red means obstacle
SetFwdVelRadiusRoomba(serPort, 0, inf);

end

% Returns the new position of the robot 
function pos = updatedPosition(last_pos, dist_travelled, angle)

pos = [0 0];
pos(1) = last_pos(1) + dist_travelled * cos(angle);
pos(2) = last_pos(2) + dist_travelled * sin(angle);

end


% Updates map
function map = updateMap(map, contact, pos, diameter)

pos_floor = floor(pos/diameter);
% Doing length(map)/2 assumes length(map) is an even number
index = pos_floor + length(map)/2 + 1;

% update if cell is open or unvisted
if (map(index(2), index(1)) > -1)
    if contact
        map(index(2), index(1)) = -1;
    else
        map(index(2), index(1)) = 1;
    end
end

end

% Sets grid spaces as closed if they are surrounded by closed spaces
function map = fillObjects(map)

sz = size(map);

% Only consider matrix positions that have entries all around them
for i=2:sz(1)-1
    for j=2:sz(2)-1
        map = fillPosition(map, i, j);
    end
end

end

function map = fillPosition(map, i, j)

if map(i,j) == -1 || map(i,j) == 1;
    return;
end

if map(i-1,j) == -1 && map(i+1,j) == -1 && map(i,j+1) == -1 && map(i,j-1) == -1
    map(i,j) = -1;
end

end