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
map = zeros(14);

while toc(tStart) < maxDuration
    % Get sensor values
    [bRight, bLeft, ~, ~, ~, bFront] = BumpsWheelDropsSensorsRoomba(serPort);
    contact = bRight || bLeft || bFront;
    %wallSensor = WallSensorReadRoomba(serPort);

    % Add coordinate to map matrix
    map = updateMap(map, bLeft, bRight, bFront, pos, diameter, angle);
    
    if contact
        randomTurn(serPort, bLeft);
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

% Make a random turn
% @bLeft = true if the left bumper is in contact
function randomTurn(serPort, bLeft)

x = randi([30 225]);
if bLeft
    x = x * -1;
end
turnAngle(serPort, 0.2, x);     

end


% Returns the new position of the robot 
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


% Updates map
function map = updateMap(map, bLeft, bRight, bFront, pos, diameter, angle)

contact = bLeft || bRight || bFront;

if contact
    pos = contactPosition(bLeft, bRight, bFront, pos, diameter, angle);
end

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