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


function hw2_team_04( serPort )

% Start the timer
tStart= tic;
maxDuration = 600;

% The current position and angle
pos = [0 0];
angle = 0;

% Cache the location we hit an obstacle, [NaN NaN] implies we aren't
% navigating an obstacle right now.
poc = [NaN NaN];

% True when navigating an object
isNavigatingObstacle = false;

% A threshold distance to be within when determining proximity to a point
thresh = 0.05;

% True if we should head towards the goal when the m-line is encountered again
global hasLeftInitialPOC;
hasLeftInitialPOC = false;

% Reset sensors
BumpsWheelDropsSensorsRoomba(serPort);
DistanceSensorRoomba(serPort);
AngleSensorRoomba(serPort);

% Set the robot to move forward
SetFwdVelRadiusRoomba(serPort, .3, inf);

while toc(tStart) < maxDuration
    % Get sensor values
    [bumpRight, bumpLeft, ~, ~, ~, bumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    contact = bumpRight || bumpLeft || bumpFront;
    wallSensor = WallSensorReadRoomba(serPort);
    
    % Break out out if returned back to point of contact
    if hasLeftInitialPOC
        if pointsAreSimilar(pos, poc, thresh)
            % We've returned to our point of contact - we're bounded
            break;
        end
    end
    
    % There's nothing in our path and we aren't navigation an obstacle
    if ~isNavigatingObstacle && ~contact
        if hasReachedGoal(pos, 2*thresh)
            % Made it!
            break;
        else
            % Not home yet - go forward
            headTowardsGoal(serPort, angle);
        end
    
    % We were navigating an object but we rediscovered the M-line
    elseif hasRediscoveredMLine(pos, poc, thresh)
        poc = [NaN NaN];
        isNavigatingObstacle = false;
        headTowardsGoal(serPort, angle);
        
    % We must be navigating an object
    else
        if isnan(poc(1))
            poc = pos;
            hasLeftInitialPOC = false;
        end
        
        % Continue tracing the object
        isNavigatingObstacle = true;
        navigateObstacle(serPort, contact, wallSensor);
    end
    
    pause(0.1);
    
    % Update position based on the new orientation and distance travelled
    dist_travelled = DistanceSensorRoomba(serPort);
    angle = angle + AngleSensorRoomba(serPort);
    pos = updatedPosition(pos, dist_travelled, angle);
    
    display(pos);
    display(poc);
    display(angle);
end

SetFwdVelRadiusRoomba(serPort, 0, inf);

end

function similar = pointsAreSimilar(p1, p2, thresh)

similar = false;

if abs(p1(1) - p2(1)) < thresh && abs(p1(2) - p2(2)) < thresh
    similar = true;
end

end

% Head towards the goal, but correct our angle if we start to veer
function headTowardsGoal(serPort, angle)

if abs(angle) > 0.05
    turnAngle(serPort, 0.2, -(angle * 180 / pi));
else
    SetFwdVelRadiusRoomba(serPort, .3, inf);
end

end

% Returns true if our y is within some threshhold of 0
function discovered = hasRediscoveredMLine(pos, poc, thresh)

discovered = false;

global hasLeftInitialPOC;

if ~hasLeftInitialPOC
    if abs(pos(2)) > thresh
        hasLeftInitialPOC = true;
    end
elseif pos(1) > poc(1) && abs(pos(2)) < thresh && pos(1) < 4 + thresh
    discovered = true;
end

end

% Returns true if we are within some threshhold of our goal
function reached = hasReachedGoal(pos, thresh)

reached = false;
if abs(4 - pos(1)) < thresh && abs(pos(2)) < thresh
    reached = true;
end

end

% Returns the new position of the robot 
function pos = updatedPosition(last_pos, dist_travelled, angle)

pos = [0 0];
pos(1) = last_pos(1) + dist_travelled * cos(angle);
pos(2) = last_pos(2) + dist_travelled * sin(angle);

end

% Tracing the object
function navigateObstacle(serPort, contact, wallSensor)

if contact
    % We're in contact with an object, rotate left 
    
    turnAngle(serPort, .2, 5);
    SetFwdVelRadiusRoomba(serPort, 0.05, inf);
    pause(0.05);
elseif wallSensor
    % The wall is close to our right - go straight to keep tracing
    SetFwdVelRadiusRoomba(serPort, 0.4, inf);
    pause(0.05);
else
    % We've lost bumper contact and/or broken the wall sensor, turn
    % back right to re-engage the obstacle
    turnAngle(serPort, .2, -7);
    SetFwdVelRadiusRoomba(serPort, 0.2, inf);
    pause(0.05);
end

end