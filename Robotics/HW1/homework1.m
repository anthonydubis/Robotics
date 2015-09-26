function homework1( serPort )
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
% WallSensor = WallSensorReadRoomba(serPort);    % Read Wall Sensor, Requires WallSensorReadRoomba file    
% display(WallSensor)                            % Display WallSensor Value
%==========================================================================

%==========================================================================
% 4. While Loop with Maximum Time and Distance Sensor
%==========================================================================

% Start the timer and set initial distance
tStart= tic;
maxDuration = 600;

% Specify how close we have to come back to the starting point to finish
thresh = .05;

% Set initial values
pos = [0 0];
curr_angle = 0;
tracingObject = false;

% A global variable that determines if a Create has left the region where 
% it initially had contact with the object. Used so that we don't say the
% robot has returned to the starting point at the beginning of the journey
global hasLeftIntialContactRegion;
hasLeftIntialContactRegion = false;

% Reset sensors
[~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);

while toc(tStart) < maxDuration
    % Get and display sensor values
    [bumpRight, bumpLeft, ~, ~, ~, bumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    wallSensor = WallSensorReadRoomba(serPort);
    
    if ~tracingObject
        if bumpRight || bumpLeft || bumpFront || wallSensor
            % We hit an object, time to begin tracing
            tracingObject = true;
            % Reset the angle and distance sensors
            AngleSensorRoomba(serPort);
            DistanceSensorRoomba(serPort);
        else
            % Keep moving forward until we hit an object
            SetFwdVelRadiusRoomba(serPort, 0.3, inf);
        end
    else
        if bumpRight || bumpLeft || bumpFront
            % We're in contact with an object, rotate left 
            SetFwdVelRadiusRoomba(serPort, -.5, inf);
            pause(.05);
            turnAngle(serPort, .2, 30);
            SetFwdVelRadiusRoomba(serPort, .1, inf);
        elseif wallSensor
            % The wall is still close - go straight to keep tracing
            SetFwdVelRadiusRoomba(serPort, 0.4, inf);
            % TRY - longer pause here, shorter pause elsewhere
        else
            % We've lost bumper contact and broken the wall sensor, turn
            % back right to re-engage
            SetFwdVelRadiusRoomba(serPort, .1, -.1);
        end
        
        pause(0.05);
        
        % Update the new position based on the new orientation and distance
        % travelled
        dist_travelled = DistanceSensorRoomba(serPort);
        curr_angle = curr_angle + AngleSensorRoomba(serPort);
        pos = updatedPosition(pos, dist_travelled, curr_angle);
        
        % Break out of the loop if we've returned to the starting point
        if hasReturned(pos, thresh)
            break;
        end
    end
end

SetFwdVelRadiusRoomba(serPort, 0, inf);

end

% Returns the new position of the robot 
function pos = updatedPosition(last_pos, dist_travelled, curr_angle)

pos = [0 0];
pos(1) = last_pos(1) + dist_travelled * cos(curr_angle);
pos(2) = last_pos(2) + dist_travelled * sin(curr_angle);

end

% Returns the Euclidean distance from the (0, 0) origin.
function dist = distanceFromOrigin(pos)

dist = sqrt(pos(1)^2 + pos(2)^2);

end

% pos - the last position of the robot
%
% thresh - a threshold distance from the origin the robot must be within to
% have returned to its starting point
%
% returned - true if the Create has returned to its initial point of
% impact after circumnavigating the object
function returned = hasReturned(pos, thresh)

global hasLeftIntialContactRegion;
returned = false;

if ~hasLeftIntialContactRegion
    if distanceFromOrigin(pos) > thresh
        % The Create has left its initial point of contact - it can
        % terminate when it returns to this area again
        hasLeftIntialContactRegion = true;
    end
else
    if distanceFromOrigin(pos) < thresh
        % The Create is back to where its started within the distance threshold
        returned = true;
    end
end

end