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

% Start the timer and set initial distance
tStart= tic;
maxDuration = 600;

% Set initial values
pos = [0 0];
angle = 0;

% Reset sensors
BumpsWheelDropsSensorsRoomba(serPort);
DistanceSensorRoomba(serPort);
AngleSensorRoomba(serPort);

% Set the robot to move forward
SetFwdVelRadiusRoomba(serPort, .3, inf);

while toc(tStart) < maxDuration
    % Get sensor values
    [bumpRight, bumpLeft, ~, ~, ~, bumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    wallSensor = WallSensorReadRoomba(serPort);
    
    % Update position based on the new orientation and distance travelled
    dist_travelled = DistanceSensorRoomba(serPort);
    angle = angle + AngleSensorRoomba(serPort);
    pos = updatedPosition(pos, dist_travelled, angle);
    
    display(pos);
    display(angle);
    pause(0.1);
end

end

% Returns the new position of the robot 
function pos = updatedPosition(last_pos, dist_travelled, angle)

pos = [0 0];
pos(1) = last_pos(1) + dist_travelled * cos(angle);
pos(2) = last_pos(2) + dist_travelled * sin(angle);

end
