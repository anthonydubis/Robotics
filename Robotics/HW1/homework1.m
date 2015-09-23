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
maxDuration = 120;

% Reset value in DistanceSensorRoomba by calling it
Initial_Distance = DistanceSensorRoomba(serPort);   % Get the Initial Distance
Total_Distance = 0;                                 % Initialize Total Distance

tracingObject = false;

while toc(tStart) < maxDuration
    % Get and display sensor values
    [bumpRight, bumpLeft, ~, ~, ~, bumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    wallSensor = WallSensorReadRoomba(serPort);
    display(bumpLeft)
    display(bumpRight)
    display(bumpFront)
    display(wallSensor)
    
    if ~tracingObject
        if bumpRight || bumpLeft || bumpFront || wallSensor
            % We hit an object, time to begin tracing
            tracingObject = true;
        else
            % Keep moving forward until we hit an object
            SetFwdVelRadiusRoomba(serPort, 0.3, inf);
            pause(1);
        end
    else
        if bumpRight || bumpLeft || bumpFront
            % We've collided with an object, rotate left 10 degrees at a time until we're off it
            turnAngle(serPort, .2, 5);
            SetFwdVelRadiusRoomba(serPort, .1, inf);
        elseif wallSensor
            % The wall is still close - go straight to keep tracing
            SetFwdVelRadiusRoomba(serPort, 0.4, inf);
            % TRY - longer pause here, shorter pause elsewhere
        else
            % We've lost bumper contact and broken the wall sensor, turn
            % back right to re-engage
            turnAngle(serPort, .2, -5)
            SetFwdVelRadiusRoomba(serPort, .1, inf);
        end
        pause(0.2)
    end
end
%==========================================================================

end