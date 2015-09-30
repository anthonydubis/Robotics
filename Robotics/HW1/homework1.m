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

% Update this variable based on whether running on the simulator
isSimulator = true;

% Configurations based on isSimulator boolean
% thresh is how close we need to get back to our initial point of contact
thresh = 0.5;
if isSimulator 
    thresh = 0.3; 
end

% backwards velocity used after running into an object
back_vel = -0.1;
if isSimulator
    back_vel = 0.0;
end

% backwards pause
backwards_pause = 0.05;
if isSimulator
    backwards_pause = 0.0;
end

% Initial forward velocity
initial_vel = 0.2;
if isSimulator
    initial_vel = 0.45;
end

% Left turn angle
left_turn_angle = 15;
if isSimulator
    left_turn_angle = 5;
end

% Forward velocity after a left turn
left_forw_vel = .15;
if isSimulator
    left_forw_vel = .05;
end

% Right turn angle
right_turn_angle = -10;
if isSimulator
    right_turn_angle = -7;
end

% Forward velocity after a right turn
right_forw_vel = .15;
if isSimulator
    right_forw_vel = .2;
end

end_of_loop_pause = 0.0;
if isSimulator
    end_of_loop_pause = 0.1;
end

% Start the timer and set initial distance
tStart= tic;
maxDuration = 600;

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

% Set the robot to move forward
SetFwdVelRadiusRoomba(serPort, initial_vel, inf);

while toc(tStart) < maxDuration
    % Get and display sensor values
    [bumpRight, bumpLeft, ~, ~, ~, bumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    wallSensor = WallSensorReadRoomba(serPort);
    
    if ~tracingObject
        if bumpRight || bumpLeft || bumpFront || wallSensor
            % We hit an object, time to begin tracing
            tracingObject = true;
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            % Reset the angle and distance sensors
            AngleSensorRoomba(serPort);
            DistanceSensorRoomba(serPort);
        end
    else
        if bumpRight || bumpLeft || bumpFront
            % We're in contact with an object, rotate left 
            SetFwdVelRadiusRoomba(serPort, back_vel, inf);
            pause(backwards_pause);
            turnAngle(serPort, .2, left_turn_angle);
            SetFwdVelRadiusRoomba(serPort, left_forw_vel, inf);
            pause(0.05);
        elseif wallSensor
            % The wall is still close - go straight to keep tracing
            SetFwdVelRadiusRoomba(serPort, 0.4, inf);
            % TRY - longer pause here, shorter pause elsewhere
            pause(0.05);
        else
            % We've lost bumper contact and broken the wall sensor, turn
            % back right to re-engage
            turnAngle(serPort, .2, right_turn_angle);
            SetFwdVelRadiusRoomba(serPort, right_forw_vel, inf);
            pause(0.05);
        end
        
        pause(end_of_loop_pause);
        
        % Update the new position based on the new orientation and distance
        % travelled
        dist_travelled = DistanceSensorRoomba(serPort);
        curr_angle = curr_angle + AngleSensorRoomba(serPort);
        pos = updatedPosition(pos, dist_travelled, curr_angle);
        display(pos);
        
        % Break out of the loop if we've returned to the starting point
        if hasReturned(pos, thresh)
            SetFwdVelRadiusRoomba(serPort, 0, inf);
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