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

function hw1_team_04( serPort )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update this boolean based on whether running on the simulator or not

isSimulator = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Configurations based on isSimulator boolean - see function for variable descriptions
[thresh, back_vel, backwards_pause, initial_vel, left_turn_angle, left_forw_vel, right_turn_angle, right_forw_vel, end_of_loop_pause] = getConfig(isSimulator);

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
            % The wall is close to our right - go straight to keep tracing
            SetFwdVelRadiusRoomba(serPort, 0.4, inf);
            pause(0.05);
        else
            % We've lost bumper contact and/or broken the wall sensor, turn
            % back right to re-engage the obstacle
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
        
        % Break out of the loop if we've returned to the starting point
        if hasReturned(pos, thresh)
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            break;
        end
    end
end

% Stop the robot when finished
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This function is used to get the configuration parameters for when
% running on the robot vs on the simulator. The parameters returned are
% based on the boolean passed in.
%
% thresh = distance the robot must be from initial point of contact to stop
% back_vel = velocity of the robot when moving backwards
% backwards_pause = duration to pause after sending move backwards command
% initial_vel = initial velocity for robot when approaching the obstacle
% left_turn_angle = the degree of left turns
% left_forw_vel = the forward velocity of the robot after turning left
% right_turn_angle = the degree of right turns
% right_forw_vel = the forward velocity of the robot after turning right
% end_of_loop_pause = the pause value at the end of the loop when tracing
% an object
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [thresh, back_vel, backwards_pause, initial_vel, left_turn_angle, left_forw_vel, right_turn_angle, right_forw_vel, end_of_loop_pause] = getConfig(isSimulator)

thresh = 0.5;
back_vel = -0.1;
backwards_pause = 0.05;
initial_vel = 0.2;
left_turn_angle = 15;
left_forw_vel = 0.15;
right_turn_angle = -10;
right_forw_vel = 0.15;
end_of_loop_pause = 0;

if isSimulator 
    thresh = 0.3;
    back_vel = 0.0;
    backwards_pause = 0.0;
    initial_vel = 0.45;
    left_turn_angle = 5;
    left_forw_vel = 0.05;
    right_turn_angle = -7;
    right_forw_vel = 0.2;
    end_of_loop_pause = 0.1;
end

end