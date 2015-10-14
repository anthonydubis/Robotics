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

% Cache information for graphing
pos_hist = [];
ang_hist = [];
idx = 1;

% Cache the location we hit an obstacle, [NaN NaN] implies we aren't
% navigating an obstacle right now.
poc = [NaN NaN];

% True when navigating an object
isNavigatingObstacle = false;

% A threshold distance to be within when determining proximity to a point
thresh = 0.1;

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
    
    if hasReachedGoal(pos, 2*thresh)
        % Made it!
        break;
    end
    % Break out out if returned back to point of contact
    if hasLeftInitialPOC
        if pointsAreSimilar(pos, poc, thresh)
            % We've returned to our point of contact - we're bounded
            fprintf('Bounded by an obstacle - terminating\n');
            break;
        end
    end
    
    % There's nothing in our path and we aren't navigation an obstacle
    if ~isNavigatingObstacle && ~contact
        % Not home yet - go forward
        headTowardsGoal(serPort, angle);
    
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
    
    pos_hist = [pos_hist; idx pos];
    ang_hist = [ang_hist; idx angle];
    idx = idx + 1;
end

SetFwdVelRadiusRoomba(serPort, 0, inf);

plot_progress(pos_hist, ang_hist);
plot_position(pos_hist);

end

% A function that plots (x, y) coordinates the robot reached
%
% pos_hist - N x 3 matrix, each entry is an (n, x, y) where n is the
% iteration number, x is the x position, and y is the y position.
function plot_position(pos_hist)

figure; hold on;

axis([-5 5 -5 5]);
title('(X, Y) Coordinates Reached'); 
legend('(x,y) coordinate');
xlabel('Meters');
ylabel('Meters');

for i=1:size(pos_hist,1)
    pt = plot(pos_hist(:,2), pos_hist(:,3), 'o', 'MarkerEdgeColor', 'r');
    set(pt, 'MarkerSize', 2, 'LineWidth', 2);
end

hold off;

end

% A function that plots the robots progress by changes in distance and
% orientation
%
% pos_hist - N x 3 matrix, each entry is an (n, x, y) where n is the
% iteration number, x is the x position, and y is the y position.
%
% ang_hist - N x 1 vector, each entry is the orientation (angle in radians) 
% of the robot relative to its goal (0 radians means facing the goal)
function plot_progress(pos_hist, ang_hist)

figure; hold on;

axis([0 size(pos_hist,1) -5 5]);
title('Progress Towards Goal (0-4) and Orientation (Angle in Radians)'); 
legend('Distance Traveled to Goal (meters)','Orientation (radians)');
xlabel('Iteration Number');
ylabel('Meters for Progress, Radians for Orientation');

for i=1:size(pos_hist,1)
    pt = plot(pos_hist(:,1), pos_hist(:,2), 'o', 'MarkerEdgeColor', 'r');
    set(pt, 'MarkerSize', 2, 'LineWidth', 2);
    
    ang = plot(ang_hist(:,1), ang_hist(:,2), 'x', 'MarkerEdgeColor', 'b');
    set(ang, 'MarkerSize', 2, 'LineWidth', 2);
end

hold off;

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
    turnAngle(serPort, 0.2, -(angle * 180 / pi / 4));
else
    SetFwdVelRadiusRoomba(serPort, .3, inf);
end

end

% Returns true if our y is within some threshhold of 0
function discovered = hasRediscoveredMLine(pos, poc, thresh)

discovered = false;

global hasLeftInitialPOC;

if ~hasLeftInitialPOC
    if ~isnan(poc(1))
        if abs(pos(1) - poc(1)) > thresh*2 || abs(pos(2) - poc(2)) > thresh*2
            % We have left our initial point of contact
            hasLeftInitialPOC = true;
        end
    end
elseif pos(1) > poc(1) && abs(pos(2)) < thresh && pos(1) < 4 + thresh
    discovered = true;
end

end

% Returns true if we are within some threshhold of our goal
function reached = hasReachedGoal(pos, thresh)

reached = false;
if pos(1) > 4
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
    SetFwdVelRadiusRoomba(serPort, -0.05, inf);
    pause(0.1);
    turnAngle(serPort, .2, 5);
    SetFwdVelRadiusRoomba(serPort, 0.1, inf);
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