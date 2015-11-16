%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 4
%
% Team number: 4
% Team leader: Anthony Dubis (ajd2194)
% Team members: Lilly Wang (lfw2114), Samir Mathrani (sm3619)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% @param R - the robot (serPort)
function hw4_team_04( R )

    % Get the ordered vertices
    vertices = importdata('path.txt');
    
    n = size(vertices, 1);       % # of vertices
    tvi = 1;                     % The current target vertex index
    curr_pos = vertices(tvi,:);  % The current position of the robot
    curr_angle = pi / 2;         % The current angle - start facing +y
    thresh = 0.1;                % Threshold distance for similarity
    
    while tvi <= n
        % Get the target point
        target = vertices(tvi,:);
        
        % Test to see if we've reached the target point
        if pointsAreSimilar(curr_pos, target, thresh)
            % If we've reached the last point, we're at the goal
            if tvi == n
                break;
            end
            
            % Move to the next point
            tvi = tvi + 1;
            target = vertices(tvi,:);
        end
        
        turnTowardsTarget(R, curr_pos, target, curr_angle);
        SetFwdVelRadiusRoomba(R, 0.15, inf);
        pause(0.1);
        
        dist_travelled = DistanceSensorRoomba(R);
        curr_angle = curr_angle + AngleSensorRoomba(R);
        curr_pos = updatedPosition(curr_pos, dist_travelled, curr_angle);
    end
    
    SetFwdVelRadiusRoomba(R, 0, inf)
end

% This function will turn the robot towards the goal if it's current
% orientation is off by more than 5 degrees. This function is called with
% each iteration so the robot can adjust its orientation if it's gone off
% course.
function turnTowardsTarget(R, curr_pos, target, curr_angle)
    theta = atan2(target(2) - curr_pos(2), target(1) - curr_pos(1));
    turn_angle = theta - curr_angle;
    
    % Adjust our angle if we're off by more than 5 degrees
    if abs(turn_angle) > .087
        turnAngle(R, 0.1, (turn_angle * 180 / pi));
    end
end

% Returns true if p1 & p2 are within thresh meters of each other
function similar = pointsAreSimilar(p1, p2, thresh)
    similar = false;
    
    dist = sqrt(sum((p1 - p2) .^ 2));
    if dist < thresh
        similar = true;
    end
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