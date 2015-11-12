function hw4_team_04( R )
    % @param R - the robot (serPort)
    
    [world, obstacles] = get_obstacles();
    [start, goal] = start_and_goal();
end

% Get the obstacles from the provided file
% @retval world: the vertices for the world
% @retval obstacles: a cell of matrixes representing the obstacle veritices
function [world, obstacles] = get_obstacles()
    data = importdata('hw4_world_and_obstacles_convex.txt');
    n = data(1);
    obstacles = cell(n,1);
    idx = 2;
    i = 1;
    
    while i <= n
        lines = data(idx) * 2;
        obstacles{i} = vec2mat((data(idx+1:idx+lines,:)), 2);
        idx = idx + lines + 1;
        i = i + 1;
    end
    
    world = obstacles{1};
    obstacles(1,:) = [];
end

% Get the Start and Goal points
function [start, goal] = start_and_goal()
    data = importdata('hw4_start_goal.txt');
    start = data(1,:);
    goal = data(2,:);
end