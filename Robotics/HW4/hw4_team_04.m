function hw4_team_04( R )
    % @param R - the robot (serPort)
    
    [world, obstacles] = get_obstacles()
    [start, goal] = start_and_goal();
    for k=1:length(obstacles)
        obstacles{k}
    end
end

function obstacle = get_obstacle(vertices_vector)
    obstacle = vec2mat(vertices_vector,2);
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
        obstacles{i} = get_obstacle(data(idx+1:idx+lines,:));
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