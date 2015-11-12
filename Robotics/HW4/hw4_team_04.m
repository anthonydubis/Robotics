function hw4_team_04( R )
    % @param R - the robot (serPort)
    
    [start, goal] = getStartAndGoal();

end

% Get the Start and Goal points
function [start, goal] = getStartAndGoal()
    data = importdata('hw4_start_goal.txt');
    start = data(1,:);
    goal = data(2,:);
end

