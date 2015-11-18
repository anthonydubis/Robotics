%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%COMS W4733 Computational Aspects of Robotics 2015
% Homework 4
%
% Team number: 4
% Team leader: Samir Mathrani (sm3619)
% Team members: Lilly Wang (lfw2114), Anthony Dubis (ajd2194)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Our java program, PathPlanner.java takes care of the path finding. Placing all 
.java files submitted in the same directory and then run the following command
with the appropriate arguments:

java PathPlanner [txt file world and obstacles] [txt file start and goal]

It produces a path.txt file which contains the points in the shortest path.
This file should be placed in the same dir as the MATLAB program and then
the MATLAB program directs the robot to follow the path specified. The MATLAB
function, hw4_team_04( R ), is run by passing in the robot instance as normal.
It looks for the path.txt, which is a hardcoded filename within the function.
