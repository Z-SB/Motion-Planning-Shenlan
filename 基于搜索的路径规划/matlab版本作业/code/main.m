% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 9.0;
yTarget = 9.0;
MAX_X = 10;
MAX_Y = 10;
% store the cooridinates of obstacle including start and goal point.
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
% map = load('Data/Dense_map.mat');
% Waypoint Generator Using the A* 
[path,OPEN] = A_star_search(map, MAX_X,MAX_Y,3);
% Get the explored nodes along the search
% explored_node =  explored_nodes(OPEN);used to compare
% visualize the 2D grid map
visualize_map(map, path, OPEN);

% save map
% save('Data/Dense_map.mat', 'map', 'MAX_X', 'MAX_Y');
