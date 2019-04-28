% Copyright 2018, Michael Otte
%
% Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
%
%The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
%
%THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%
% this will display the search tree and path
% assuming that the files have been generated

clear all
close all
clc

problemNum = 5    %% CHANGE THIS
nodeOffset = 0;

search_tree_raw = csvread(['search_tree_' num2str(problemNum + nodeOffset,'%d') '.txt']);
path_raw = csvread(['output_path_' num2str(problemNum + nodeOffset,'%d') '.txt']);
nodes_raw = csvread(['nodes_' num2str(problemNum + nodeOffset,'%d') '.txt']);
edges_raw = csvread(['edges_' num2str(problemNum + nodeOffset,'%d') '.txt']);
obstacles_raw = csvread('obstacles.txt');
vehicle_raw = csvread('robot.txt');

% specify goal region [x, y, radius] and start point [x, y]
start_points = [
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    5, 65;
    ];
goal_regions = [
    100, 60, 2.5;
    113.7, 68.2, 2.5;
    117.4, 79.3, 2.5;
    107.6, 91.5, 2.5;
    91.8, 92, 2.5
    82.25, 90.3, 2.5;
    70.3, 91, 2.5;
    53.749515, 90.058330, 2.5;
    41.748312, 92.299849, 2.5
    29.412292, 90.071815, 2.5;
    18.527725, 83.437182, 2.5;
    12.174539, 70.981114, 2.5
    ];

gate_region = [5, 60, 5];

% a bit of data processing for faster plotting
search_tree = nan(3*size(search_tree_raw, 1), 2);

search_tree(1:3:end-2, 1) = search_tree_raw(:, 2);
search_tree(2:3:end-1, 1) = search_tree_raw(:, 5);
search_tree(1:3:end-2, 2) = search_tree_raw(:, 3);
search_tree(2:3:end-1, 2) = search_tree_raw(:, 6);

nodes = nodes_raw(2:end,[2,3,9]);
edges_raw = edges_raw(2:end,:);
edges = nan(3*size(edges_raw, 1), 2);

for i = 1:length(edges_raw(:,1))
    edges_raw(i, 1) = find(nodes(:,3) == edges_raw(i, 1));
    edges_raw(i, 2) = find(nodes(:,3) == edges_raw(i, 2));
end

edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);

% edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
% edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
% edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
% edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);


figure('Renderer', 'painters', 'Position', [50 50 600 600])

% plct the goal region
pos = [
    (goal_regions(problemNum,1) - goal_regions(problemNum,3)),...
    (goal_regions(problemNum,2) - goal_regions(problemNum,3)),...
    goal_regions(problemNum,3) * 2,...
    goal_regions(problemNum,3) * 2
    ]; 
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[0 .5 .5],...
    'EdgeColor','b', 'LineWidth',2)

hold on

plot(nodes(:,1), nodes(:,2), '.k')
plot(edges(:,1), edges(:,2), 'k')
plot(search_tree(:, 1), search_tree(:, 2), 'm', 'LineWidth', 0.5);


axis([-35.000000, 140.000000, 15.000000, 100.000000])


% plot obstacles
for i = 1:length(obstacles_raw(:,1))
    circle(obstacles_raw(i,1), obstacles_raw(i,2), obstacles_raw(i,3))
end


% plot the gate region
pos = [
    (gate_region(1) - gate_region(3)),...
    (gate_region(2) - gate_region(3)),...
    gate_region(3) * 2,...
    gate_region(3) * 2
    ]; 
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor','r',...
    'EdgeColor','k', 'LineWidth',3)

plot(start_points(problemNum,1), start_points(problemNum,2), 'or', 'MarkerSize', 10,...
    'MarkerFaceColor', 'r')
plot(path_raw(:,2), path_raw(:,3), 'g:', 'LineWidth', 4);

title('Path Search Tree')
xlabel('X-Coordinate (m)')
ylabel('Y-Coordinate (m)')
hold off


