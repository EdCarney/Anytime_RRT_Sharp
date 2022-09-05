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

folder = "testData_20220903235850"; %% change this

% generate params and solve problem
service = clib.ArrtsLibrary.ArrtsService();
params = clib.ArrtsLibrary.ArrtsParams(folder, 2.5);
pathStates = service.calculatePath(params);

% plot
path = zeros(pathStates.Dimensions, 2);
for i=1:pathStates.Dimensions
    path(i,1) = pathStates(i).x;
    path(i,2) = pathStates(i).y;
end

obstacles_file = fullfile(folder, 'obstacles.txt');
state_file = fullfile(folder, 'states.txt');

obstacles_raw = readmatrix(obstacles_file);
state_raw = readmatrix(state_file);

% specify goal region [x, y, radius] and start point [x, y]
root_node = state_raw(1,1:2);
uav_start = [state_raw(2,1:2), 2.5];

figure('Renderer', 'painters', 'Position', [50 50 600 600])
hold on

% plot the goal region
pos = [
    (uav_start(1) - uav_start(3)),...
    (uav_start(2) - uav_start(3)),...
    uav_start(3) * 2,...
    uav_start(3) * 2
    ]; 
rectangle('Position',pos,'Curvature',[1 1], 'FaceColor',[0 .5 .5],...
    'EdgeColor','b', 'LineWidth',3)

plot(root_node(1), root_node(2), 'or', 'MarkerSize', 10,...
    'MarkerFaceColor', 'r')
plot(path(:,1), path(:,2), 'g:', 'LineWidth', 4);

% plot obstacles
for i = 1:length(obstacles_raw(:,1))
    circle(obstacles_raw(i,1), obstacles_raw(i,2), obstacles_raw(i,3))
end

title('Path Search Tree')
xlabel('X-Coordinate (m)')
ylabel('Y-Coordinate (m)')
hold off


