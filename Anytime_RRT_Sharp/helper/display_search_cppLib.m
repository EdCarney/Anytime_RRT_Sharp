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

folder = "testData_paper_1_Dubins3d"; %% change this

path_file = fullfile(folder, 'full_output_path.txt');
path_node_file = fullfile(folder, 'output_path.txt');
obstacles_file = fullfile(folder, 'obstacles.txt');
state_file = fullfile(folder, 'states.txt');

path = readmatrix(path_file);
path_node = readmatrix(path_node_file);
[num_path_nodes, ~] = size(path_node);
state_raw = readcell(state_file);

% specify goal region [x, y, z, radius] and start point [x, y, z]
root_node_array = double(string(state_raw(2)).split(' '));
uav_start_array = double(string(state_raw(3)).split(' '));
root_node = root_node_array(1:3);
uav_start = [uav_start_array(1:3)', uav_start_array(6)];

figure('Renderer', 'painters', 'Position', [50 50 600 600])
hold on

h = zeros(1, 4);

marker_size = 10;

h(1) = plot3(root_node(1), root_node(2), root_node(3), 'square', 'MarkerSize', marker_size,...
    'MarkerFaceColor', 'b', 'DisplayName', 'Initial Position');
h(2) = plot3(uav_start(1), uav_start(2), uav_start(3), 'ob', 'MarkerSize', marker_size,...
    'MarkerFaceColor', 'b', 'DisplayName', 'Goal Region');
h(3) = plot3(path_node(2:num_path_nodes-1,1), path_node(2:num_path_nodes-1,2),...
    path_node(2:num_path_nodes-1,3), '^b', 'MarkerSize', marker_size, 'MarkerFaceColor', 'b',...
    'DisplayName', 'Intermediate Nodes');
h(4) = plot3(path(:,1), path(:,2), path(:,3), 'g-', 'LineWidth', 3, 'DisplayName', 'Flight Path');

% plot obstacles
sphere_obstacles = [];
rect_obstacles = [];
obs_fid = fopen(obstacles_file);
line = fgetl(obs_fid); % ignore format line
while ischar(line)
    line_vals = split(line);
    obs_type = line_vals{1};
    if obs_type == "SPHERE"
        spherePlot(str2double(line_vals{2}), str2double(line_vals{3}), str2double(line_vals{4}), str2double(line_vals{5}))
    elseif obs_type == "RECTANGLE"
        rectanglePlot(str2double(line_vals{2}), str2double(line_vals{3}), str2double(line_vals{4}), str2double(line_vals{5}), str2double(line_vals{6}), str2double(line_vals{7}))
    end
    line = fgetl(obs_fid);
end

legend(h, "Location","east")
xlabel('X-Coordinate (m)')
ylabel('Y-Coordinate (m)')

% set axis to equal
current_axis = axis;
current_xlim = current_axis(1:2);
current_ylim = current_axis(3:4);
current_zlim = zlim;

zlim([0 current_zlim(2)])

x_diff = current_xlim(2) - current_xlim(1);
y_diff = current_ylim(2) - current_ylim(1);
z_diff = current_zlim(2);

max_diff = max([x_diff, y_diff, z_diff]);

shift_xlim = (max_diff - x_diff) / 2;
shift_ylim = (max_diff - y_diff) / 2;
shift_zlim = max_diff - z_diff;

new_xlim = [current_xlim(1) - shift_xlim, current_xlim(2) + shift_xlim];
new_ylim = [current_ylim(1) - shift_ylim, current_ylim(2) + shift_ylim];
new_zlim = [0, current_zlim(2) + shift_zlim];

xlim(new_xlim)
ylim(new_ylim)
zlim(new_zlim)

view(90,0)
grid on
box on
hold off


