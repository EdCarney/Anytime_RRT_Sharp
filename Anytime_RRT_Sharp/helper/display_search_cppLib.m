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

folder = "testData_20220918194311"; %% change this

node_file = fullfile(folder, 'nodes.txt');
edge_file = fullfile(folder, 'edges.txt');
path_file = fullfile(folder, 'output_path.txt');
search_tree_file = fullfile(folder, 'search_tree.txt');
obstacles_file = fullfile(folder, 'obstacles.txt');
state_file = fullfile(folder, 'states.txt');

nodes = readmatrix(node_file);
edges_raw = readmatrix(edge_file);
path = readmatrix(path_file);
search_tree_raw = readmatrix(search_tree_file);
state_raw = readmatrix(state_file, "Range",1:1);

% specify goal region [x, y, z, radius] and start point [x, y, z]
root_node = state_raw(2,1:3);
uav_start = [state_raw(3,1:3), state_raw(3,5)];

% a bit of data processing for faster plotting
search_tree = nan(3*size(search_tree_raw, 1), 3);

search_tree(1:3:end-2, 1) = search_tree_raw(:, 2);
search_tree(1:3:end-2, 2) = search_tree_raw(:, 3);
search_tree(1:3:end-2, 3) = search_tree_raw(:, 4);

search_tree(2:3:end-1, 1) = search_tree_raw(:, 6);
search_tree(2:3:end-1, 2) = search_tree_raw(:, 7);
search_tree(2:3:end-1, 3) = search_tree_raw(:, 8);

edges = nan(3*size(edges_raw, 1), 3);

for i = 1:length(edges_raw(:,1))
    edges_raw(i, 1) = find(nodes(:,5) == edges_raw(i, 1));
    edges_raw(i, 2) = find(nodes(:,5) == edges_raw(i, 2));
end

edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
edges(1:3:end-2, 3) = nodes(edges_raw(:, 1),3);
edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);
edges(2:3:end-1, 3) = nodes(edges_raw(:, 2),3);

figure('Renderer', 'painters', 'Position', [50 50 600 600])

% plot the goal region
[xs,ys,zs] = sphere();
xs = xs*uav_start(4);
ys = ys*uav_start(4);
zs = zs*uav_start(4);
surf(xs + uav_start(1),ys + uav_start(2),zs + uav_start(3),'EdgeColor','none')

hold on

%plot3(nodes(:,1), nodes(:,2), nodes(:,3), '.k')
%plot3(edges(:,1), edges(:,2), edges(:,3), 'k')
%plot3(search_tree(:, 1), search_tree(:, 2), search_tree(:, 3), 'm', 'LineWidth', 0.5);
plot3(root_node(1), root_node(2), root_node(3), 'or', 'MarkerSize', 7,...
    'MarkerFaceColor', 'r')
plot3(path(:,1), path(:,2), path(:,3), 'g:*', 'LineWidth', 4);


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

title('Path Search Tree')
xlabel('X-Coordinate (m)')
ylabel('Y-Coordinate (m)')
hold off


