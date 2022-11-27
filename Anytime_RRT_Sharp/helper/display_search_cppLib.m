clear all
close all
clc

folder = "testData_paper_2_Dubins3d_2"; %% change this

path_file = fullfile(folder, 'full_output_path.txt');
path_node_file = fullfile(folder, 'output_path.txt');
obstacles_file = fullfile(folder, 'obstacles.txt');
state_file = fullfile(folder, 'states.txt');

path = readmatrix(path_file);
path_node = readmatrix(path_node_file);
state_raw = readcell(state_file);

% specify goal region [x, y, z, radius] and start point [x, y, z]
root_node_array = double(string(state_raw(2)).split(' '));
uav_start_array = double(string(state_raw(3)).split(' '));
root_node = root_node_array(1:3);
uav_start = [uav_start_array(1:3)', uav_start_array(6)];

% get obstacles
sphere_obstacles = [];
rectangle_obstacles = [];
obs_fid = fopen(obstacles_file);
line = fgetl(obs_fid); % ignore format line
while ischar(line)
    line_vals = split(line);
    obs_type = line_vals{1};
    if obs_type == "SPHERE"
        sphere = [str2double(line_vals{2}), str2double(line_vals{3}), str2double(line_vals{4}), str2double(line_vals{5})];
        sphere_obstacles = [sphere; sphere_obstacles];
    elseif obs_type == "RECTANGLE"
        rect = [str2double(line_vals{2}), str2double(line_vals{3}), str2double(line_vals{4}), str2double(line_vals{5}), str2double(line_vals{6}), str2double(line_vals{7})];
        rectangle_obstacles = [rect; rectangle_obstacles];
    end
    line = fgetl(obs_fid);
end

figure(1)
subplot(2,2,1)
pathPlot(root_node, uav_start, path, path_node, sphere_obstacles, rectangle_obstacles, 90, 0, 10, false);
subplot(2,2,2)
pathPlot(root_node, uav_start, path, path_node, sphere_obstacles, rectangle_obstacles, 0, 90, 10, false);
subplot(2,2,[3,4])
pathPlot(root_node, uav_start, path, path_node, sphere_obstacles, rectangle_obstacles, 45, 20, 10, true);

figure(2)
subplot(2,1,1)
limitsPlot(flip(path(:,4)) - pi, [], '\psi (°)', 'Heading Angle \psi during Flight Path', true);
subplot(2,1,2)
limitsPlot(-flip(path(:,5)), [-30, 30], '\gamma (°)', 'Pitch Angle \gamma during Flight Path', true);

