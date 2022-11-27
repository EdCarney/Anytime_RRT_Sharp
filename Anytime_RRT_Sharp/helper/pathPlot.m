function pathPlot(root_node, uav_start, path, path_node, sphere_obstacles, rect_obstacles, view_az, view_el, marker_size, display_legend)

hold on

% plot path
num_path_nodes = size(path_node, 1);
h = zeros(1, 4);
h(1) = plot3(root_node(1), root_node(2), root_node(3), 'square', 'MarkerSize', marker_size,...
    'MarkerFaceColor', 'b', 'DisplayName', 'Initial Position');
h(2) = plot3(uav_start(1), uav_start(2), uav_start(3), 'ob', 'MarkerSize', marker_size,...
    'MarkerFaceColor', 'b', 'DisplayName', 'Goal Region');
h(3) = plot3(path_node(2:num_path_nodes-1,1), path_node(2:num_path_nodes-1,2),...
    path_node(2:num_path_nodes-1,3), '^b', 'MarkerSize', marker_size, 'MarkerFaceColor', 'b',...
    'DisplayName', 'Intermediate Nodes');
h(4) = plot3(path(:,1), path(:,2), path(:,3), 'g-', 'LineWidth', 3, 'DisplayName', 'Flight Path');

% plot obstacles
num_sphere_obs = size(sphere_obstacles, 1);
num_rect_obs = size(rect_obstacles, 1);
for i = 1:num_sphere_obs
    spherePlot(sphere_obstacles(i, 1), sphere_obstacles(i, 2), sphere_obstacles(i, 3), sphere_obstacles(i, 4));
end
for i = 1:num_rect_obs
    rectanglePlot(rect_obstacles(i,1), rect_obstacles(i,2), rect_obstacles(i,3), rect_obstacles(i,4), rect_obstacles(i,5), rect_obstacles(i,6));
end

xlabel('X-Coordinate (m)')
ylabel('Y-Coordinate (m)')
zlabel('Z-Coordinate (m)')
title(sprintf("Azimuth: %d°, Elevation: %d°", view_az, view_el))

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
new_zlim = [0, Inf];%current_zlim(2) + shift_zlim];

xlim(new_xlim)
ylim(new_ylim)
zlim(new_zlim)

% update view angle
view(view_az, view_el);
grid on
box on

if display_legend
    legend(h)
end

hold off

end

