function limitsPlot(pathLimits, minMaxPitch, legendDisplayName, titleName, shiftValues)

hold on

path_size = size(pathLimits, 1);
path_percent = zeros(path_size, 1);
path_percent_step = 1 / path_size;
for i = 1:path_size
    path_percent(i) = i * path_percent_step;
    if shiftValues
        if pathLimits(i) > pi
            pathLimits(i) = pathLimits(i) - 2 * pi;
        elseif pathLimits(i) < -pi
            pathLimits(i) = pathLimits(i) + 2 * pi;
        end
    end
end

h = [plot(path_percent, rad2deg(pathLimits), 'k-', 'DisplayName', legendDisplayName)];
if length(minMaxPitch) == 2
    pitch_lim_min = ones(path_size, 1) * minMaxPitch(1);
    pitch_lim_max = ones(path_size, 1) * minMaxPitch(2);
    h = [h, plot(path_percent, pitch_lim_min, 'r--', 'DisplayName', 'Limits')];
    plot(path_percent, pitch_lim_max, 'r--');
end

xlabel('Proportion of Path Completed')
ylabel('Angle (°)')

if length(minMaxPitch) == 2
    ylim([minMaxPitch(1) * 2, minMaxPitch(2) * 2])
else
    ylim([-Inf, Inf])
end

legend(h)
title(titleName)

hold off

end

