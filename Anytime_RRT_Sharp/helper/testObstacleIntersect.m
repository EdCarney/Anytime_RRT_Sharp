clear all
close all

radius_c = 3;
center_c = [1,1,1];

p1 = [-10,0,-10];
p2 = [15,0,15];

circ_vec = center_c;
line_vec = p2 - p1;

figure
hold on
spherePlot(center_c(1), center_c(2), center_c(3), radius_c);
plot3(0:0.5:10, zeros(1,21), 0:0.5:10);

num_norm_points = 10;
max_norm_point = n * num_norm_points;
% plot3( ...
%     center_c(1) + [0:n(1):max_norm_point(1)],...
%     center_c(2) + [0:n(2):max_norm_point(2)],...
%     center_c(3) + [0:n(3):max_norm_point(3)])

xlim = 10;
ylim = 10;
zlim = 10;

quiver3(0,0,-max(zlim),0,0,2*max(zlim),'b','LineWidth',1)
quiver3(0,-max(ylim),0,0,2*max(ylim),0,'b','LineWidth',1)
quiver3(-max(xlim),0,0,2*max(xlim),0,0,'b','LineWidth',1)
text(0,0,max(zlim),'Z','Color','b')
text(0,max(ylim),0,'Y','Color','b')
text(max(xlim),0,0,'X','Color','b')
axis equal
view(30,30)
set(gca, 'LineWidth',2, 'XGrid','on', 'GridLineStyle','--')


dx = p2(1) - p1(1);
dy = p2(2) - p1(2);
dz = p2(3) - p1(3);
a = dx*dx + dy*dy + dz*dz;
b = 2 * (dx*(p1(1) - center_c(1)) + dy*(p1(2) - center_c(2)) + dz*(p1(3) - center_c(3)));
c = center_c(1)*center_c(1) + center_c(2)*center_c(2) + center_c(3)*center_c(3);
c = c + (p1(1)*p1(1) + p1(2)*p1(2) + p1(3)*p1(3));
c = c - (2 * (center_c(1) * p1(1) + center_c(2) * p1(2) + center_c(3) * p1(3)));
c = c - (radius_c * radius_c);
bb4ac = b * b - 4 * a * c


mu1 = (-b + sqrt(bb4ac)) / (2 * a);
mu2 = (-b - sqrt(bb4ac)) / (2 * a);

pm1 = p1 + mu1 * (p2 - p1)
pm2 = p1 + mu2 * (p2 - p1)

AB = p2 - p1;
AC1 = pm1 - p1;
AC2 = pm2 - p1;

dotAB = dot(AB,AB);
dotAC1 = dot(AB,AC1);
dotAC2 = dot(AB,AC2);

if (dotAC1 < 0 && dotAC2 < 0)
    disp("No intersection")
else
    if (dotAC1 == 0 || dotAC1 == dotAB)
        disp("pm1 is on end of line segment")
    elseif (dotAC1 > 0 && dotAC1 < dotAB)
        disp("pm1 is somewhere on line segment")
    end
    if (dotAC2 == 0 || dotAC2 == dotAB)
        disp("pm2 is on end of line segment")
    elseif (dotAC2 > 0 && dotAC2 < dotAB)
        disp("pm2 is somewhere on line segment")
    end
end