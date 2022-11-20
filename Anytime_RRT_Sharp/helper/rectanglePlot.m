function rectanglePlot(minX, minY, minZ, maxX, maxY, maxZ)
x = [ minX, maxX, maxX, minX, minX;
      minX, maxX, maxX, minX, minX];
y = [ minY, minY, maxY, maxY, minY;
      minY, minY, maxY, maxY, minY];
z = [ minZ, minZ, minZ, minZ, minZ;
      maxZ, maxZ, maxZ, maxZ, maxZ];

s1 = surf(x, y, z);
set(s1, 'FaceColor', '#FF8B72');

[x_top, y_top] = meshgrid(minX:0.5:maxX, minY:0.5:maxY);
z_top = ones(size(x_top)) * maxZ;
s2 = surf(x_top, y_top, z_top);
set(s2, 'FaceColor', '#FF8B72');

end

