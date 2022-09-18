function rectanglePlot(minX, minY, minZ, maxX, maxY, maxZ)
x = [ minX, maxX, maxX, minX, minX;
      minX, maxX, maxX, minX, minX];
y = [ minY, minY, maxY, maxY, minY;
      minY, minY, maxY, maxY, minY];
z = [ minZ, minZ, minZ, minZ, minZ;
      maxZ, maxZ, maxZ, maxZ, maxZ];

surf(x, y, z)
end

