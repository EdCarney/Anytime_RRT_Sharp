function spherePlot(x,y,z,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)

% unit sphere points
[X,Y,Z] = sphere;

% scale
X = X*r;
Y = Y*r;
Z = Z*r;

surf(X + x, Y + y, Z + z)
end

