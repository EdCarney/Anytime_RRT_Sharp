
clear all
close all
clc

numObstacles = ceil(rand * 50 + 5);

xMin = 0;
xMax = 150;
yMin = 0;
yMax = 150;
zMin = 40;
zMax = 120;
rMin = 1;
rMax = 10;
startMinZ = 100;
startMaxZ = 160;
goalMinZ = 0;
goalMaxZ = 5;
minRectWidth = 5;
maxRectWidth = 15;
minRectHeight = 10;
maxRectHeight = 80;

x = zeros(1, numObstacles);
y = zeros(1, numObstacles);
z = zeros(1, numObstacles);
r = zeros(1, numObstacles);

xRect = zeros(2, numObstacles);
yRect = zeros(2, numObstacles);
zRect = zeros(2, numObstacles);

xRange = xMax - xMin;
yRange = yMax - yMin;
zRange = zMax - zMin;
rRange = rMax - rMin;

figure()
hold on

for i = 1:numObstacles
    x(i) = xMin + rand * xRange;
    y(i) = yMin + rand * yRange;
    z(i) = zMin + rand * zRange;
    r(i) = rMin + rand * rRange;
    spherePlot(x(i), y(i), z(i), r(i))

    xVal = xMin + rand * xRange;
    yVal = yMin + rand * yRange;
    zVal = zMin + rand * zRange;
    xRect(:,i) = [ xVal; xVal + max(minRectWidth, rand * maxRectWidth) ];
    yRect(:,i) = [ yVal; yVal + max(minRectWidth, rand * maxRectWidth) ];
    zRect(:,i) = [ 0; rand * maxRectHeight ];
    rectanglePlot(xRect(1,i), yRect(1,i), zRect(1,i), xRect(2,i), yRect(2,i), zRect(2,i))
end

[goalX, goalY, goalZ] = generatePoint(xMin, xMax, yMin, yMax, goalMinZ, goalMaxZ, x, y, z, r, xRect, yRect, zRect);
[startX, startY, startZ] = generatePoint(xMin, xMax, yMin, yMax, startMinZ, startMaxZ, x, y, z, r, xRect, yRect, zRect);

plot3(goalX, goalY, goalZ, 'g*');
plot3(startX, startY, startZ, 'b*');

hold off

outputData(startX, startY, startZ, goalX, goalY, goalZ, x, y, z, r, xRect, yRect, zRect)

function [x, y, z] = generatePoint(xMin, xMax, yMin, yMax, zMin, zMax, obsX, obsY, obsZ, obsR, rectX, rectY, rectZ)
    xRange = xMax - xMin;
    yRange = yMax - yMin;
    zRange = zMax - zMin;
    
    x = xMin + rand * xRange;
    y = yMin + rand * yRange;
    z = zMin + rand * zRange;

    while (inObstacle(x, y, z, obsX, obsY, obsZ, obsR, rectX, rectY, rectZ))
        x = xMin + rand * xRange;
        y = yMin + rand * yRange;
        z = zMin + rand * zRange;
    end
end

function inObstacle = inObstacle(x, y, z, obsX, obsY, obsZ, obsR, rectX, rectY, rectZ)
    inObstacle = false;
    numSphereObs = size(obsX,2);
    numRectObs = size(rectX,2);
    for i = 1:numSphereObs
        xDiff = (x - obsX(i))^2;
        yDiff = (y - obsY(i))^2;
        zDiff = (z - obsZ(i))^2;
        dist = sqrt(xDiff + yDiff + zDiff);
        if (dist <= obsR(i))
            inObstacle = true;
            break;
        end
    end
    for i = 1:numRectObs
        inXRange = x >= rectX(1,i) && x <= rectX(2,i);
        inYRange = y >= rectY(1,i) && y <= rectY(2,i);
        inZRange = z >= rectZ(1,i) && z <= rectZ(2,i);
        if (inXRange && inYRange && inZRange)
            inObstacle = true;
            break;
        end
    end
end

function outputData(startX, startY, startZ, goalX, goalY, goalZ, obsX, obsY, obsZ, obsR, rectX, rectY, rectZ)
    t = datetime('now','TimeZone','utc','Format','yyyyMMddHHmmss');
    folder = "testData_" + string(t);
    mkdir(folder);
    
    obsInfo = [];
    numObs = size(obsX, 2);    
    for i = 1:numObs
        obsInfo = [obsInfo, "SPHERE", obsX(i), obsY(i), obsZ(i), obsR(i)];
    end

    rectInfo = [];
    numRectObs = size(rectX, 2);
    for i = 1:numRectObs
        rectInfo = [rectInfo, "RECTANGLE", rectX(1,i), rectY(1,i), rectZ(1,i), rectX(2,i), rectY(2,i),  rectZ(2,i)];
    end

    obsFormat = "%s %f %f %f %f\n";
    rectFormat = "%s %f %f %f %f %f %f\n";
    obsFile = fullfile(folder, "obstacles.txt");
    writelines("FORMAT: (SPHERE x y z radius) (RECTANGLE minX minY minZ maxX maxY maxZ)", obsFile);
    fid = fopen(obsFile, "a+");
    fprintf(fid, obsFormat, obsInfo);
    fprintf(fid, rectFormat, rectInfo);
    fclose(fid);

    stateFormat = "%f, %f, %f, %f\n";
    stateFile = fullfile(folder, "states.txt");
    writelines("FORMAT: (startX, startY, startZ, startTheta) (goalX, goalY, goalZ, goalTheta)", stateFile);
    fid = fopen(stateFile, "a+");
    fprintf(fid, stateFormat, [startX, startY, startZ, 0], [goalX, goalY, goalZ, 0]);
    fclose(fid);
end


