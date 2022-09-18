
clear all
close all
clc

numObstacles = ceil(rand * 50 + 5);

xMin = 0;
xMax = 150;
yMin = 0;
yMax = 150;
zMin = 0;
zMax = 150;
rMin = 1;
rMax = 10;

x = zeros(1, numObstacles);
y = zeros(1, numObstacles);
z = zeros(1, numObstacles);
r = zeros(1, numObstacles);

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
end

[goalX, goalY, goalZ] = generatePoint(xMin, xMax, yMin, yMax, zMin, zMax, x, y, z, r);
[startX, startY, startZ] = generatePoint(xMin, xMax, yMin, yMax, zMin, zMax, x, y, z, r);

plot3(goalX, goalY, goalZ, 'g*');
plot3(startX, startY, startZ, 'b*');

hold off

outputData(startX, startY, startZ, goalX, goalY, goalZ, x, y, z, r)

function [x, y, z] = generatePoint(xMin, xMax, yMin, yMax, zMin, zMax, obsX, obsY, obsZ, obsR)
    xRange = xMax - xMin;
    yRange = yMax - yMin;
    zRange = zMax - zMin;
    
    x = xMin + rand * xRange;
    y = yMin + rand * yRange;
    z = zMin + rand * zRange;

    while (inObstacle(x, y, z, obsX, obsY, obsZ, obsR))
        x = xMin + rand * xRange;
        y = yMin + rand * yRange;
        z = zMin + rand * zRange;
    end
end

function inObstacle = inObstacle(x, y, z, obsX, obsY, obsZ, obsR)
    inObstacle = false;
    num = size(obsX,2);
    for i = 1:num
        xDiff = (x - obsX(i))^2;
        yDiff = (y - obsY(i))^2;
        zDiff = (z - obsZ(i))^2;
        dist = sqrt(xDiff + yDiff + zDiff);
        if (dist <= obsR(i))
            inObstacle = true;
            break;
        end
    end
end

function outputData(startX, startY, startZ, goalX, goalY, goalZ, obsX, obsY, obsZ, obsR)
    t = datetime('now','TimeZone','utc','Format','yyyyMMddHHmmss');
    folder = "testData_" + string(t);
    mkdir(folder);
    
    obsInfo = [];
    numObs = size(obsX, 2);
    for i = 1:numObs
        obsInfo = [obsInfo, "SPHERE", obsX(i), obsY(i), obsZ(i), obsR(i)];
    end

    obsFormat = "%s %f %f %f %f\n";
    obsFile = fullfile(folder, "obstacles.txt");
    writelines("FORMAT: (SPHERE x y z radius) (RECTANGLE minX minY minZ maxX maxY maxZ)", obsFile);
    fid = fopen(obsFile, "a+");
    fprintf(fid, obsFormat, obsInfo);
    fclose(fid);

    stateFormat = "%f, %f, %f, %f\n";
    stateFile = fullfile(folder, "states.txt");
    writelines("FORMAT: (startX, startY, startZ, startTheta) (goalX, goalY, goalZ, goalTheta)", stateFile);
    fid = fopen(stateFile, "a+");
    fprintf(fid, stateFormat, [startX, startY, startZ, 0], [goalX, goalY, goalZ, 0]);
    fclose(fid);
end


