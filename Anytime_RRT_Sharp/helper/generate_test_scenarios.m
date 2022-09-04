
clear all
close all
clc

numObstacles = ceil(rand * 50 + 5);

xMin = 0;
xMax = 150;
yMin = 0;
yMax = 150;
rMin = 1;
rMax = 10;

x = zeros(1, numObstacles);
y = zeros(1, numObstacles);
r = zeros(1, numObstacles);

xRange = xMax - xMin;
yRange = yMax - yMin;
rRange = rMax - rMin;

figure()
hold on

for i = 1:numObstacles
   x(i) = xMin + rand * xRange;
   y(i) = yMin + rand * yRange;
   r(i) = rMin + rand * rRange;
   circle(x(i), y(i), r(i))
end

[goalX, goalY] = generatePoint(xMin, xMax, yMin, yMax, x, y, r);
[startX, startY] = generatePoint(xMin, xMax, yMin, yMax, x, y, r);

plot(goalX, goalY, 'g*');
plot(startX, startY, 'b*');

hold off

outputData(startX, startY, goalX, goalY, x, y, r, xMin, xMax, yMin, yMax)

function [x, y] = generatePoint(xMin, xMax, yMin, yMax, obsX, obsY, obsR)
    xRange = xMax - xMin;
    yRange = yMax - yMin;
    
    x = xMin + rand * xRange;
    y = yMin + rand * yRange;

    while (inObstacle(x, y, obsX, obsY, obsR))
        x = xMin + rand * xRange;
        y = yMin + rand * yRange;
    end
end

function inObstacle = inObstacle(x, y, obsX, obsY, obsR)
    inObstacle = false;
    num = size(obsX,2);
    for i = 1:num
        dist = hypot(x - obsX(i), y - obsY(i));
        if (dist <= obsR(i))
            inObstacle = true;
            break;
        end
    end
end

function outputData(startX, startY, goalX, goalY, obsX, obsY, obsR, xMin, xMax, yMin, yMax)
    t = datetime('now','TimeZone','utc','Format','yyyyMMddHHmmss');
    folder = "testData_" + string(t);
    mkdir(folder);
    
    obsInfo = [];
    numObs = size(obsX, 2);
    for i = 1:numObs
        obsInfo = [obsInfo, obsX(i), obsY(i), obsR(i)];
    end

    obsFormat = "%f, %f, %f\n";
    obsFile = fullfile(folder, "obstacles.txt");
    writelines("FORMAT: (x, y, radius)", obsFile);
    fid = fopen(obsFile, "a+");
    fprintf(fid, obsFormat, obsInfo);
    fclose(fid);

    stateFormat = "%f, %f, %f\n";
    stateFile = fullfile(folder, "states.txt");
    writelines("FORMAT: (startX, startY, startTheta) (goalX, goalY, goalTheta)", stateFile);
    fid = fopen(stateFile, "a+");
    fprintf(fid, stateFormat, [startX, startY, 0], [goalX, goalY, 0]);
    fclose(fid);
end


