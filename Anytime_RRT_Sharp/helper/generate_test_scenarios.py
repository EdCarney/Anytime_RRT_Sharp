import os
from datetime import datetime
from random import randrange
from basic_geometry import Point, Rectangle, Sphere
from plotting_tools import generatePlot

GOAL_RADIUS = 2.5

def generatePointAvoidingObstacles(xRange: tuple, yRange: tuple, zRange: tuple, sphereObs: list[Sphere], rectanObs: list[Rectangle]) -> Point:
    point = Point.generateRandom(xRange, yRange, zRange)
    while (any(x.intersects(point) for x in sphereObs) or any(x.intersects(point) for x in rectanObs)):
        point = Point.generateRandom(xRange, yRange, zRange)
    return point

def writeOutputData(start: Point, goal: Point, spheres: list[Sphere], rectangles: list[Rectangle]) -> None:
    t = datetime.utcnow().strftime("%Y%m%d%H%M%S")
    folder = "testData_" + t
    os.mkdir(folder)

    obsFile = os.path.join(folder, "obstacles.txt")
    stateFile = os.path.join(folder, "states.txt")

    obsFile = open(obsFile, "w+")
    stateFile = open(stateFile, "w+")

    obsFile.write("FORMAT: (SPHERE x y z radius) (RECTANGLE minX minY minZ maxX maxY maxZ)\n")
    for s in spheres:
        obsFile.write("SPHERE {0} {1} {2} {3}\n".format(s.x, s.y, s.z, s.r))
    for r in rectangles:
        obsFile.write("RECTANGLE {0} {1} {2} {3} {4} {5}\n".format(r.minPoint.x, r.minPoint.y, r.minPoint.z, r.maxPoint.x, r.maxPoint.y, r.maxPoint.z))
    obsFile.close()

    stateFile.write("FORMAT: (startX startY startZ startTheta) (goalX goalY goalZ goalTheta)\n")
    stateFile.write("{0} {1} {2} 0.0\n".format(start.x, start.y, start.z))
    stateFile.write("{0} {1} {2} 0.0 {3}\n".format(goal.x, goal.y, goal.z, GOAL_RADIUS))
    stateFile.close()

### define ranges for data generation ###
numSphereObsRange = (5, 20)
numRectanObsRange = (5, 30)
xRange = (0, 150)
yRange = (0, 150)
sphereObsZRange = (40, 120)
sphereObsRadiusRange = (1, 10)
rectanObsZRange = (0, 0)
rectanObsHeightRange = (10, 80)
rectanObsWidthRange = (5, 15)
startZRange = (100, 160)
goalZRange = (0, 5)
#########################################

numSphereObs = randrange(numSphereObsRange[0], numSphereObsRange[1])
numRectanObs = randrange(numRectanObsRange[0], numRectanObsRange[1])

sphereObs = [Sphere.generateRandom(xRange, yRange, sphereObsZRange, sphereObsRadiusRange) for _ in range(numSphereObs)]
rectanObs = [Rectangle.generateRandom(xRange, yRange, rectanObsZRange, rectanObsWidthRange, rectanObsHeightRange) for _ in range(numRectanObs)]

startPoint = generatePointAvoidingObstacles(xRange, yRange, startZRange, sphereObs, rectanObs)
goalPoint = generatePointAvoidingObstacles(xRange, yRange, goalZRange, sphereObs, rectanObs)

writeOutputData(startPoint, goalPoint, sphereObs, rectanObs)
generatePlot(startPoint, goalPoint, GOAL_RADIUS, sphereObs, rectanObs)