import matplotlib.pyplot as plt
import math
import os
import numpy as np
from datetime import datetime
from random import uniform, randrange

class Point:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def generateRandom(cls, xRange: tuple, yRange: tuple, zRange: tuple) -> 'Point':
        x = uniform(xRange[0], xRange[1])
        y = uniform(yRange[0], yRange[1])
        z = uniform(zRange[0], zRange[1])
        return cls(x, y, z)

    def __str__(self) -> str:
        return  "({0}, {1}, {2})".format(self.x, self.y, self.z)

class Rectangle:
    def __init__(self, minPoint: Point, maxPoint: Point) -> None:
        self.minPoint = minPoint
        self.maxPoint = maxPoint

    def intersects(self, point: Point) -> bool:
        inXRange = point.x >= self.minPoint.x and point.x <= self.maxPoint.x
        inYRange = point.y >= self.minPoint.y and point.y <= self.maxPoint.y
        inZRange = point.z >= self.minPoint.z and point.z <= self.maxPoint.z
        return inXRange and inYRange and inZRange

    @classmethod
    def generateRandom(cls, xRange: tuple, yRange: tuple, zRange: tuple, widthRange: tuple, heightRange: tuple) -> 'Rectangle':
        xWidth = uniform(widthRange[0], widthRange[1])
        yWidth = uniform(widthRange[0], widthRange[1])
        height = uniform(heightRange[0], heightRange[1])
        minPoint = Point.generateRandom(xRange, yRange, zRange)
        maxPoint = Point(minPoint.x + xWidth, minPoint.y + yWidth, minPoint.z + height)
        return cls(minPoint, maxPoint)

    def __str__(self) -> str:
        return "MinPoint: {0}, MaxPoint: {1}".format(self.minPoint, self.maxPoint)

class Sphere(Point):
    def __init__(self, x: float, y: float, z: float, r: float) -> None:
        super().__init__(x, y, z)
        self.r = r
    
    def __init__(self, point: Point, r: float) -> None:
        super().__init__(point.x, point.y, point.z)
        self.r = r

    def intersects(self, point: Point) -> bool:
        xDiff = math.pow(self.x - point.x, 2)
        yDiff = math.pow(self.y - point.y, 2)
        zDiff = math.pow(self.z - point.z, 2)
        dist = math.sqrt(xDiff + yDiff + zDiff)
        return dist <= self.r

    @classmethod
    def generateRandom(cls, xRange: tuple, yRange: tuple, zRange: tuple, rRange: tuple) -> 'Sphere':
        point = Point.generateRandom(xRange, yRange, zRange)
        radius = uniform(rRange[0], rRange[1])
        return cls(point, radius)

    def __str__(self) -> str:
        return "Center: {0}, Radius: {1}".format(super(Sphere, self).__str__(), self.r)

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
    stateFile.write("{0} {1} {2} 0.0 2.5\n".format(goal.x, goal.y, goal.z))
    stateFile.close()

def plotSphere(ax, sphere: Sphere) -> None:
    u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
    x = sphere.x + np.cos(u) * np.sin(v) * sphere.r
    y = sphere.y + np.sin(u) * np.sin(v) * sphere.r
    z = sphere.z + np.cos(v) * sphere.r
    ax.plot_surface(x, y, z)

def plotRectangle(ax, rectangle: Rectangle) -> None:
    minX = rectangle.minPoint.x
    minY = rectangle.minPoint.y
    minZ = rectangle.minPoint.z
    maxX = rectangle.maxPoint.x
    maxY = rectangle.maxPoint.y
    maxZ = rectangle.maxPoint.z
    x = np.array([[minX, maxX, maxX, minX, minX],[minX, maxX, maxX, minX, minX]])
    y = np.array([[minY, minY, maxY, maxY, minY],[minY, minY, maxY, maxY, minY]])
    z = np.array([[minZ, minZ, minZ, minZ, minZ],[maxZ, maxZ, maxZ, maxZ, maxZ]])
    ax.plot_surface(x, y, z)

def generatePlot(start: Point, goal: Point, spheres: list[Sphere], rectangles: list[Rectangle]) -> None:
    fig = plt.figure()
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax = fig.add_subplot(projection='3d')
    for s in spheres:
        plotSphere(ax, s)
    for r in rectangles:
        plotRectangle(ax, r)
    ax.plot(start.x, start.y, start.z, "b*")
    ax.plot(goal.x, goal.y, goal.z, "g*")
    plt.show()

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
generatePlot(startPoint, goalPoint, sphereObs, rectanObs)