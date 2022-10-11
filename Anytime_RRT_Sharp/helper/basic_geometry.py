import math
from random import uniform

class Point:
    def __init__(self, x: float, y: float, z: float, pitch: float = 0, yaw: float = 0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.yaw = yaw

    @classmethod
    def generateRandom(cls, xRange: tuple, yRange: tuple, zRange: tuple, pitchRangeDeg: tuple, yawRangeDeg: tuple) -> 'Point':
        x = uniform(xRange[0], xRange[1])
        y = uniform(yRange[0], yRange[1])
        z = uniform(zRange[0], zRange[1])
        pitch = uniform(pitchRangeDeg[0] * (math.pi / 180), pitchRangeDeg[1] * (math.pi / 180))
        yaw = uniform(yawRangeDeg[0] * (math.pi / 180), yawRangeDeg[1] * (math.pi / 180))
        return cls(x, y, z, pitch, yaw)

    @classmethod
    def generateRandomBasic(cls, xRange: tuple, yRange: tuple, zRange: tuple) -> 'Point':
        x = uniform(xRange[0], xRange[1])
        y = uniform(yRange[0], yRange[1])
        z = uniform(zRange[0], zRange[1])
        return cls(x, y, z, 0, 0)

    def __str__(self) -> str:
        return  "({0}, {1}, {2}, {3}, {4})".format(self.x, self.y, self.z, self.pitch, self.yaw)

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
        minPoint = Point.generateRandomBasic(xRange, yRange, zRange)
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
        point = Point.generateRandomBasic(xRange, yRange, zRange)
        radius = uniform(rRange[0], rRange[1])
        return cls(point, radius)

    def __str__(self) -> str:
        return "Center: {0}, Radius: {1}".format(super(Sphere, self).__str__(), self.r)