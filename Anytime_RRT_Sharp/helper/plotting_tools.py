import matplotlib.pyplot as plt
import numpy as np
from basic_geometry import Sphere, Rectangle, Point

def plotSphere(ax, sphere: Sphere, color: str = "r") -> None:
    u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
    x = sphere.x + np.cos(u) * np.sin(v) * sphere.r
    y = sphere.y + np.sin(u) * np.sin(v) * sphere.r
    z = sphere.z + np.cos(v) * sphere.r
    ax.plot_surface(x, y, z, color=color)

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

def plotStartGoalStates(ax, start: Point, goal: Point, goalRadius: float) -> None:
    plotSphere(ax, Sphere(goal, goalRadius), "g")
    ax.plot(start.x, start.y, start.z, "b*")

def generatePlot(start: Point, goal: Point, goalRadius: float, spheres: list[Sphere], rectangles: list[Rectangle]) -> None:
    fig = plt.figure()
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax = fig.add_subplot(projection='3d')
    for s in spheres:
        plotSphere(ax, s)
    for r in rectangles:
        plotRectangle(ax, r)
    plotStartGoalStates(ax, start, goal, goalRadius)
    plt.show()