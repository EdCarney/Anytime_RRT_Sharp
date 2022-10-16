import os, sys
from typing import Tuple
from math import inf
import matplotlib.pyplot as plt
from basic_geometry import *
from plotting_tools import *

NODE_FILE_NAME = "nodes.txt"
EDGE_FILE_NAME = "edges.txt"
PATH_FILE_NAME = "output_path.txt"
FULL_PATH_FILE_NAME = "full_output_path.txt"
TREE_FILE_NAME = "search_tree.txt"
STATE_FILE_NAME = "states.txt"
OBSTACLE_FILE_NAME = "obstacles.txt"

def __getSplitFileLines(filePath: str) -> list[str]:
    with open(filePath, "r") as file:
        lines = [line.split() for line in file.readlines()]
    return lines

def plotPath(ax, folder: str) -> None:
    pathFile = os.path.join(folder, PATH_FILE_NAME)
    lines = __getSplitFileLines(pathFile)
    path = np.array([[],[],[]])
    for line in lines:
        path = np.array([
            np.append(path[0], float(line[0])),
            np.append(path[1], float(line[1])),
            np.append(path[2], float(line[2]))
            ])
    ax.plot3D(path[0], path[1], path[2], "g-")

def plotFullPath(ax, folder: str) -> None:
    pathFile = os.path.join(folder, FULL_PATH_FILE_NAME)
    lines = __getSplitFileLines(pathFile)
    path = np.array([[],[],[]])
    for line in lines:
        path = np.array([
            np.append(path[0], float(line[0])),
            np.append(path[1], float(line[1])),
            np.append(path[2], float(line[2]))
            ])
    ax.plot3D(path[0], path[1], path[2], "r-")

def plotObstacles(ax, folder: str) -> None:
    obstacleFile = os.path.join(folder, OBSTACLE_FILE_NAME)
    lines = __getSplitFileLines(obstacleFile)
    for line in lines:
        if (line[0] == "SPHERE"):
            center = Point(float(line[1]), float(line[2]), float(line[3]))
            radius = float(line[4])
            plotSphere(ax, Sphere(center, radius))
        elif (line[0] == "RECTANGLE"):
            minPoint = Point(float(line[1]), float(line[2]), float(line[3]))
            maxPoint = Point(float(line[4]), float(line[5]), float(line[6]))
            plotRectangle(ax, Rectangle(minPoint, maxPoint))

def plotStates(ax, folder: str) -> None:
    stateFile = os.path.join(folder, STATE_FILE_NAME)
    lines = __getSplitFileLines(stateFile)
    startLine = lines[1]
    goalLine = lines[2]
    start = Point(float(startLine[0]), float(startLine[1]), float(startLine[2]))
    goal = Point(float(goalLine[0]), float(goalLine[1]), float(goalLine[2]))
    goalRadius = float(goalLine[4])
    plotStartGoalStates(ax, start, goal, goalRadius)

def getAxes(folder: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    minLims, maxLims = getPathLimits(folder)#getLimits(folder)
    axisSizes = list(map(lambda x: x[0] - x[1], zip(maxLims, minLims)))
    maxAxisSize = max(axisSizes)

    minAxisLims = list(map(lambda val: val[0] - (maxAxisSize - val[1]) / 2, zip(minLims, axisSizes)))
    maxAxisLims = list(map(lambda val: val[0] + (maxAxisSize - val[1]) / 2, zip(maxLims, axisSizes)))

    # set z axis to have zero as bottom
    maxAxisLims[2] = maxAxisSize
    minAxisLims[2] = 0

    return (minAxisLims, maxAxisLims)

def getLimits(folder: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    minStateLims, maxStateLims = getStateLimits(folder)
    minPathLims, maxPathLims = getPathLimits(folder)
    minObstacleLims, maxObstacleLims = getObstacleLimits(folder)
    minLims = list(map(min, zip(minStateLims, minPathLims, minObstacleLims)))
    maxLims = list(map(max, zip(maxStateLims, maxPathLims, maxObstacleLims)))
    return (minLims, maxLims)

def getStateLimits(folder: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    stateFile = os.path.join(folder, STATE_FILE_NAME)
    lines = __getSplitFileLines(stateFile)
    maxVals = (-inf, -inf, -inf)
    minVals = (inf, inf, inf)
    for line in lines:
        if "FORMAT:" in line: continue
        valItrs = list(zip(map(float, line), range(3)))
        maxVals = list(map(lambda valItr: max(valItr[0], maxVals[valItr[1]]), valItrs))
        minVals = list(map(lambda valItr: min(valItr[0], minVals[valItr[1]]), valItrs))
    return (minVals, maxVals)

def getPathLimits(folder: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    pathFile = os.path.join(folder, PATH_FILE_NAME)
    lines = __getSplitFileLines(pathFile)
    maxVals = (-inf, -inf, -inf)
    minVals = (inf, inf, inf)
    for line in lines:
        if "FORMAT:" in line: continue
        valItrs = list(zip(map(float, line), range(3)))
        maxVals = list(map(lambda valItr: max(valItr[0], maxVals[valItr[1]]), valItrs))
        minVals = list(map(lambda valItr: min(valItr[0], minVals[valItr[1]]), valItrs))
    return (minVals, maxVals)

def getObstacleLimits(folder: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    obstacleFile = os.path.join(folder, OBSTACLE_FILE_NAME)
    lines = __getSplitFileLines(obstacleFile)
    maxVals = (-inf, -inf, -inf)
    minVals = (inf, inf, inf)
    for line in lines:
        if "FORMAT:" in line: continue
        if (line[0] == "SPHERE"):
            radius = float(line[4])
            valItrs = list(zip(map(float, line[1:4]), range(3)))
            maxVals = list(map(lambda valItr: max(valItr[0] + radius, maxVals[valItr[1]]), valItrs))
            minVals = list(map(lambda valItr: min(valItr[0] - radius, minVals[valItr[1]]), valItrs))
        elif (line[0] == "RECTANGLE"):
            minValIts = list(zip(map(float, line[1:4]), range(3)))
            maxValIts = list(zip(map(float, line[4:7]), range(3)))
            maxVals = list(map(lambda valItr: max(valItr[0], maxVals[valItr[1]]), maxValIts))
            minVals = list(map(lambda valItr: min(valItr[0], minVals[valItr[1]]), minValIts))
    return (minVals, maxVals)

if (len(sys.argv) < 2):
    print("ERROR: at least one test data folder must be specified. Exiting...")
    exit()

folder = sys.argv[1]

fig = plt.figure()
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax = fig.add_subplot(projection='3d')

axesBottom, axesTop = getAxes(folder)

ax.set_xlim(axesBottom[0], axesTop[0])
ax.set_ylim(axesBottom[1], axesTop[1])
ax.set_zlim(axesBottom[2], axesTop[2])

plotPath(ax, folder)
plotFullPath(ax, folder)
plotObstacles(ax, folder)
plotStates(ax, folder)
plt.show()