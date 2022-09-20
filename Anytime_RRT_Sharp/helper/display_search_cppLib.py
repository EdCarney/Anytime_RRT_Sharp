import os, sys
import matplotlib.pyplot as plt
from basic_geometry import *
from plotting_tools import *

NODE_FILE_NAME = "nodes.txt"
EDGE_FILE_NAME = "edges.txt"
PATH_FILE_NAME = "output_path.txt"
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

if (len(sys.argv) < 2):
    print("ERROR: at least one test data folder must be specified. Exiting...")
    exit()

folder = sys.argv[1]

fig = plt.figure()
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax = fig.add_subplot(projection='3d')

plotPath(ax, folder)
plotObstacles(ax, folder)
plotStates(ax, folder)
plt.show()