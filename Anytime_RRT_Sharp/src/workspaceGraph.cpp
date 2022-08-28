#include "WorkspaceGraph.hpp"

Vehicle WorkspaceGraph::vehicle()
{
    return _vehicle;
}

void WorkspaceGraph::setVehicle(Vehicle v)
{
    _vehicle = v;
}

vector<Obstacle> WorkspaceGraph::obstacles()
{
    return _obstacles;
}

GoalState WorkspaceGraph::goalRegion()
{
    return _goalRegion;
}

Obstacle WorkspaceGraph::obstacles(int i)
{
    return _obstacles[i];
}

void WorkspaceGraph::_buildWorkspaceGraph()
{
    printf("Constructing a default empty workspace graph.\n");
    _minPoint = Point(0, 0);
    _maxPoint = Point(0, 0);
    _goalRegionReached = false;
}

bool WorkspaceGraph::readObstaclesFromFile(const char* obstacleFile)
{
    FILE* pFile;

    // open the obstacle file
    pFile = fopen(obstacleFile, "r");

    // check for null pointer
    if (pFile == NULL)
    {
        printf("Unable to open file %s\n", obstacleFile);
        return false;
    }

    // confirm reading in file
    printf("Reading in obstacle data from %s.\n", obstacleFile);

    // determine the number of obstacles in the obstacle file
    int obstacleCount = 0;
    double x, y, radius;
    
    while (fscanf(pFile, "%lf,%lf,%lf", &x, &y, &radius) != EOF)
        ++obstacleCount;

    printf("Number of obstacles is %d.\n", obstacleCount); // DEBUG STATEMENT TO CHECK # OF OBSTACLES

    _obstacles.resize(obstacleCount);
    // assign values
    rewind(pFile);
    for (int i = 0; i < obstacleCount; i++)
    {
        fscanf(pFile, "%lf,%lf,%lf", &x, &y, &radius);
        _obstacles[i] = Obstacle(x, y, radius);
    }

    // close the file
    fclose(pFile);

    // done reading data
    printf("Completed reading data.\n");
    return true;
}

double WorkspaceGraph::computeObsVol()
{
    double volume = 0.0;

    for (Obstacle o : _obstacles)
        volume += 3.14159 * pow(o.radius(), 2);

    return volume;
}

void WorkspaceGraph::addGoalRegion(double x, double y, double theta, double radius)
{
    _goalRegion = GoalState(x, y, radius, theta);
}

void WorkspaceGraph::updateGoalRegion(double x, double y, double theta, double radius)
{
    _goalRegion = GoalState(x, y, radius, theta);
}

void WorkspaceGraph::defineFreespace(double minX, double minY, double maxX, double maxY)
{
    _minPoint = Point(minX, minY);
    _maxPoint = Point(maxX, maxY);
}

vector<ConfigspaceNode> WorkspaceGraph::checkSafety(ConfigspaceNode newNode, vector<ConfigspaceNode> neighbors)
{
    // check if line from node to neighbor intersects any obstacle
    for (auto itr = neighbors.begin(); itr < neighbors.end(); ++itr)
    {
        if (_pathIntersectsObstacle(newNode, *itr))
        {
            neighbors.erase(itr);
            break;
        }
    }

    return neighbors;
}

bool WorkspaceGraph::obstacleInFreespace(double xObs, double yObs, double radiusObs)
{
    if ((xObs - radiusObs < maxX() && xObs + radiusObs > minX()) &&
        (yObs - radiusObs < maxY() && yObs + radiusObs > minY()))
        return true;
    
    return false;
}

void WorkspaceGraph::addObstacle(double x, double y, double radius)
{
    _obstacles.push_back(Obstacle(x, y, radius));
}

bool WorkspaceGraph::atGate(GraphNode node)
{
    double dist = hypot((node.x() - _goalRegion.x()), (node.y() - _goalRegion.y()));
    return dist <= _goalRegion.radius();
}

ConfigspaceNode WorkspaceGraph::extendToNode(GraphNode parentNode, GraphNode newNode, double epsilon)
{
    ConfigspaceNode currentNode;
    double dist = hypot((parentNode.x() - newNode.x()), (parentNode.y() - newNode.y()));

    if (dist >= epsilon)
    {
        double xVal = parentNode.x() + ((newNode.x() - parentNode.x()) / dist) * epsilon;
        double yVal = parentNode.y() + ((newNode.y() - parentNode.y()) / dist) * epsilon;
        currentNode = ConfigspaceNode(xVal, yVal, 0, parentNode.id(), 0, 0);
    }
    else
    {
        currentNode = ConfigspaceNode(newNode.x(), newNode.y(), 0, parentNode.id(), 0, 0);
    }

    if (_nodeIntersectsObstacle(currentNode) || _pathIntersectsObstacle(parentNode, currentNode))
        currentNode.setId(parentNode.id());

    return currentNode;
}

bool WorkspaceGraph::_nodeIntersectsObstacle(GraphNode node)
{
    for (Obstacle o : _obstacles)
        if (o.intersects(node))
            return true;
    return false;
}

bool WorkspaceGraph::_pathIntersectsObstacle(GraphNode parent, GraphNode child)
{
    for (Obstacle o : _obstacles)
        if (o.intersects(Line(parent, child)))
            return true;
    return false;
}

ConfigspaceNode WorkspaceGraph::connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{
    newNode.setParentId(parentNode.id());
    return newNode;
}

bool WorkspaceGraph::checkAtGoal(ConfigspaceNode node)
{
    // update vehicle state to temp node
    State s(node.x(), node.y(), node.theta);
    _vehicle.updateState(s);

    double distToGoal = hypot((_vehicle.state().x() - _goalRegion.x()), (_vehicle.state().y() - _goalRegion.y()));
    return distToGoal < (_goalRegion.radius() + _vehicle.boundingRadius());
}
