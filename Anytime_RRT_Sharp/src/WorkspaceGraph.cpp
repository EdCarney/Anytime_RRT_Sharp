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

void WorkspaceGraph::setGoalRegion(double x, double y, double theta, double radius)
{
    _goalRegion = GoalState(x, y, radius, theta);
}

void WorkspaceGraph::setGoalRegion(State goalState, double radius)
{
    _goalRegion = GoalState(goalState.x(), goalState.y(), radius, goalState.theta());
}

void WorkspaceGraph::defineFreespace(double minX, double minY, double maxX, double maxY)
{
    _minPoint = Point(minX, minY);
    _maxPoint = Point(maxX, maxY);
}

void WorkspaceGraph::defineFreespace(Rectangle limits)
{
    _minPoint = limits.minPoint();
    _maxPoint = limits.maxPoint();
}

bool WorkspaceGraph::_obstacleInFreespace(double x, double y, double radius) const
{
    if ((x - radius < maxX() && x + radius > minX()) &&
        (y - radius < maxY() && y + radius > minY()))
        return true;
    
    return false;
}

bool WorkspaceGraph::_obstacleInFreespace(Obstacle o) const
{
    return _obstacleInFreespace(o.x(), o.y(), o.radius());
}

void WorkspaceGraph::addObstacle(double x, double y, double radius)
{
    _obstacles.push_back(Obstacle(x, y, radius));
}

void WorkspaceGraph::addObstacles(vector<Obstacle> obstacles)
{
    for (Obstacle o : obstacles)
        if (_obstacleInFreespace(o))
            _obstacles.push_back(o);
}

bool WorkspaceGraph::atGate(GraphNode node)
{
    double dist = node.distanceTo(_goalRegion);
    return dist <= _goalRegion.radius();
}

bool WorkspaceGraph::nodeIsSafe(Point p)
{
    for (Obstacle o : _obstacles)
        if (o.intersects(p))
            return false;
    return true;
}

bool WorkspaceGraph::pathIsSafe(Point p1, Point p2)
{
    for (Obstacle o : _obstacles)
        if (o.intersects(Line(p1, p2)))
            return false;
    return true;
}

bool WorkspaceGraph::checkAtGoal(GraphNode node)
{
    // update vehicle state to temp node
    State s(node.x(), node.y(), node.theta());
    _vehicle.updateState(s);

    double distToGoal = _vehicle.state().distanceTo(_goalRegion);
    return distToGoal < (_goalRegion.radius() + _vehicle.boundingRadius());
}
