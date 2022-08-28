#include "ARRTS.hpp"

void ArrtsService::setGoalState(double x, double y, double theta)
{
    _goalState = { x, y, theta };
}

State ArrtsService::goalState()
{
    return _goalState;
}

void ArrtsService::setStartState(double x, double y, double theta)
{
    _startState = { x, y, theta };
}

State ArrtsService::startState()
{
    return _startState;
}

void ArrtsService::addObstacle(double x, double y, double radius)
{
    _obstacles.push_back(Obstacle(x, y, radius));
}

void ArrtsService::addObstacles(const double* x, const double* y, const double* radius, int numObs)
{
    for (int i = 0; i < numObs; ++i)
        _obstacles.push_back(Obstacle(x[i], y[i], radius[i]));
}

void ArrtsService::addObstaclesFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in AddObstaclesFromFile");

    // get num of points for allocation
    int obsCount = 0;
    double xVal, yVal, rVal;
    while (fscanf(file, "%lf,%lf,%lf", &xVal, &yVal, &rVal) != EOF)
        obsCount++;

    // get points
    double* x = new double[obsCount];
    double* y = new double[obsCount];
    double* r = new double[obsCount];
    rewind(file);
    for (int i = 0; i < obsCount; ++i)
    {
        fscanf(file, "%lf,%lf,%lf", &xVal, &yVal, &rVal);
        x[i] = xVal;
        y[i] = yVal;
        r[i] = rVal;
    }

    // close file
    fclose(file);

    // add points
    addObstacles(x, y, r, obsCount);
}

vector<Obstacle> ArrtsService::obstacles()
{
    return _obstacles;
}

Obstacle ArrtsService::obstacles(int i)
{
    if (i >= _obstacles.size() || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetObstacle");

    return _obstacles[i];
}

vector<State> ArrtsService::calculatePath(double standoffRange, double positionBuffer, double freespaceBuffer)
{
    return vector<State>(0);
}