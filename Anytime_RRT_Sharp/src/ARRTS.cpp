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

void ArrtsService::addObstacles(const vector<double>& x, const vector<double>& y, const vector<double>& r)
{
    int size = x.size();
    for (int i = 0; i < size; ++i)
        _obstacles.push_back(Obstacle(x[i], y[i], r[i]));
}

void ArrtsService::readObstaclesFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in AddObstaclesFromFile");

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");

    double xVal, yVal, rVal;
    vector<double> x, y, r;
    while (fscanf(file, "%lf,%lf,%lf", &xVal, &yVal, &rVal) != EOF)
    {
        x.push_back(xVal);
        y.push_back(yVal);
        r.push_back(rVal);
    }
    fclose(file);
    addObstacles(x, y, r);
}

void ArrtsService::readStatesFromFile(FILE* file)
{
    return;
}

void ArrtsService::readLimitsFromFile(FILE* file)
{
    return;
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