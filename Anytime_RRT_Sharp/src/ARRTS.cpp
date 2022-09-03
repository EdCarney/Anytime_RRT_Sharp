#include "ARRTS.hpp"

State ArrtsService::goalState() const
{
    return _goalState;
}

void ArrtsService::setGoalState(double x, double y, double theta)
{
    _goalState = { x, y, theta };
}

State ArrtsService::startState() const
{
    return _startState;
}

Rectangle ArrtsService::limits() const
{
    return _limits;
}

void ArrtsService::setLimits(Point minPoint, Point maxPoint)
{
    _limits = Rectangle(minPoint, maxPoint);
}

void ArrtsService::setLimits(double minX, double minY, double maxX, double maxY)
{
    _limits = Rectangle(minX, minY, maxX, maxY);
}

void ArrtsService::setStartState(double x, double y, double theta)
{
    _startState = { x, y, theta };
}

vector<Obstacle> ArrtsService::obstacles() const
{
    return _obstacles;
}

Obstacle ArrtsService::obstacles(int i) const
{
    if (i >= _obstacles.size() || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetObstacle");

    return _obstacles[i];
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
        throw runtime_error("NULL file pointer in readObstaclesFromFile()");

    double xVal, yVal, rVal;
    vector<double> x, y, r;

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");
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
    if (file == NULL)
        throw runtime_error("NULL file pointer in readStatesFromFile()");

    double startX, startY, startTheta;
    double goalX, goalY, goalTheta;

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");
    fscanf(file, "%lf,%lf,%lf", &startX, &startY, &startTheta);
    fscanf(file, "%lf,%lf,%lf", &goalX, &goalY, &goalTheta);
    fclose(file);

    setStartState(startX, startY, startTheta);
    setGoalState(goalX, goalY, goalTheta);
}

void ArrtsService::readLimitsFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in readLimitsFromFile()");

    double minX, minY, maxX, maxY;

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");
    fscanf(file, "%lf,%lf", &minX, &minY);
    fscanf(file, "%lf,%lf", &maxX, &maxY);
    fclose(file);

    setLimits(minX, minY, maxX, maxY);
}

void ArrtsService::initializeFromDataDirectory(string dataDir)
{
    string statesFile = dataDir + "/" + DEFAULT_STATES_FILE;
    string limitsFile = dataDir + "/" + DEFAULT_LIMITS_FILE;
    string obstaclesFile = dataDir + "/" + DEFAULT_OBSTACLES_FILE;

    readLimitsFromFile(fopen(limitsFile.c_str(), "r"));
    readStatesFromFile(fopen(statesFile.c_str(), "r"));
    readObstaclesFromFile(fopen(obstaclesFile.c_str(), "r"));
}

vector<State> ArrtsService::calculatePath(double standoffRange, double positionBuffer, double freespaceBuffer)
{
    return vector<State>(0);
}