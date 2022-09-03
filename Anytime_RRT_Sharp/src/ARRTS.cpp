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

void ArrtsService::setStartState(double x, double y, double theta)
{
    _startState = { x, y, theta };
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

Vehicle ArrtsService::vehicle() const
{
    return _vehicle;
}

void ArrtsService::setVehicle(vector<double> x, vector<double> y)
{
    _vehicle = Vehicle(x, y);
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

void ArrtsService::readVehicleFromFile(FILE* file)
{
    _vehicle = Vehicle(file);
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

void ArrtsService::initializeFromDataDirectory(string dataDir)
{
    string statesFile = dataDir + "/" + DEFAULT_STATES_FILE;
    string vehicleFile = dataDir + "/" + DEFAULT_VEHICLE_FILE;
    string obstaclesFile = dataDir + "/" + DEFAULT_OBSTACLES_FILE;

    readStatesFromFile(fopen(statesFile.c_str(), "r"));
    readVehicleFromFile(fopen(vehicleFile.c_str(), "r"));
    readObstaclesFromFile(fopen(obstaclesFile.c_str(), "r"));
}

vector<State> ArrtsService::calculatePath(double standoffRange, double positionBuffer, double freespaceBuffer)
{
    return vector<State>(0);
}