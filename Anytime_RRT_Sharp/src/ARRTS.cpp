#include "ARRTS.hpp"

ArrtsService::ArrtsService() { }

ArrtsService::ArrtsService(string dataDirectory)
{
    initializeFromDataDirectory(dataDirectory);
}

//TEMP
double ArrtsService::obstacleVolume()
{
    return _obstacleVolume;
}

void ArrtsService::_calculateObstacleVolume()
{
    _obstacleVolume = 0.0;
    for (Obstacle o : _obstacles)
        _obstacleVolume += o.area();
}

void ArrtsService::_updateLimitsFromStates()
{
    double minX, maxX, minY, maxY;

    // include buffer percentage
    double bufferX = abs(startState().x() - goalState().x());
    double bufferY = abs(startState().y() - goalState().y());
    double buffer = max(bufferX, bufferY);
    buffer *= 0.5;

    if (startState().x() < goalState().x())
    {
        minX = startState().x() - buffer;
        maxX = goalState().x() + buffer;
    }
    else
    {
        minX = goalState().x() - buffer;
        maxX = startState().x() + buffer;
    }

    if (startState().y() < goalState().y())
    {
        minY = startState().y() - buffer;
        maxY = goalState().y() + buffer;
    }
    else
    {
        minY = goalState().y() - buffer;
        maxY = startState().y() + buffer;
    }
    
    setLimits(minX, minY, maxX, maxY);
}

void ArrtsService::_removeObstaclesNotInLimits()
{
    vector<Obstacle> newObstacles;
    for (Obstacle o : obstacles())
        if (o.intersects(_limits))
            newObstacles.push_back(o);
    _obstacles = newObstacles;
    _calculateObstacleVolume();
}

void ArrtsService::_configureWorkspace()
{
    _workspaceGraph.setGoalRegion(goalState(), _goalRadius);
    _workspaceGraph.defineFreespace(limits());
    _workspaceGraph.addObstacles(obstacles());
    _workspaceGraph.setVehicle(vehicle());
}

void ArrtsService::_configureConfigspace()
{
    _configspaceGraph.defineFreespace(_limits, _dimension, _obstacleVolume);
    _configspaceGraph.setRootNode(goalState());
}

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
    _calculateObstacleVolume();
}

void ArrtsService::addObstacles(const vector<double>& x, const vector<double>& y, const vector<double>& r)
{
    int size = x.size();
    for (int i = 0; i < size; ++i)
        _obstacles.push_back(Obstacle(x[i], y[i], r[i]));
    _calculateObstacleVolume();
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

void ArrtsService::initializeFromDataDirectory(string dataDirectory)
{
    string statesFile = dataDirectory + "/" + DEFAULT_STATES_FILE;
    string vehicleFile = dataDirectory + "/" + DEFAULT_VEHICLE_FILE;
    string obstaclesFile = dataDirectory + "/" + DEFAULT_OBSTACLES_FILE;

    readStatesFromFile(fopen(statesFile.c_str(), "r"));
    readVehicleFromFile(fopen(vehicleFile.c_str(), "r"));
    readObstaclesFromFile(fopen(obstaclesFile.c_str(), "r"));

    _updateLimitsFromStates();
    _removeObstaclesNotInLimits();
}

vector<State> ArrtsService::calculatePath(double goalRadius)
{
    _goalRadius = goalRadius;
    _configureWorkspace();
    _configureConfigspace();
    return vector<State>(0);
}