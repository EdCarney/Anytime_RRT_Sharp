#include "ArrtsParams.hpp"

ArrtsParams::ArrtsParams(State start, State goal, vector<SphereObstacle> obstacles, double goalRadius, int minNodeCount, int maxNieghborCount)
{
    _start = start;
    _goal = goal;
    _obstacles = obstacles;
    _goalRadius = goalRadius;
    _minNodeCount = minNodeCount;
    _maxNeighborCount = maxNieghborCount;

    _setLimitsFromStates();
    _removeObstaclesNotInLimits();
}

ArrtsParams::ArrtsParams(string dataDirectory, int minNodeCount, int maxNieghborCount)
{   
    printf("Initializing data from %s\n", dataDirectory.c_str());

    string statesFile = dataDirectory + "/" + DEFAULT_STATES_FILE;
    string vehicleFile = dataDirectory + "/" + DEFAULT_VEHICLE_FILE;
    string obstaclesFile = dataDirectory + "/" + DEFAULT_OBSTACLES_FILE;

    _readStatesFromFile(fopen(statesFile.c_str(), "r"));
    _readObstaclesFromFile(fopen(obstaclesFile.c_str(), "r"));
    _minNodeCount = minNodeCount;
    _maxNeighborCount = maxNieghborCount;

    _setLimitsFromStates();
    _removeObstaclesNotInLimits();
}

void ArrtsParams::_setLimitsFromStates()
{
    double minX, maxX, minY, maxY, minZ, maxZ;

    // 50% buffer
    double bufferX = abs(_start.x() - _goal.x());
    double bufferY = abs(_start.y() - _goal.y());
    double bufferZ = abs(_start.z() - _goal.z());
    double buffer = max({ bufferX, bufferY, bufferZ }) * 0.5;

    minX = _start.x() < _goal.x() ? _start.x() - buffer : _goal.x() - buffer;
    maxX = _start.x() > _goal.x() ? _start.x() + buffer : _goal.x() + buffer;
    minY = _start.y() < _goal.y() ? _start.y() - buffer : _goal.y() - buffer;
    maxY = _start.y() > _goal.y() ? _start.y() + buffer : _goal.y() + buffer;
    minZ = _start.z() < _goal.z() ? _start.z() - buffer : _goal.z() - buffer;
    maxZ = _start.z() > _goal.z() ? _start.z() + buffer : _goal.z() + buffer;

    _limits = Rectangle(minX, minY, minZ, maxX, maxY, maxZ);
}

void ArrtsParams::_removeObstaclesNotInLimits()
{
    vector<SphereObstacle> newObstacles;
    for (SphereObstacle o : _obstacles)
        if (o.intersects(_limits))
            newObstacles.push_back(o);
    _obstacles = newObstacles;
    _calculateObstacleVolume();
}

void ArrtsParams::_calculateObstacleVolume()
{
    _obstacleVolume = 0.0;
    for (SphereObstacle o : _obstacles)
        _obstacleVolume += o.volume();
}

void ArrtsParams::_readStatesFromFile(FILE* file, bool isOptional)
{
    if (file == NULL && !isOptional)
    {
        if (!isOptional)
            throw runtime_error("NULL file pointer in _readStatesFromFile()");
        printf("WARN: Error loading state information, skipping...\n");
        return;
    }

    double startX, startY, startZ, startTheta;
    double goalX, goalY, goalZ, goalTheta, goalRadius;

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");
    fscanf(file, "%lf,%lf,%lf,%lf", &startX, &startY, &startZ, &startTheta);
    fscanf(file, "%lf,%lf,%lf,%lf,%lf", &goalX, &goalY, &goalZ, &goalTheta, &goalRadius);
    fclose(file);

    _start = State(startX, startY, startZ, startTheta);
    _goal = State(goalX, goalY, goalZ, goalTheta);
    _goalRadius = goalRadius;
}

void ArrtsParams::_readVehicleFromFile(FILE* file, bool isOptional)
{
    try
    {
        _vehicle = Vehicle(file);
    }
    catch(const std::exception& e)
    {
        if (!isOptional)
            throw;
        printf("WARN: Error loading vehicle information, skipping...\n");
    }
}

void ArrtsParams::_readObstaclesFromFile(FILE* file, bool isOptional)
{
    if (file == NULL && !isOptional)
    {
        if (!isOptional)
            throw runtime_error("NULL file pointer in readObstaclesFromFile()");
        printf("WARN: Error loading obstacle information, skipping...\n");
        return;
    }

    double x, y, z, r;
    _obstacles.clear();

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");
    while (fscanf(file, "%lf,%lf,%lf,%lf", &x, &y, &z, &r) != EOF)
        _obstacles.push_back(SphereObstacle(x, y, z, r));

    fclose(file);
}

int ArrtsParams::dimension() { return DIMENSION; }

int ArrtsParams::minNodeCount() { return _minNodeCount; }

int ArrtsParams::maxNeighborCount() { return _maxNeighborCount; }

double ArrtsParams::goalRadius() { return _goalRadius; }

double ArrtsParams::obstacleVolume() { return _obstacleVolume; }

State ArrtsParams::start() { return _start; }

State ArrtsParams::goal() { return _goal; }

Rectangle ArrtsParams::limits() { return _limits; }

Vehicle ArrtsParams::vehicle() { return _vehicle; }

vector<SphereObstacle> ArrtsParams::obstacles() { return _obstacles; }

SphereObstacle ArrtsParams::obstacles(int i) { return _obstacles[i]; }