#include "ArrtsParams.hpp"

ArrtsParams::ArrtsParams(State start, State goal, vector<Shape3d*> obstacles, double goalRadius, int minNodeCount, int maxNieghborCount)
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
    _readObstaclesFromFile(obstaclesFile);
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
    vector<Shape3d*> newObstacles;
    for (auto o : _obstacles)
        if (o->intersects(_limits))
            newObstacles.push_back(o);
    _obstacles = newObstacles;
    _calculateObstacleVolume();
}

void ArrtsParams::_calculateObstacleVolume()
{
    _obstacleVolume = 0.0;
    for (auto o : _obstacles)
        _obstacleVolume += o->volume();
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

void ArrtsParams::_readObstaclesFromFile(string fileName, bool isOptional)
{
    ifstream file(fileName);

    string obstacleType, line;
    istringstream iss;
    double x, y, z, r;
    double minX, minY, minZ, maxX, maxY, maxZ;

    _obstacles.clear();

    while (getline(file, line))
    {
        obstacleType = line.substr(0, line.find(" "));
        line = line.substr(line.find(" ") + 1);
        iss = istringstream(line);
        if (obstacleType == "SPHERE")
        {
            iss >> x >> y >> z >> r;
            _obstacles.push_back(new Sphere(x, y, z, r));
        }
        else if(obstacleType == "RECTANGLE")
        {
            iss >> minX >> minY >> minZ >> maxX >> maxY >> maxZ;
            _obstacles.push_back(new Rectangle(minX, minY, minZ, maxX, maxY, maxZ));
        }
    }
    file.close();
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

vector<Shape3d*>& ArrtsParams::obstacles() { return _obstacles; }

Shape3d* const ArrtsParams::obstacles(int i) { return _obstacles[i]; }