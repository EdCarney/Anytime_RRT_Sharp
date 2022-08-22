#include "ARRTS.hpp"

#define FUNC_PREFIX __func__

template <typename T>
void ResetArraySize(T** arrStart, int oldSize, int newSize)
{
    if (arrStart == NULL)
    {
        *arrStart = (T*) calloc(newSize, sizeof(T));
    }
    else
    {
        auto temp = (T*) calloc(oldSize + newSize, sizeof(T));
        copy(*arrStart, *arrStart + oldSize, temp);
        delete *arrStart;
        *arrStart = temp;
    }
}

ArrtsService::ArrtsService()
{
    numObstacles = 0;
    numVehiclePoints = 0;
    obstacles = NULL;
    vehicleOutline = NULL;
}

void ArrtsService::SetGoalPosition(double x, double y, double theta)
{
    goalPosition = { x, y, theta };
}

Position ArrtsService::GetGoalPosition()
{
    return goalPosition;
}

void ArrtsService::SetStartPosition(double x, double y, double theta)
{
    startPosition = { x, y, theta };
}

Position ArrtsService::GetStartPosition()
{
    return startPosition;
}

void ArrtsService::AddObstacle(double x, double y, double r)
{
    ResetArraySize<Obstacle>(&obstacles, numObstacles, numObstacles + 1);
    obstacles[numObstacles++] = { x, y, r };
}

void ArrtsService::AddObstacles(const double* x, const double* y, const double* r, int numObs)
{
    ResetArraySize<Obstacle>(&obstacles, numObstacles, numObstacles + numObs);
    for (int i = 0; i < numObs; ++i)
        obstacles[numObstacles++] = { x[i], y[i], r[i] };
}

Obstacle* ArrtsService::GetObstacles()
{
    return obstacles;
}

Obstacle ArrtsService::GetObstacle(int i)
{
    if (i >= numObstacles || i < 0)
        throw runtime_error("ERROR: Attempt to read index beyond array limits in GetObstacle");

    return obstacles[i];
}

int ArrtsService::GetNumObstacles()
{
    return numObstacles;
}

void ArrtsService::AddVehiclePoint(double x, double y)
{
    ResetArraySize<Node>(&vehicleOutline, numVehiclePoints, numVehiclePoints + 1);
    vehicleOutline[numVehiclePoints++] = { x, y };
}

void ArrtsService::AddVehiclePoints(const double* x, const double* y, int numPoints)
{
    ResetArraySize<Node>(&vehicleOutline, numVehiclePoints, numVehiclePoints + numPoints);
    for (int i = 0; i < numPoints; ++i)
        vehicleOutline[numVehiclePoints++] = { x[i], y[i] };
}

Node* ArrtsService::GetVehiclePoints()
{
    return vehicleOutline;
}

Node ArrtsService::GetVehiclePoint(int i)
{
    if (i >= numVehiclePoints || i < 0)
        throw runtime_error("ERROR: Attempt to read index beyond array limits in GetVehiclePoint");

    return vehicleOutline[i];
}

int ArrtsService::GetNumVehiclePoints()
{
    return numVehiclePoints;
}