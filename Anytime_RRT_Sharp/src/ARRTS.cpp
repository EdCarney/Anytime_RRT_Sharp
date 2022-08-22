#include "ARRTS.hpp"

#define FUNC_PREFIX __func__

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
    if (obstacles == NULL)
    {
        obstacles = (Obstacle*) calloc(1, sizeof(Obstacle));
    }
    else
    {
        auto temp = (Obstacle*) calloc(numObstacles + 1, sizeof(Obstacle));
        copy(obstacles, obstacles + numObstacles, temp);
        delete obstacles;
        obstacles = temp;
    }
    obstacles[numObstacles++] = { x, y, r };
}

void ArrtsService::AddObstacles(const double* x, const double* y, const double* r, int numObs)
{
    if (obstacles == NULL)
    {
        obstacles = (Obstacle*) calloc(numObs, sizeof(Obstacle));
    }
    else
    {
        auto temp = (Obstacle*) calloc(numObstacles + numObs, sizeof(Obstacle));
        copy(obstacles, obstacles + numObstacles, temp);
        delete obstacles;
        obstacles = temp;
    }
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
    if (vehicleOutline == NULL)
    {
        vehicleOutline = (Node*) calloc(1, sizeof(Node));
    }
    else
    {
        auto temp = (Node*) calloc(numVehiclePoints + 1, sizeof(Node));
        copy(vehicleOutline, vehicleOutline + numVehiclePoints, temp);
        delete vehicleOutline;
        vehicleOutline = temp;
    }
    vehicleOutline[numVehiclePoints++] = { x, y };
}

void ArrtsService::AddVehiclePoints(const double* x, const double* y, int numPoints)
{
    if (vehicleOutline == NULL)
    {
        vehicleOutline = (Node*) calloc(numPoints, sizeof(Node));
    }
    else
    {
        auto temp = (Node*) calloc(numVehiclePoints + numPoints, sizeof(Node));
        copy(vehicleOutline, vehicleOutline + numVehiclePoints, temp);
        delete vehicleOutline;
        vehicleOutline = temp;
    }
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

template <typename T>
void ResetArraySize(T* arrStart, int oldSize, int newSize)
{
    if (arrStart == NULL)
    {
        arrStart = (T*) calloc(newSize, sizeof(T));
    }
    else
    {
        auto temp = (T*) calloc(oldSize + newSize, sizeof(T));
        copy(arrStart, arrStart + oldSize, temp);
        delete arrStart;
        arrStart = temp;
    }
}