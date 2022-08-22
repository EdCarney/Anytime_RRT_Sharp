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
        obstacles[0] = { x, y, r };
        numObstacles = 1;
    }
    else
    {
        auto temp = (Obstacle*) calloc(numObstacles + 1, sizeof(Obstacle));
        copy(obstacles, obstacles + numObstacles, temp);
        delete obstacles;
        obstacles = temp;
        obstacles[numObstacles++] = { x, y, r };
    }
}

void ArrtsService::AddObstacles(const double* x, const double* y, const double* r, int numObs)
{
    if (obstacles == NULL)
    {
        obstacles = (Obstacle*) calloc(numObs, sizeof(Obstacle));
        for (int i = 0; i < numObs; ++i)
            obstacles[numObstacles++] = { x[i], y[i], r[i] };
    }
    else
    {
        auto temp = (Obstacle*) calloc(numObstacles + numObs, sizeof(Obstacle));
        copy(obstacles, obstacles + numObstacles, temp);
        delete obstacles;
        obstacles = temp;
        for (int i = 0; i < numObs; ++i)
            obstacles[numObstacles++] = { x[i], y[i], r[i] };
    }
}

Obstacle* ArrtsService::GetObstacles()
{
    return obstacles;
}

Obstacle ArrtsService::GetObstacle(int i)
{
    if (i >= numObstacles)
        throw runtime_error("ERROR: Attempt to read index beyond obstacle array in GetObstacle");

    return obstacles[i];
}

int ArrtsService::GetNumObstacles()
{
    return numObstacles;
}