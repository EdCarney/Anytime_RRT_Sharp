#include "ARRTS.hpp"

ArrtsService::ArrtsService()
{
    numObstacles = 0;
    numVehiclePoints = 0;
    obstacles = NULL;
    vehicleOutline = NULL;
}

void ArrtsService::SetGoalState(double x, double y, double theta)
{
    goalState = { x, y, theta };
}

State ArrtsService::GetGoalState()
{
    return goalState;
}

void ArrtsService::SetStartState(double x, double y, double theta)
{
    startState = { x, y, theta };
}

State ArrtsService::GetStartState()
{
    return startState;
}

void ArrtsService::AddObstacle(double x, double y, double r)
{
    ResetArraySize<Obstacle>(&obstacles, numObstacles, numObstacles + 1);
    obstacles[numObstacles++] = Obstacle(x, y, r);
}

void ArrtsService::AddObstacles(const double* x, const double* y, const double* r, int numObs)
{
    ResetArraySize<Obstacle>(&obstacles, numObstacles, numObstacles + numObs);
    for (int i = 0; i < numObs; ++i)
        obstacles[numObstacles++] = Obstacle(x[i], y[i], r[i]);
}

void ArrtsService::AddObstaclesFromFile(FILE* file)
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
    AddObstacles(x, y, r, obsCount);
}

Obstacle* ArrtsService::GetObstacles()
{
    return obstacles;
}

Obstacle ArrtsService::GetObstacle(int i)
{
    if (i >= numObstacles || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetObstacle");

    return obstacles[i];
}

int ArrtsService::GetNumObstacles()
{
    return numObstacles;
}

State* ArrtsService::CalculatePath(double standoffRange, double positionBuffer, double freespaceBuffer)
{

}