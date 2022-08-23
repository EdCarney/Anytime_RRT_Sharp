#include "ARRTS.hpp"

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

void ArrtsService::AddVehiclePointsFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in AddVehiclePointsFromFile");

    // get num of points for allocation
    int pointCount = 0;
    double xVal, yVal;
    while (fscanf(file, "%lf,%lf", &xVal, &yVal) != EOF)
        pointCount++;

    // get points
    double* x = new double[pointCount];
    double* y = new double[pointCount];
    rewind(file);
    for (int i = 0; i < pointCount; ++i)
    {
        fscanf(file, "%lf,%lf", &xVal, &yVal);
        x[i] = xVal;
        y[i] = yVal;
    }

    // close file
    fclose(file);

    // add points
    AddVehiclePoints(x, y, pointCount);
}

Node* ArrtsService::GetVehiclePoints()
{
    return vehicleOutline;
}

Node ArrtsService::GetVehiclePoint(int i)
{
    if (i >= numVehiclePoints || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetVehiclePoint");

    return vehicleOutline[i];
}

int ArrtsService::GetNumVehiclePoints()
{
    return numVehiclePoints;
}

Position* ArrtsService::CalculatePath(double standoffRange, double positionBuffer, double freespaceBuffer)
{

}