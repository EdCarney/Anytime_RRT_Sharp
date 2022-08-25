#include "Vehicle.hpp"

Vehicle::Vehicle()
{
    buildVehicle();
}

Vehicle::Vehicle(const double* x, const double *y, int numPoints)
{
    buildVehicle();
    AddOffsetNodes(x, y, numPoints);
}

void Vehicle::buildVehicle()
{
    nodes = NULL;
    offsetNodes = NULL;
    numNodes = 0;
    state = { 0.0, 0.0, 0.0 };
    boundingRadius = 0.0;
}

void Vehicle::calculateBoundingRadius()
{
    double maxRadius = 0, tempMaxRadius = 0;
    for (int i = 0; i < numNodes; ++i)
    {
        tempMaxRadius = hypot(offsetNodes[i].x - centroid.x, offsetNodes[i].y - centroid.y);
        if (tempMaxRadius > maxRadius)
            maxRadius = tempMaxRadius;
    }
    boundingRadius = maxRadius;
}

void Vehicle::calculateCentroid()
{
    double xSum = 0, ySum = 0;
    for (int i = 0; i < numNodes; ++i)
    {
        xSum += nodes[i].x;
        ySum += nodes[i].y;
    }
    centroid = { xSum / numNodes, ySum / numNodes };
}

void Vehicle::updateOffsetParams()
{
    calculateCentroid();
    calculateBoundingRadius();
    UpdateState(state);
}

void Vehicle::UpdateState(State newState)
{
    state = { newState.x, newState.y, newState.theta };

    // update body nodes based on deltas
    for (int i = 0; i < numNodes; i++)
    {
        nodes[i].x = state.x + cos(state.theta) * offsetNodes[i].x - sin(state.theta) * offsetNodes[i].y;
        nodes[i].y = state.y + sin(state.theta) * offsetNodes[i].x + cos(state.theta) * offsetNodes[i].y;
    }
}

void Vehicle::AddOffsetNode(double x, double y)
{
    ResetArraySize<Point>(&nodes, numNodes, numNodes + 1);
    ResetArraySize<Point>(&offsetNodes, numNodes, numNodes + 1);
    offsetNodes[numNodes++] = { x, y };
    updateOffsetParams();
}

void Vehicle::AddOffsetNodes(const double* x, const double* y, int numPoints)
{
    ResetArraySize<Point>(&nodes, numNodes, numNodes + numPoints);
    ResetArraySize<Point>(&offsetNodes, numNodes, numNodes + numPoints);
    for (int i = 0; i < numPoints; ++i)
        offsetNodes[numNodes++] = { x[i], y[i] };
    updateOffsetParams();
}

void Vehicle::AddOffsetNodesFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in AddOffsetNodesFromFile");

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
    AddOffsetNodes(x, y, pointCount);
}

Point* Vehicle::GetNodes()
{
    return nodes;
}

Point Vehicle::GetNode(int i)
{
    if (i >= numNodes || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetNode");

    return nodes[i];
}

Point* Vehicle::GetOffsetNodes()
{
    return offsetNodes;
}

Point Vehicle::GetOffsetNode(int i)
{
    if (i >= numNodes || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetOffsetNode");

    return nodes[i];
}

State Vehicle::GetState()
{
    return state;
}

int Vehicle::GetNumNodes()
{
    return numNodes;
}

double Vehicle::GetBoundingRadius()
{
    return boundingRadius;
}