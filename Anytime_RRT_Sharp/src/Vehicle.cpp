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
    state = { 0.0, 0.0, 0.0 };
    boundingRadius = 0.0;
}

void Vehicle::calculateBoundingRadius()
{
    double maxRadius = 0;
    for (Point p : offsetNodes)
        maxRadius = max(maxRadius, hypot(p.GetX() - centroid.GetX(), p.GetY() - centroid.GetY()));
    boundingRadius = maxRadius;
}

void Vehicle::calculateCentroid()
{
    double xSum = 0, ySum = 0;
    for (Point p : nodes)
    {
        xSum += p.GetX();
        ySum += p.GetY();
    }
    centroid = { xSum / nodes.size(), ySum / nodes.size() };
}

void Vehicle::updateOffsetParams()
{
    calculateCentroid();
    calculateBoundingRadius();
    UpdateState(state);
}

void Vehicle::UpdateState(State newState)
{
    state = { newState.GetX(), newState.GetY(), newState.GetTheta() };

    double x, y;

    // update body nodes based on deltas
    for (int i = 0; i < nodes.size(); i++)
    {
        x = state.GetX() + cos(state.GetTheta()) * offsetNodes[i].GetX() - sin(state.GetTheta()) * offsetNodes[i].GetY();
        y = state.GetY() + sin(state.GetTheta()) * offsetNodes[i].GetX() + cos(state.GetTheta()) * offsetNodes[i].GetY();
        nodes[i] = Point(x, y);
    }
}

void Vehicle::AddOffsetNode(double x, double y)
{
    offsetNodes.push_back(Point(x, y));
    nodes.resize(offsetNodes.size());
    updateOffsetParams();
}

void Vehicle::AddOffsetNodes(const double* x, const double* y, int numPoints)
{
    for (int i = 0; i < numPoints; ++i)
        offsetNodes.push_back(Point(x[i], y[i]));
    nodes.resize(offsetNodes.size());
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
    auto retVal = (Point*) calloc(nodes.size(), sizeof(Point));
    for (int i = 0; i < nodes.size(); ++i)
        retVal[i] = Point(nodes[i].GetX(), nodes[i].GetY());
    return nodes.size() > 0 ? retVal : NULL;
}

Point Vehicle::GetNode(int i)
{
    if (i >= nodes.size() || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetNode");

    return nodes[i];
}

Point* Vehicle::GetOffsetNodes()
{
    auto retVal = (Point*) calloc(offsetNodes.size(), sizeof(Point));
    for (int i = 0; i < offsetNodes.size(); ++i)
        retVal[i] = Point(offsetNodes[i].GetX(), offsetNodes[i].GetY());
    return offsetNodes.size() > 0 ? retVal : NULL;
}

Point Vehicle::GetOffsetNode(int i)
{
    if (i >= offsetNodes.size() || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetOffsetNode");

    return nodes[i];
}

State Vehicle::GetState()
{
    return state;
}

int Vehicle::GetNumNodes()
{
    return nodes.size();
}

double Vehicle::GetBoundingRadius()
{
    return boundingRadius;
}