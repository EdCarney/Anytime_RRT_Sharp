#include "Vehicle.hpp"

Vehicle::Vehicle()
{
    _buildVehicle();
}

Vehicle::Vehicle(const double* x, const double *y, int numPoints)
{
    _buildVehicle();
    addOffsetNodes(x, y, numPoints);
}

void Vehicle::_buildVehicle()
{
    _state = { 0.0, 0.0, 0.0 };
    _boundingRadius = 0.0;
}

void Vehicle::_calculateBoundingRadius()
{
    double maxRadius = 0;
    for (Point p : _offsetNodes)
        maxRadius = max(maxRadius, p.distanceTo(_centroid));
    _boundingRadius = maxRadius;
}

void Vehicle::_calculateCentroid()
{
    double xSum = 0, ySum = 0;
    for (Point p : _nodes)
    {
        xSum += p.x();
        ySum += p.y();
    }
    _centroid = { xSum / _nodes.size(), ySum / _nodes.size() };
}

void Vehicle::_updateOffsetParams()
{
    _calculateCentroid();
    _calculateBoundingRadius();
    updateState(_state);
}

void Vehicle::updateState(State newState)
{
    _state = { newState.x(), newState.y(), newState.theta() };

    double x, y;

    // update body nodes based on deltas
    for (int i = 0; i < _nodes.size(); i++)
    {
        x = _state.x() + cos(_state.theta()) * _offsetNodes[i].x() - sin(_state.theta()) * _offsetNodes[i].y();
        y = _state.y() + sin(_state.theta()) * _offsetNodes[i].x() + cos(_state.theta()) * _offsetNodes[i].y();
        _nodes[i] = Point(x, y);
    }
}

void Vehicle::addOffsetNode(double x, double y)
{
    _offsetNodes.push_back(Point(x, y));
    _nodes.resize(_offsetNodes.size());
    _updateOffsetParams();
}

void Vehicle::addOffsetNodes(const double* x, const double* y, int numPoints)
{
    for (int i = 0; i < numPoints; ++i)
        _offsetNodes.push_back(Point(x[i], y[i]));
    _nodes.resize(_offsetNodes.size());
    _updateOffsetParams();
}

void Vehicle::addOffsetNodesFromFile(FILE* file)
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
    addOffsetNodes(x, y, pointCount);
}

vector<Point> Vehicle::nodes()
{
    return _nodes;
}

Point Vehicle::nodes(int i)
{
    if (i >= _nodes.size() || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetNode");

    return _nodes[i];
}

vector<Point> Vehicle::offsetNodes()
{
    return _offsetNodes;
}

Point Vehicle::offsetNodes(int i)
{
    if (i >= _offsetNodes.size() || i < 0)
        throw runtime_error("Attempt to read index beyond array limits in GetOffsetNode");

    return _offsetNodes[i];
}

State Vehicle::state()
{
    return _state;
}

double Vehicle::boundingRadius()
{
    return _boundingRadius;
}