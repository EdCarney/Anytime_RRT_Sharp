#include "Vehicle.hpp"

Vehicle::Vehicle()
{
    _buildVehicle();
}

Vehicle::Vehicle(vector<double> x, vector<double> y)
{
    _buildVehicle();
    addOffsetNodes(x, y);
}

Vehicle::Vehicle(FILE* file)
{
    _buildVehicle();
    addOffsetNodesFromFile(file);
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

void Vehicle::addOffsetNodes(vector<double> x, vector<double> y)
{
    if (x.size() != y.size())
        throw runtime_error("Inconsistent array size in addOffsetNodes()");

    int size = x.size();
    for (int i = 0; i < size; ++i)
    {
        _offsetNodes.push_back(Point(x[i], y[i]));
    }
    _nodes.resize(_offsetNodes.size());
    _updateOffsetParams();
}

void Vehicle::addOffsetNodesFromFile(FILE* file)
{
    if (file == NULL)
        throw runtime_error("NULL file pointer in AddOffsetNodesFromFile");

    // ignore first line (formatting)
    fscanf(file, "%*[^\n]\n");

    double xVal, yVal;
    vector<double> x, y;
    while (fscanf(file, "%lf,%lf", &xVal, &yVal) != EOF)
    {
        x.push_back(xVal);
        y.push_back(yVal);
    }

    fclose(file);
    addOffsetNodes(x, y);
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
        throw runtime_error("Attempt to read index beyond array limits in offsetNodes()");

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