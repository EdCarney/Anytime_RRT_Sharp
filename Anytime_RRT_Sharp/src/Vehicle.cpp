#include "Vehicle.hpp"

Vehicle::Vehicle()
{
    _buildVehicle();
}

Vehicle::Vehicle(vector<double> x, vector<double> y, vector<double> z)
{
    _buildVehicle();
    addOffsetNodes(x, y, z);
}

Vehicle::Vehicle(string fileName)
{
    _buildVehicle();
    addOffsetNodesFromFile(fileName);
}

void Vehicle::_buildVehicle()
{
    _state = { 0, 0, 0, 0, 0 };
    _boundingRadius = 0;
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
    double xSum = 0, ySum = 0, zSum = 0;
    for (Point p : _nodes)
    {
        xSum += p.x();
        ySum += p.y();
        zSum += p.z();
    }
    int numNodes = _nodes.size();
    _centroid = { xSum / numNodes, ySum / numNodes, zSum / numNodes };
}

void Vehicle::_updateOffsetParams()
{
    _calculateCentroid();
    _calculateBoundingRadius();
    updateState(_state);
}

void Vehicle::updateState(State newState)
{
    _state = { newState.x(), newState.y(), newState.z(), newState.theta(), newState.rho() };

    double x, y, z;

    // update body nodes based on deltas
    for (int i = 0; i < _nodes.size(); i++)
    {
        x = _state.x() + cos(_state.theta()) * _offsetNodes[i].x() - sin(_state.theta()) * _offsetNodes[i].y();
        y = _state.y() + sin(_state.theta()) * _offsetNodes[i].x() + cos(_state.theta()) * _offsetNodes[i].y();
        z = 0; // TODO fix this to use rotation matrices!!
        _nodes[i] = Point(x, y, z);
    }
}

void Vehicle::addOffsetNode(double x, double y, double z)
{
    _offsetNodes.push_back(Point(x, y, z));
    _nodes.resize(_offsetNodes.size());
    _updateOffsetParams();
}

void Vehicle::addOffsetNodes(vector<double> x, vector<double> y, vector<double> z)
{
    if (x.size() != y.size() || z.size() != z.size())
        throw runtime_error("Inconsistent array size in addOffsetNodes()");

    int size = x.size();
    for (int i = 0; i < size; ++i)
    {
        _offsetNodes.push_back(Point(x[i], y[i], z[i]));
    }
    _nodes.resize(_offsetNodes.size());
    _updateOffsetParams();
}

void Vehicle::addOffsetNodesFromFile(string fileName)
{
    ifstream file(fileName);
    istringstream iss;
    string obstacleType, line;
    double xVal, yVal, zVal;
    vector<double> x, y, z;

    getline(file, line); // ignore first line (formatting)
    while (getline(file, line))
    {
        iss = istringstream(line);
        iss >> xVal >> yVal >> zVal;
        x.push_back(xVal);
        y.push_back(yVal);
        z.push_back(zVal);
    }

    file.close();
    addOffsetNodes(x, y, z);
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