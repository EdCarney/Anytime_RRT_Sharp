#include "WorkspaceGraph.hpp"

void WorkspaceGraph::_buildWorkspaceGraph()
{
    _minPoint = Point(0, 0, 0);
    _maxPoint = Point(0, 0, 0);
    _goalRegionReached = false;
}

void WorkspaceGraph::setGoalRegion(State goalState, double radius)
{
    _goalRegion = GoalState(goalState.x(), goalState.y(), goalState.z(), radius, goalState.theta());
}

void WorkspaceGraph::defineFreespace(Rectangle limits)
{
    _minPoint = limits.minPoint();
    _maxPoint = limits.maxPoint();
    _volume = _calculateVolume();
}

void WorkspaceGraph::addObstacle(double x, double y, double z, double radius)
{
    _obstacles.push_back(new Sphere(x, y, z, radius));
}

void WorkspaceGraph::addObstacles(vector<Shape3d*>& obstacles)
{
    for (auto o : obstacles)
        _obstacles.push_back(o);
}

bool WorkspaceGraph::atGate(GraphNode node)
{
    double dist = node.distanceTo(_goalRegion);
    return dist <= _goalRegion.radius();
}

bool WorkspaceGraph::nodeIsSafe(Point p)
{
    for (auto o : _obstacles)
        if (o->intersects(p))
            return false;
    return true;
}

bool WorkspaceGraph::pathIsSafe(Point p1, Point p2)
{
    // TODO: generate dubins path instead
    Line pathSegment(p1, p2);
    for (auto o : _obstacles)
        if (o->intersects(pathSegment))
            return false;
    return true;
}

bool WorkspaceGraph::checkAtGoal(GraphNode node)
{
    // update vehicle state to temp node
    _vehicle.updateState(node);

    double distToGoal = _vehicle.state().distanceTo(_goalRegion);
    return distToGoal < (_goalRegion.radius() + _vehicle.boundingRadius());
}

Vehicle WorkspaceGraph::vehicle() { return _vehicle; }

void WorkspaceGraph::setVehicle(Vehicle v) { _vehicle = v; }

GoalState WorkspaceGraph::goalRegion() { return _goalRegion; }