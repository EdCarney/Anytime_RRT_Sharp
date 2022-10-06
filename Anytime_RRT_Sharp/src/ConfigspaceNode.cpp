#include "ConfigspaceNode.hpp"

void ConfigspaceNode::_buildConfigspaceNode()
{
    _buildGraphNode();
    _cost = 0;
}

void ConfigspaceNode::_buildConfigspaceNode(GraphNode n)
{
    _buildGraphNode(n.x(), n.y(), n.z(), n.theta(), n.rho(), n.id(), n.parentId());
    _cost = 0;
}

void ConfigspaceNode::_buildConfigspaceNode(double x, double y, double z, double theta, double rho, int id, int parentId, double cost)
{
    _buildGraphNode(x, y, z, theta, rho, id, parentId);
    _cost = cost;
}

ConfigspaceNode::ConfigspaceNode()
{
    _buildConfigspaceNode();
}

ConfigspaceNode::ConfigspaceNode(GraphNode node)
{
    _buildConfigspaceNode(node);
}

ConfigspaceNode::ConfigspaceNode(double x, double y, double z, double theta, double rho, int id, int parentId, double cost)
{
    _buildConfigspaceNode(x, y, z, theta, rho, id, parentId, cost);
}

double ConfigspaceNode::cost() const { return _cost; }

double ConfigspaceNode::pathLength() const { return _pathLength; }

void ConfigspaceNode::setCost(double cost) { _cost = cost; }

void ConfigspaceNode::generatePathFrom(State parentState)
{
    State3d qi { parentState.x(), parentState.y(), parentState.z(), parentState.theta(), parentState.rho() };
    State3d qf { x(), y(), z(), theta(), rho() };

    double rhoMin = 10;
    tuple<double, double> pitchLims = { -15.0 * M_PI / 180.0, 15.0 * M_PI / 180.0 };

    DubinsManeuver3d maneuver(qi, qf, rhoMin, pitchLims);

    _pathLength = maneuver.length();

    if (_pathLength < 0)
        // no path possible
        return;
    
    vector<State3d> states = maneuver.computeSampling(100);
    _pathTo.resize(states.size());

    State temp;
    for (int i = 0; i < states.size(); ++i)
    {
        temp = State(states.at(i).x, states.at(i).y, states.at(i).z, states.at(i).theta, states.at(i).gamma);
        _pathTo[i] = temp;
    }
}