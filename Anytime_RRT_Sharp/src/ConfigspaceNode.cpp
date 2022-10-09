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

const vector<State>& ConfigspaceNode::pathTo() const { return _pathTo; }

void ConfigspaceNode::setCost(double cost) { _cost = cost; }

void ConfigspaceNode::setPathTo(const vector<State>& pathTo)
{
    int size = pathTo.size();

    _pathTo.clear();
    _pathTo.resize(size);

    for (int i = 0; i < size; ++i)
        _pathTo[i] = State(pathTo.at(i).x(), pathTo.at(i).y(), pathTo.at(i).z(), pathTo.at(i).theta(), pathTo.at(i).rho());
}

void ConfigspaceNode::setPathTo(const vector<State3d>& pathTo)
{
    int size = pathTo.size();

    _pathTo.resize(size);

    for (int i = 0; i < size; ++i)
        _pathTo[i] = State(pathTo.at(i).x, pathTo.at(i).y, pathTo.at(i).z, pathTo.at(i).theta, pathTo.at(i).gamma);
}

void ConfigspaceNode::generatePathFrom(GraphNode parentState)
{
    auto path = DubinsEngine::generatePath(*this, parentState);

    if (path.empty())
        // no path possible
        return;

    setPathTo(path);
}