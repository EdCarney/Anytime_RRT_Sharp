#include "ConfigspaceNode.hpp"

void ConfigspaceNode::_buildConfigspaceNode()
{
    _buildGraphNode();
    _cost = 0;
}

void ConfigspaceNode::_buildConfigspaceNode(GraphNode n)
{
    _buildGraphNode(n.x(), n.y(), n.theta(), n.id(), n.parentId());
    _cost = 0;
}

void ConfigspaceNode::_buildConfigspaceNode(double x, double y, double theta, int id, int parentId, double cost)
{
    _buildGraphNode(x, y, theta, id, parentId);
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

ConfigspaceNode::ConfigspaceNode(double x, double y, double theta, int id, int parentId, double cost)
{
    _buildConfigspaceNode(x, y, theta, id, parentId, cost);
}

double ConfigspaceNode::cost()
{
    return _cost;
}

void ConfigspaceNode::setCost(double cost)
{
    _cost = cost;
}