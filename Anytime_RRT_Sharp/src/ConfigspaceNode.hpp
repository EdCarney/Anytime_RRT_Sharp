#include "Geometry2D.hpp"
#include "Geometry3D.hpp"

#ifndef CONFIGSPACE_NODE_H
#define CONFIGSPACE_NODE_H

class ConfigspaceNode : public GraphNode
{
    double _cost;
    void _buildConfigspaceNode();
    void _buildConfigspaceNode(GraphNode node);
    void _buildConfigspaceNode(double x, double y, double z, double theta, int id, int parentId, double cost);

    public:
        ConfigspaceNode();
        ConfigspaceNode(GraphNode node);
        ConfigspaceNode(double x, double y, double z, double theta, int id, int parentId, double cost);
        double cost();
        void setCost(double cost);
};

#endif //CONFIGSPACE_NODE_H