#include <vector>
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"
#include "Dubins3d/src/DubinsManeuver3d.hpp"

using namespace std;

#ifndef CONFIGSPACE_NODE_H
#define CONFIGSPACE_NODE_H

class ConfigspaceNode : public GraphNode
{
    double _cost, _pathLength;
    vector<State> _pathTo;
    void _buildConfigspaceNode();
    void _buildConfigspaceNode(GraphNode node);
    void _buildConfigspaceNode(double x, double y, double z, double theta, double rho, int id, int parentId, double cost);

    public:
        ConfigspaceNode();
        ConfigspaceNode(GraphNode node);
        ConfigspaceNode(double x, double y, double z, double theta, double rho, int id, int parentId, double cost);
        double cost() const;
        double pathLength() const;
        void setCost(double cost);
        void generatePathFrom(State parentState);
};

#endif //CONFIGSPACE_NODE_H