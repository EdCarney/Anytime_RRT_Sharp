#include <vector>
#include "DubinsEngine.hpp"
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"

using namespace std;

#ifndef CONFIGSPACE_NODE_H
#define CONFIGSPACE_NODE_H

class ConfigspaceNode : public GraphNode
{
    double _cost, _pathLength;
    vector<State> _pathTo;
    void _buildConfigspaceNode();
    void _buildConfigspaceNode(GraphNode node);
    void _buildConfigspaceNode(double x, double y, double z, double theta, double rho, unsigned long id, unsigned long parentId, double cost);

    public:
        ConfigspaceNode();
        ConfigspaceNode(GraphNode node);
        ConfigspaceNode(double x, double y, double z, double theta, double rho, unsigned long id, unsigned long parentId, double cost);
        double cost() const;
        double pathLength() const;
        const vector<State>& pathTo() const;
        void setCost(double cost);
        void setPathTo(const vector<State>& pathTo);
        void setPathTo(const vector<State3d>& pathTo);
        void generatePathFrom(GraphNode parentState);
};

#endif //CONFIGSPACE_NODE_H