#include <vector>
#include "ManeuverEngine.hpp"
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"
#include "Vehicle.hpp"

#ifndef WORKSPACE_H
#define WORKSPACE_H

class WorkspaceGraph : public Rectangle
{
    GoalState _goalRegion;
    vector<Shape3d*> _obstacles;
    Vehicle _vehicle;
    void _buildWorkspaceGraph();
    bool _goalRegionReached;

    bool _nodeInFreespace(Point& point) const;

    public:
        void setGoalRegion(State goalState, double radius);
        void defineFreespace(Rectangle limits);
        bool checkAtGoal(const GraphNode node);
        bool nodeIsSafe(const Point p) const;
        bool pathIsSafe(const GraphNode g1, const GraphNode g2) const;
        bool pathIsSafe(const vector<State>& path) const;
        void addObstacle(double x, double y, double z, double radius);
        void addObstacles(vector<Shape3d*>& obstacles);
        bool atGate(GraphNode node);
        Vehicle vehicle();
        void setVehicle(Vehicle v);
        GoalState goalRegion();
        WorkspaceGraph() { _buildWorkspaceGraph(); }
};

#endif //WORKSPACE_H