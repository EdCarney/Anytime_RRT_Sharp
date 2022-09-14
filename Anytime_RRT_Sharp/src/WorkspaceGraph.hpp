#include <vector>
#include "Geometry2D.hpp"
#include "Geometry3D.hpp"
#include "Vehicle.hpp"

#ifndef WORKSPACE_H
#define WORKSPACE_H

class WorkspaceGraph : public Rectangle
{
    GoalState _goalRegion;
    vector<Sphere> _obstacles;
    Vehicle _vehicle;
    void _buildWorkspaceGraph();
    bool _goalRegionReached;
    bool _obstacleInFreespace(double x, double y, double z, double radius) const;
    bool _obstacleInFreespace(Sphere o) const;

    public:

        Vehicle vehicle();
        void setVehicle(Vehicle v);
        GoalState goalRegion();
        void setGoalRegion(double x, double y, double z, double theta, double radius);
        void setGoalRegion(State goalState, double radius);
        vector<Sphere> obstacles();
        Sphere obstacles(int i);
        void defineFreespace(Rectangle limits);
        bool checkAtGoal(GraphNode node);
        bool nodeIsSafe(Point p);
        bool pathIsSafe(Point p1, Point p2);
        bool pathIsSafe(Point p1, vector<Point> points);
        void addObstacle(double x, double y, double z, double radius);
        void addObstacles(vector<Sphere> obstacles);
        bool atGate(GraphNode node);
        WorkspaceGraph() { _buildWorkspaceGraph(); }
};

#endif //WORKSPACE_H