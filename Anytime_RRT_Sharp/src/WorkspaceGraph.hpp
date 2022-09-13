#include <vector>
#include "Geometry.hpp"
#include "Obstacle.hpp"
#include "Vehicle.hpp"

#ifndef WORKSPACE_H
#define WORKSPACE_H

class WorkspaceGraph : public Rectangle
{
    GoalState _goalRegion;
    vector<SphereObstacle> _obstacles;
    Vehicle _vehicle;
    void _buildWorkspaceGraph();
    bool _goalRegionReached;
    bool _obstacleInFreespace(double x, double y, double z, double radius) const;
    bool _obstacleInFreespace(SphereObstacle o) const;

    public:

        Vehicle vehicle();
        void setVehicle(Vehicle v);
        GoalState goalRegion();
        void setGoalRegion(double x, double y, double z, double theta, double radius);
        void setGoalRegion(State goalState, double radius);
        vector<SphereObstacle> obstacles();
        SphereObstacle obstacles(int i);
        void defineFreespace(Rectangle limits);
        bool checkAtGoal(GraphNode node);
        bool nodeIsSafe(Point p);
        bool pathIsSafe(Point p1, Point p2);
        bool pathIsSafe(Point p1, vector<Point> points);
        void addObstacle(double x, double y, double z, double radius);
        void addObstacles(vector<SphereObstacle> obstacles);
        bool atGate(GraphNode node);
        WorkspaceGraph() { _buildWorkspaceGraph(); }
};

#endif //WORKSPACE_H