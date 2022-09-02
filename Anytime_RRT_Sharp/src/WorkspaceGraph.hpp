#include <vector>
#include "Geometry.hpp"
#include "Obstacle.hpp"
#include "Vehicle.hpp"

#ifndef WORKSPACE_H
#define WORKSPACE_H

class WorkspaceGraph : Rectangle
{
    GoalState _goalRegion;
    vector<Obstacle> _obstacles;
    Vehicle _vehicle;
    void _buildWorkspaceGraph();
    bool _goalRegionReached;

    public:

        Vehicle vehicle();
        void setVehicle(Vehicle v);
        GoalState goalRegion();
        void setGoalRegion(double x, double y, double theta, double radius);
        vector<Obstacle> obstacles();
        Obstacle obstacles(int i);
        void defineFreespace(double minX, double minY, double maxX, double maxY);
        bool checkAtGoal(GraphNode node);
        bool nodeIsSafe(Point p);
        bool pathIsSafe(Point p1, Point p2);
        bool pathIsSafe(Point p1, vector<Point> points);
        bool obstacleInFreespace(double xObs, double yObs, double radiusObs);
        void addObstacle(double xObs, double yObs, double radiusObs);
        bool readObstaclesFromFile(const char* obstacleFile);
        double computeObsVol();
        bool atGate(GraphNode node);
        WorkspaceGraph() { _buildWorkspaceGraph(); }
};

#endif //WORKSPACE_H