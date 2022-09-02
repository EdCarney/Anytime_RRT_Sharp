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
    bool _obstacleInFreespace(double x, double y, double radius) const;
    bool _obstacleInFreespace(Obstacle o) const;

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
        void addObstacle(double x, double y, double radius);
        void addObstacles(vector<Obstacle> obstacles);
        bool readObstaclesFromFile(const char* obstacleFile);
        double obstacleVolume() const;
        bool atGate(GraphNode node);
        WorkspaceGraph() { _buildWorkspaceGraph(); }
};

#endif //WORKSPACE_H