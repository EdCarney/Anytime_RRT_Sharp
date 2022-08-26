#include <vector>
#include "configspaceGraph.hpp"
#include "Geometry.hpp"
#include "Obstacle.hpp"
#include "Utility.hpp"
#include "Vehicle.hpp"

#ifndef WORKSPACE_H
#define WORKSPACE_H

class WorkspaceGraph
{
    void _buildWorkspaceGraph();

public:
    vector<Obstacle> obstacles;
    Vehicle vehicle;
    GoalState goalRegion;

    double minX, minY, maxX, maxY;    // limits of the graph freespace
    double minTheta, maxTheta;        // limits of the orientation theta

    bool goalRegionReached;            // boolean value to indicate if the goal region has been reached

    // sets the goal region for the graph
    // this is treated similar to an obstacle
    void addGoalRegion(double x, double y, double theta, double radius);

    // updates the position and configuration of the graph goal region
    void updateGoalRegion(double x, double y, double theta, double radius);

    // defines freespace for problem
    // used when extending to a new node
    void defineFreespace(double minX, double minY, double minTheta, double maxX,
        double maxY, double maxTheta);

    // checks if specified node is in the goal region
    // returns true if it is and false otherwise
    bool checkAtGoal(ConfigspaceNode node);

    // checks if any vehicle collides with any obstacle at the provided configuration point
    // returns true if there is a collision and false otherwise
    bool checkForCollision(ConfigspaceNode node);

    // attempts to build a path from a parent configuration node to a new configuration node
    // uses interpolation with a step size of delta and a maximum extension of epsilon
    ConfigspaceNode extendToNode(ConfigspaceNode parentNode, ConfigspaceNode newNode, double epsilon);

    // attempt to connect two nodes; used for rewiring the graph for RRT*
    // will attempt to go from parent node to new node until the difference is less then some epsilon
    // returns zero if successful and one otherwise
    ConfigspaceNode connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode);

    // will check if an obstacle is within the vicinity of the new node and any of
    // its neighbors; will return an array of safe nodes
    ConfigspaceNode* checkSafety(ConfigspaceNode newNode, ConfigspaceNode* neighbors);

    // checks if an obstacle (defined by position and radius) lies within the current freespace
    bool obstacleInFreespace(double xObs, double yObs, double radiusObs);

    // adds an obstacle to the freespace given the obstacle position and radius
    void addObstacle(double xObs, double yObs, double radiusObs);

    // read in obstacles from a specified file
    bool readObstaclesFromFile(const char* obstacleFile);

    // compute the volume of the obstacles defined in the graph
    double computeObsVol();

    // checks if the vehicle goal region intersects the gate
    bool atGate(ConfigspaceNode node);

    // default constructor
    WorkspaceGraph() { _buildWorkspaceGraph(); }
};

#endif //WORKSPACE_H