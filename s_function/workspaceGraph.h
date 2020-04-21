/*
This graph data structure is heavily based on the graph data structure provided
by Dr. Michael Otte.
*/

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "configspaceGraph.h"

#ifndef WORKSPACE_H
#define WORKSPACE_H

struct WorkspaceNode
{
	int id;		// unique identifer for the node; 1-indexed

	double x;	// x coordinate location
	double y;	// y coordinate location

	int parentNodeId;
};

// for the RRT algorithm obstacles will be defined
// as circles with a given position and radius
struct Obstacle
{
	double x;		// x coordinate of obstacle center
	double y;		// y coordinate of obstacle center
	double radius;	// radius of disk obstacle

};

struct GoalRegion
{
	double x;
	double y;
	double theta;
	double v;
	double w;
	double radius;
};

// a struct to define the shape of the vehicle for
// collision checking for RRT for  rigid body of n nodes
struct Vehicle
{
	int numNodes;					// number of representing the vehicle

	WorkspaceNode* nodes;			// an array of all nodes for the vehicle
	WorkspaceNode* offsetNodes;		// an array of offset valus loaded on vehicle initialization
	WorkspaceNode centroid;			// node defining the centeriod of the node points

	double theta;					// rotation angle of the rigid body
	double maxPointRadius;			// the radius of a ball circumscribing the vehicle; used for quick collision checking

private:
	void buildVehicle();

	// calculates the centroid of the vehicle (as a node)
	// using the list of nodes for the point
	void calculateCentroid();

public:
	// default constructor
	Vehicle() { buildVehicle(); }

	// update the vehicle state based on config node
	void updateState(ConfigspaceNode newConfigNode);
};


class WorkspaceGraph
{
public:

	int numObstacles;		// total number of obstacles in the graph
	int numVehicles;		// total number of vehicles in the graph

	Obstacle* obstacles;	// an array containing all obstacles
	Vehicle* vehicles;		// an array containing all vehicles
	GoalRegion goalRegion;	// the goal region for the graph (treated as an obstacle)

	double minX, minY, maxX, maxY;	// limits of the graph freespace
	double minTheta, maxTheta;		// limits of the oreintation theta
	double minV, minW, maxV, maxW;	// limits of the angular and translational velocity
	double maxAbsA, maxAbsGamma;	// absolute limits of the angular and translational acceleration

	bool goalRegionReached;	// boolean value to indicate if the goal region has been reached

private:
	void buildWorkspaceGraph();
	void deleteWorkspaceGraph();

public:

	// sets the goal region for the graph
	// this is treated similar to an obstacle
	void addGoalRegion(double x, double y, double theta, double v, double w, double radius);

	// updates the position and configuration of the graph goal region
	void updateGoalRegion(double x, double y, double theta, double v, double w, double radius);

	// defines freespace for problem
	// used when extending to a new node
	void defineFreespace(double minX, double minY, double minTheta, double minV, double minW,
		double maxX, double maxY, double maxTheta, double maxV, double maxW, double newMaxAbsA, double newMaxAbsGamma);

	// checks if specified node is in the goal region
	// returns true if it is and false otherwise
	bool checkAtGoal(ConfigspaceNode node);

	// checks if any vehicle collides with any obstacle at the provided configuration point
	// returns true if there is a collision and false otherwise
	bool checkCollision(ConfigspaceNode node);

	// attempts to buid a path from a parent configuratin node to a new configuration node
	// uses interpolation with a step size of delta and a maximum extension of epsilon
	ConfigspaceNode extendToNode(ConfigspaceNode parentNode, ConfigspaceNode newNode, double delta, double epsilon);

	// attempt to connect two nodes; used for rewiring the graph for RRT*
	// will attempt to go from parent node to new node until the difference is less then some epsilon
	// returns zero if successful and one otherwise
	ConfigspaceNode connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode, int numIterations);

	// will check if an obstacle is within the vicinity of the new node and any of
	// its neighbors; will return an array of safe nodes
	ConfigspaceNode* checkSafety(ConfigspaceNode newNode, ConfigspaceNode* neighbors);

	// checks if an obsatcle (defined by position and radius) lies within the current freespace
	bool obstacleInFreespace(double xObs, double yObs, double radiusObs);

	// adds an obstacle to the freespace given the obstacle position and radius
	void addObstacle(double xObs, double yObs, double radiusObs);

	// adds a vehicle to the graph (currently hard-coded to four points)
	void addVehicle(double vehiclePointXPosition[4], double vehiclePointYPosition[4], int numVehiclePoints);

	// read in obstacles from a specified file
	bool readObstaclesFromFile(const char* obstacleFile);

	// read in vehicle from a specified file
	bool readVehicleFromFile(const char* vehicleFile);

	// compute the volume of the obstacles defined in the graph
	double computeObsVol();

	// checks if the vehicle goal region intersects the gate
	bool atGate(ConfigspaceNode node);

	// default constructor
	WorkspaceGraph() { buildWorkspaceGraph(); }

	// default destructor
	~WorkspaceGraph() { deleteWorkspaceGraph(); }

	///////////////////////////////////////////////////////////////////

	ConfigspaceNode extendToNode_basic(ConfigspaceNode parentNode, ConfigspaceNode newNode, double epsilon);
	bool checkForCollision_basic(ConfigspaceNode node);
	ConfigspaceNode connectNodes_basic(ConfigspaceNode parentNode, ConfigspaceNode newNode);
	bool checkAtGoal_basic(ConfigspaceNode node);
};

#endif // WORKSPACE_H