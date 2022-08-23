#include "configspaceGraph.hpp"
#include "Geometry.hpp"
#include "Utility.hpp"

#ifndef WORKSPACE_H
#define WORKSPACE_H

struct GoalRegion
{
	double x;
	double y;
	double theta;
	double radius;
};

// a struct to define the shape of the vehicle for
// collision checking for RRT for rigid body of n nodes
struct Vehicle
{
	int numNodes;					// number of representing the vehicle

	Node* nodes;				// an array of all nodes for the vehicle
	Node* offsetNodes;			// an array of offset values loaded on vehicle initialization
	Node centroid;				// node defining the centeroid of the node points

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
	void updateState(Position position);
};


class WorkspaceGraph
{
public:

	int numObstacles;				// total number of obstacles in the graph

	Obstacle* obstacles;			// an array containing all obstacles
	Vehicle vehicle;				// an array containing all vehicles
	GoalRegion goalRegion;			// the goal region for the graph (treated as an obstacle)

	double minX, minY, maxX, maxY;	// limits of the graph freespace
	double minTheta, maxTheta;		// limits of the orientation theta

	bool goalRegionReached;			// boolean value to indicate if the goal region has been reached

private:
	void buildWorkspaceGraph();
	void deleteWorkspaceGraph();

public:

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

	// adds a vehicle to the graph (currently hard-coded to four points)
	void setVehicle(double vehiclePointXPosition[4], double vehiclePointYPosition[4], int numVehiclePoints);

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
};

#endif // WORKSPACE_H