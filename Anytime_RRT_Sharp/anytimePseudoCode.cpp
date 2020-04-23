#include <iostream>
#include <time.h> 
#include <math.h>
#include <cmath>
#include <typeinfo>

#include "configspaceGraph.h"
#include "workspaceGraph.h"

using namespace std;

// computes the gate (root) node based on the approximate gate location
// and orientation; outputs a ConfigspaceNode to add to the graph
ConfigspaceNode calcGateNode(double xPosition, double yPosition, double gateOrientation, double standOffRange)
{
	ConfigspaceNode gateNode;

	gateNode.x = xPosition + standOffRange * cos(gateOrientation - M_PI);
	gateNode.y = yPosition + standOffRange * sin(gateOrientation - M_PI);
	gateNode.theta = gateOrientation;

	gateNode.v = 0.0; gateNode.w = 0.0; gateNode.t = 0.0;
	gateNode.a = 0.0; gateNode.gamma = 0.0; gateNode.cost = 0.0;
	gateNode.parentNodeId = 0;

	return gateNode;
}

int main()
{

	#pragma region Global Initialization 
	//------------------------------------------------------------------------//
	//-------this info should be ingested from the intialization script-------//
	//------------------------------------------------------------------------//

	// define arrays for the gate and obstacle information
	double approxGateXPosition = 5.0;
	double approxGateYPosition = 60.0;
	double approxGateApproach = M_PI / 2.0;

	double obstacleXPosition[] = { 80, 73, 63, 53, 43, 33, 28, 25, 25, 25, 25, 35, 40, 45, 80, 85, 90, 95, 100, 100, 100, 100, 60 };
	double obstacleYPosition[] = { 40, 30, 25, 25, 26, 25, 35, 47, 57, 67, 77, 80, 80, 80, 80, 80, 80, 80, 80, 0, 5, 10, 100 };
	double obstacleRadius[]    = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8 };
	int numObstacles = sizeof(obstacleRadius) / sizeof(double);

	// define additional input parameters for the goal node calculation
	double standOffRange = 5.0;

	// initial UAV orientation
	double uavStartX = 100.0, uavStartY = 60.0, uavStartTheta = 0.0;
	double uavStartV = 0.0, uavStartW = 0.0;

	// goal region (UAV position) radius
	double uavGoalRadius = 2.5;

	// vehicle rigid body information
	int numVehiclePoints = 4;
	double vehiclePointXPosition[numVehiclePoints] = { -0.5, 0.5, 0.5, -0.5 };
	double vehiclePointYPosition[numVehiclePoints] = { -0.5, -0.5, 0.5, 0.5 };

	// initialize variables for the graph limits
	double xMin = 0.0, yMin = 0.0, xMax = 0.0, yMax = 0.0;
	double thetaMin = 0.0, thetaMax = 2 * M_PI;
	double vMin = 0.0, vMax = 1.0, wMin = -M_PI / 8, wMax = M_PI / 8;
	double linAccelMax = 1.0, rotAccelMax = M_PI / 4;

	// define a buffer region and the side length to use when
	// defining the square freespace
	double buffer = 40.0;

	// seed for random node generation
	srand (time(NULL));
	//------------------------------------------------------------------------//
	//------------------------------------------------------------------------//

	//------------------------------------------------------------------------//
	//------------------this info must be declared regardless-----------------//
	//------------------------------------------------------------------------//
	double obsVol = 0.0;
	int dimension = 2;
	int goalBiasCount = 100;
	int maxCount = 5000;
	int tempItr = 0;

	ConfigspaceNode gateNode, tempNode, parentNode, newNode, bestNeighbor, remainingNodeParent;
	ConfigspaceNode *nearestNeighbors = NULL, *safeNearestNeighbors = NULL, *remainingNodes = NULL, *removeNodes = NULL, *lastNodes = NULL, *costThresholdNodes = NULL;
	bool goalCheck;
	int remainingCount = 0, k = 10, m = 40, count = 0;
	double circleRadius = 0.0, epsilon = 5.0;
	//------------------------------------------------------------------------//
	//------------------------------------------------------------------------//
	#pragma endregion Initializes all necessary variables (should be read-in for real function)

	#pragma region Primary Function
	// build workspace and configspace graphs
	WorkspaceGraph G_workspace;
	ConfigspaceGraph G_configspace;

	// set the goal region (i.e. the UAV starting location)
	G_workspace.addGoalRegion(uavStartX, uavStartY, uavStartTheta, uavStartV, uavStartW, uavGoalRadius);

	// function to determine goal node based on approximate gate information
	gateNode = calcGateNode(approxGateXPosition, approxGateYPosition, approxGateApproach, standOffRange);

	// define the limits of the graph based on position of the gate
	// and the robot
	if (gateNode.x < G_workspace.goalRegion.x) { xMin = gateNode.x - buffer; xMax = G_workspace.goalRegion.x + buffer; }
	else { xMin = G_workspace.goalRegion.x - buffer; xMax = gateNode.x + buffer; }

	if (gateNode.y < G_workspace.goalRegion.y) { yMin = gateNode.y - buffer; yMax = G_workspace.goalRegion.y + buffer; }
	else { yMin = G_workspace.goalRegion.y - buffer; yMax = gateNode.y + buffer; }

	// set freespace of graphs based on the graph limits; some values will always
	// be the same (theta, v, w, a, gamma), while others will vary (x and y)
	G_workspace.defineFreespace(
		xMin, yMin, thetaMin, vMin, wMin,
		xMax, yMax, thetaMax, vMax, wMax,
		linAccelMax, rotAccelMax);
	G_configspace.defineFreespace(
		xMin, yMin, thetaMin, vMin, wMin,
		xMax, yMax, thetaMax, vMax, wMax,
		linAccelMax, rotAccelMax,
		dimension, obsVol);

	// set the obstacles for this iteration (we don't need to consider all obstacles
	// for every iteration); use the above defined freespace limits
	for (int i = 0; i < numObstacles; i++)
		if (G_workspace.obstacleInFreespace(obstacleXPosition[i], obstacleYPosition[i], obstacleRadius[i]))
			G_workspace.addObstacle(obstacleXPosition[i], obstacleYPosition[i], obstacleRadius[i]);

	// compute the volume of the obstacles
	obsVol = G_workspace.computeObsVol();

	// add gateNode to the graph
	G_configspace.addNode(gateNode);

	// add vehicle to the graph
	G_workspace.addVehicle(vehiclePointXPosition, vehiclePointYPosition, numVehiclePoints);

	printf("ObsVol: %f, NumObs: %d, NumVehicles: %d, Freespace: [%f, %f, %f, %f]\n", obsVol, G_workspace.numObstacles, G_workspace.numVehicles, xMin, xMax, yMin, yMax);
	printf("GoalRegion: %f, %f, %f\n", G_workspace.goalRegion.x, G_workspace.goalRegion.y, G_workspace.goalRegion.radius);
	printf("GateNode: %f, %f, %f\n", G_configspace.nodes[0].x, G_configspace.nodes[0].y, G_configspace.nodes[0].theta);
	///////////////////////////////////////////////////////////////////////////////////////////

	// start the anytime RRT# iterations

	free(safeNearestNeighbors);
	free(costThresholdNodes);
	free(nearestNeighbors);
	free(remainingNodes);
	free(removeNodes);
	free(lastNodes);

	safeNearestNeighbors = NULL;
	costThresholdNodes   = NULL;
	nearestNeighbors     = NULL;
	remainingNodes       = NULL;
	removeNodes          = NULL;
	lastNodes            = NULL;

	// do the RRT# thing
	while(!G_workspace.goalRegionReached || count < maxCount)
	{				
		tempNode = (count % goalBiasCount != 0) ? G_configspace.generateRandomNode() : G_configspace.generateBiasedNode(G_workspace.goalRegion.x, G_workspace.goalRegion.y);
		parentNode = G_configspace.findClosestNode(tempNode);
		free(remainingNodes);
		remainingNodes = NULL;
		if (!G_workspace.checkAtGoal(parentNode))
		{
			newNode = G_workspace.extendToNode(parentNode, tempNode, epsilon);
			newNode.cost = parentNode.cost + G_configspace.computeCost(parentNode, newNode);

			// if there is a collision, newNode id will be set to its parent's id
			if (newNode.id != parentNode.id)
			{
				circleRadius = G_configspace.computeRadius(epsilon);
				safeNearestNeighbors = G_configspace.findNeighbors(newNode, circleRadius, k);
				remainingNodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
				remainingNodes[0].id = 0;

				if (safeNearestNeighbors[0].id)
				{
					bestNeighbor = G_configspace.findBestNeighbor(newNode, safeNearestNeighbors);
					tempNode = G_workspace.connectNodes(bestNeighbor, newNode);
					tempNode.cost = bestNeighbor.cost + G_configspace.computeCost(bestNeighbor, tempNode);

					if (tempNode.cost < newNode.cost)
					{
						newNode = tempNode;
						parentNode = bestNeighbor;
					}

					free(remainingNodes);
					remainingNodes = G_configspace.removeNode(safeNearestNeighbors, bestNeighbor);
				}
				tempNode = G_configspace.addNode(newNode);
				G_configspace.addEdge(parentNode, tempNode);

				if (!G_workspace.goalRegionReached)
					if (G_workspace.checkAtGoal(tempNode))
						G_workspace.goalRegionReached = true;

				// start rewiring
				remainingCount = 0;
				while (remainingNodes[remainingCount].id)
				{
					if (remainingNodes[remainingCount].cost > (tempNode.cost + G_configspace.computeCost(remainingNodes[remainingCount], tempNode)))
					{
						newNode = G_workspace.connectNodes(tempNode, remainingNodes[remainingCount]);
						newNode.cost = tempNode.cost + G_configspace.computeCost(remainingNodes[remainingCount], tempNode);
						newNode.parentNodeId = tempNode.id;
						remainingNodeParent = G_configspace.findNodeId(remainingNodes[remainingCount].parentNodeId);
						G_configspace.removeEdge(remainingNodeParent, remainingNodes[remainingCount]);
						G_configspace.addEdge(tempNode, newNode);
						G_configspace.replaceNode(remainingNodes[remainingCount], newNode);

						ConfigspaceNode* updatedNode = (ConfigspaceNode*)calloc(2, sizeof(ConfigspaceNode));
						updatedNode[0] = newNode;
						updatedNode[1].id = 0;
						G_configspace.propagateCost(updatedNode);
					}
					++remainingCount;
				}
			}
		}
		++count;
	}

	double tempCost = 0, finalCost = 100000;
	ConfigspaceNode finalNode;

	for (int i = 0; i < G_configspace.numNodes; i++)
	{
		if (G_workspace.checkAtGoal(G_configspace.nodes[i]))
		{
			tempCost = G_configspace.nodes[i].cost;
			if (tempCost < finalCost)
			{
				finalCost = tempCost;
				finalNode = G_configspace.nodes[i];
			}
		}
	}
	printf("Total number of points: %d\n", G_configspace.numNodes);
	printf("Final node at: (%f, %f)\n", finalNode.x, finalNode.y);
	printf("Final cost is: %f\n", finalCost);
	G_configspace.printData(1  + tempItr++, finalNode);

	// get the last m nodes in the tree
	lastNodes = G_configspace.getLastNodes(finalNode, m);
	#pragma endregion Primary code for the Anytime RRT# implementaion

	return 0;

}

