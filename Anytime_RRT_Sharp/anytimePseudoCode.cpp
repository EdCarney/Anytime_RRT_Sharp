#include <iostream>
#include <time.h> 
#include <math.h>
#include <cmath>

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
	gateNode.theta = gateOrientation - M_PI;

	gateNode.v = 0.0; gateNode.w = 0.0; gateNode.t = 0.0;
	gateNode.dx = 0.0; gateNode.dy = 0.0;
	gateNode.ddx = 0.0; gateNode.ddy = 0.0;
	gateNode.a = 0.0; gateNode.gamma = 0.0;
	gateNode.cost = 0.0; gateNode.dist = 0.0;
	gateNode.parentNodeId = 0;

	return gateNode;
}

int main()
{

#pragma region Global Initialization 
	//------------------------------------------------------------------------//
	//-------this info should be ingested from the intialization script-------//
	//------------------------------------------------------------------------//
	int numGates = 1, numObstacles = 23;

	// define arrays for the gate and obstacle information (INCLUDE GATE REGION AS OBSTACLE)
	double approxGateXPosition[numGates] = { 5.0 };
	double approxGateYPosition[numGates] = { 60.0 };
	double approxGateApproach[numGates] = { 3 * M_PI / 2 };

	//double exactGateXPosition[numGates] = { 4.5 };
	//double exactGateYPosition[numGates] = { 59.7 };
	//double exactGateApproach[numGates] = { 0.95 * (3 * M_PI / 2) };

	// define additional input parameters for the goal node calculation
	double standOffRange = 5.0;

	double obstacleXPosition[numObstacles + numGates] = { 80, 73, 63, 53, 43, 33, 28, 25, 25, 25, 25, 35, 40, 45, 80, 85, 90, 95, 100, 100, 100, 100, 60, approxGateXPosition[0] };
	double obstacleYPosition[numObstacles + numGates] = { 40, 30, 25, 25, 26, 25, 35, 47, 57, 67, 77, 80, 80, 80, 80, 80, 80, 80, 80, 0, 5, 10, 100, approxGateYPosition[0] };
	double obstacleRadius[numObstacles + numGates] = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, standOffRange - 1.0 };

	// initial UAV orientation
	//double uavStartX = 100.0, uavStartY = 60.0, uavStartTheta = 0.75 * M_PI;
	//double uavStartX = 100.0, uavStartY = 20.0, uavStartTheta = 0.75 * M_PI;
	double uavStartX = 120.0, uavStartY = 80.0, uavStartTheta = 1.75 * M_PI;
	//double uavStartX = 50.0, uavStartY = 50.0, uavStartTheta = 0.25 * M_PI;
	double uavStartV = 0.0, uavStartW = 0.0;

	// goal region (UAV position) radius
	double uavGoalRadius = 2.5;

	// vehicle rigid body information
	int numVehiclePoints = 4;
	double vehiclePointXPosition[numVehiclePoints] = { -0.5, 0.5, 0.5, -0.5 };
	double vehiclePointYPosition[numVehiclePoints] = { -0.5, -0.5, 0.5, 0.5 };

	// initialize variables for the graph limits (which will
	// change with each gate iteration)
	double xMin = 0.0, yMin = 0.0, xMax = 0.0, yMax = 0.0;
	double thetaMin = 0.0, thetaMax = 2 * M_PI;
	double vMin = 0.0, vMax = 2.0, wMin = -M_PI / 8, wMax = M_PI / 8;
	double linAccelMax = 0.5, rotAccelMax = M_PI / 4;

	// define a buffer region and the side length to use when
	// defining the square freespace
	double buffer = 40.0;

	// define the timestep and delta time (dt) values used for connecting and rewiring nodes
	double timestep = 5.0, dt = 0.1;
	//------------------------------------------------------------------------//
	//------------------------------------------------------------------------//

	//------------------------------------------------------------------------//
	//------------------this info must be declared regardless-----------------//
	//------------------------------------------------------------------------//
	ConfigspaceNode gateNode;
	double obsVol = 0.0;
	int dimension = 2;

	double iterationRuntime = 0.0, maxIterationRuntime = 0.0;

	int goalBiasCount = 100;

	ConfigspaceNode tempNode, parentNode, newNode, bestNeighbor, remainingNodeParent;
	ConfigspaceNode *nearestNeighbors = NULL, *safeNearestNeighbors = NULL, *remainingNodes = NULL, *removeNodes = NULL, *lastNodes = NULL, *costThresholdNodes = NULL;
	bool goalCheck;
	int remainingCount = 0, k = 15, m = 4, count = 0;
	double circleRadius = 0.0, epsilon = 20.0;
	double *branchBounds = (double*)calloc(4, sizeof(double));
	//------------------------------------------------------------------------//
	//------------------------------------------------------------------------//
#pragma endregion Initializes all necessary variables (should be read-in for real function)

#pragma region Primary Function Definition
	for (int gate = 0; gate < numGates; gate++)
	{
		// build workspace and configspace graphs
		WorkspaceGraph G_workspace;
		ConfigspaceGraph G_configspace;

		if (gate == 0)
		{
			// set the goal region (i.e. the UAV starting location)
			// if it's not the first iteration, then the goal region will be set based
			// on the last known position of the UAV
			G_workspace.addGoalRegion(uavStartX, uavStartY, uavStartTheta - M_PI, uavStartV, uavStartW, uavGoalRadius);
		}
		/*
		else
		{
			// add the goal region based on the exit node of the
			// last run
			G_workspace.addGoalRegion(<PREVIOUS_START_NODE>);
		}
		*/

		// function to determine goal node based on approximate gate information
		gateNode = calcGateNode(approxGateXPosition[gate], approxGateYPosition[gate], approxGateApproach[gate], standOffRange);
		gateNode.v = 2.0;
		printf("Theta: %f\n", gateNode.theta);

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
		{
			if (G_workspace.obstacleInFreespace(obstacleXPosition[i], obstacleYPosition[i], obstacleRadius[i]))
			{
				G_workspace.addObstacle(obstacleXPosition[i], obstacleYPosition[i], obstacleRadius[i]);
			}
		}

		// compute the volume of the obstacles
		obsVol = G_workspace.computeObsVol();

		// add gateNode to the graph
		G_configspace.addNode_basic(gateNode);

		// add vehicle to the graph
		G_workspace.addVehicle(vehiclePointXPosition, vehiclePointYPosition, numVehiclePoints);

		printf("ObsVol: %f, NumObs: %d, NumVehicles: %d, Freespace: [%f, %f, %f, %f]\n", obsVol, G_workspace.numObstacles, G_workspace.numVehicles, xMin, xMax, yMin, yMax);
		printf("GoalRegion: %f, %f, %f\n", G_workspace.goalRegion.x, G_workspace.goalRegion.y, G_workspace.goalRegion.radius);
		printf("GateNode: %f, %f, %f\n", G_configspace.nodes[0].x, G_configspace.nodes[0].y, G_configspace.nodes[0].theta);
		///////////////////////////////////////////////////////////////////////////////////////////
#pragma endregion Define graph freespace, goal region, and root node.

#pragma region RRT# Main Code
		// start the anytime RRT# iterations
		iterationRuntime = 0.0;
		maxIterationRuntime = 7500.0;
		count = 0;
		int tempItr = 0;
		while(!G_workspace.atGate(gateNode))
		{
			iterationRuntime = 0.0;
			free(nearestNeighbors); free(safeNearestNeighbors); free(remainingNodes);
			free(removeNodes); free(lastNodes); free(costThresholdNodes);
			nearestNeighbors = NULL; safeNearestNeighbors = NULL; remainingNodes = NULL;
			removeNodes = NULL; lastNodes = NULL; costThresholdNodes = NULL;

			// do the RRT# thing
			while(iterationRuntime < maxIterationRuntime || !G_workspace.goalRegionReached)
			{				
				free(remainingNodes);
				remainingNodes = NULL;

				tempNode = (count % goalBiasCount != 0) ? G_configspace.generateRandomNode() : G_configspace.generateBiasedNode(G_workspace.goalRegion.x, G_workspace.goalRegion.y, G_workspace.goalRegion.theta);
				parentNode = G_configspace.findClosestNode(tempNode);
				
				if (!G_workspace.checkAtGoal_basic(parentNode))
				{
					newNode = G_workspace.extendToNode_basic(parentNode, tempNode, epsilon);

					// CUBIC BEZIER PATH
					if (newNode.parentNodeId)
					{
						newNode.v = tempNode.v;
						newNode.theta = tempNode.theta;
						newNode = G_workspace.connectNodesCubicBezier(parentNode, newNode, timestep, dt);
					}

					// if there is a collision, newNode parent id will be zero
					if (newNode.parentNodeId)
					{
						newNode.cost = parentNode.cost + G_configspace.computeCost_basic(parentNode, newNode);

						circleRadius = G_configspace.computeRadius(epsilon);
						safeNearestNeighbors = G_configspace.findNeighbors(newNode, circleRadius, k);
						remainingNodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
						remainingNodes[0].id = 0;

						if (safeNearestNeighbors[0].id)
						{
							bestNeighbor = G_workspace.findBestNeighbor_basic(newNode, safeNearestNeighbors, timestep, dt);
							tempNode = G_workspace.connectNodes_basic(bestNeighbor, newNode);

							// CUBIC BEZIER PATH
							if (tempNode.parentNodeId)
							{
								tempNode.v = newNode.v;
								tempNode.theta = newNode.theta;
								tempNode = G_workspace.connectNodesCubicBezier(bestNeighbor, tempNode, timestep, dt);
							}

							if (tempNode.parentNodeId)
							{
								tempNode.cost = bestNeighbor.cost + G_configspace.computeCost_basic(bestNeighbor, tempNode);

								if (tempNode.cost < newNode.cost)
								{
									newNode = tempNode;
									parentNode = bestNeighbor;
								}
							}
							free(remainingNodes);
							remainingNodes = G_configspace.removeNode(safeNearestNeighbors, bestNeighbor);
						}
						tempNode = G_configspace.addNode_basic(newNode);
						G_configspace.addEdge(parentNode, tempNode);

						if (!G_workspace.goalRegionReached)
						{
							if (G_workspace.checkAtGoal_basic(tempNode))
							{
								G_workspace.goalRegionReached = true;
							}
						}

						// start rewiring
						remainingCount = 0;
						while (remainingNodes[remainingCount].id)
						{
							//newNode.parentNodeId = tempNode.id;
							newNode = G_workspace.connectNodes_basic(tempNode, remainingNodes[remainingCount]);
							
							// CUBIC BEZIER PATH
							if (newNode.parentNodeId)
							{
								newNode.v = remainingNodes[remainingCount].v;
								newNode.theta = remainingNodes[remainingCount].theta;
								newNode = G_workspace.connectNodesCubicBezier(tempNode, newNode, timestep, dt);
							}

							//printf("RemainingNodeCost: %f, PotentialRewireCost: %f\n", remainingNodes[remainingCount].cost, tempNode.cost + G_configspace.computeCost_basic(tempNode, newNode));
							
							if (newNode.parentNodeId && (remainingNodes[remainingCount].cost > (tempNode.cost + G_configspace.computeCost_basic(tempNode, newNode))))
							{
								remainingNodeParent = G_configspace.findNodeId(remainingNodes[remainingCount].parentNodeId);
								newNode.cost = tempNode.cost + G_configspace.computeCost_basic(tempNode, newNode);
								//printf("----------------------------DEBUG3 - START; Cost: %f----------------------------\n", newNode.cost);
								G_configspace.removeEdge(remainingNodeParent, remainingNodes[remainingCount]);
								G_configspace.addEdge(tempNode, newNode);
								G_configspace.replaceNode(remainingNodes[remainingCount], newNode);

								ConfigspaceNode* updatedNode = (ConfigspaceNode*)calloc(2, sizeof(ConfigspaceNode));
								updatedNode[0] = newNode;
								updatedNode[1].id = 0;
								G_configspace.propagateCost_basic(updatedNode);
								//printf("----------------------------DEBUG3 - STOP; Cost: %f-----------------------------\n", newNode.cost);
							}
							remainingCount++;
						}
					}
				}
				count++;
				iterationRuntime += 1.0;
			}

			double tempCost = 0, finalCost = 100000;
			ConfigspaceNode finalNode;

			for (int i = 0; i < G_configspace.numNodes; i++)
			{
				if (G_workspace.checkAtGoal_basic(G_configspace.nodes[i]))
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
			G_configspace.printData(1  + tempItr++, finalNode);

			// get the last m nodes in the tree
			lastNodes = G_configspace.getLastNodes(finalNode, m);

			// get a list of nodes across which the cost exceeds the current
			// final node's cost
			costThresholdNodes = G_configspace.getCostThresholdNodes(lastNodes[0]);

			// trim these threshold nodes and all children
			G_configspace.trimTreeChildren(costThresholdNodes, lastNodes[0].id);

			// update goal region in the workspace graph to the new
			// current last node in the tree (the (n-m)th node)
			G_workspace.updateGoalRegion(lastNodes[0].x, lastNodes[0].y, lastNodes[0].theta, 0.0, 0.0, 2.5);

			// update the freespace of the graph to limit x, y search to within the current branch
			branchBounds = G_configspace.getBranchBounds(lastNodes[0]);
			
			G_workspace.defineFreespace(
				branchBounds[0], branchBounds[2], thetaMin, vMin, wMin,
				branchBounds[1], branchBounds[3], thetaMax, vMax, wMax,
				linAccelMax, rotAccelMax);
			G_configspace.defineFreespace(
				branchBounds[0], branchBounds[2], thetaMin, vMin, wMin,
				branchBounds[1], branchBounds[3], thetaMax, vMax, wMax,
				linAccelMax, rotAccelMax,
				dimension, obsVol);
			

		}
	}
#pragma endregion Primary code for the Anytime RRT# implementaion

return 0;

}

