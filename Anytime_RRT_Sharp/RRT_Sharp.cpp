// RRT.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <time.h> 
#include <math.h>
#include <cmath>

#include "configspaceGraph.h"
#include "workspaceGraph.h"

using namespace std;

int main()
{

#pragma region Initialize

	srand(time(NULL));

	WorkspaceGraph G_workspace;
	ConfigspaceGraph G_configspace;

	///////////////////////////////////////////////////////////////////////////////////////////////
	// USER DEFINED PARAMETERS
	int problemNum = 2;							// problem number to auto-define parameters and output files
	int goalBiasCount = 1000;					// number of iterations before biased goal region node
	int maxCount = 5000;						// max count before reporting failure
	int k = 10;									// number of nearest neighbors
	///////////////////////////////////////////////////////////////////////////////////////////////
	int dimension = 2;							// the dimension of the freespace
	double obsVol = 0.0;						// the obstacle volume initialized to zero
	double PI = 3.14159265358979323846;			// value of pi because it's easier this way
	double delta = 0.5;							// delta value to use to check position of vehicle in workspace
	double startX[5] = { 80, 5, 10, 63, 100 };	// start x-position of the robot centroid
	double startY[5] = { 60, 60, 90, 80, 100 };	// start y-position of the robot centroid
	double startV[5] = { 1, 1, 1, 1, 1 };		// goal velocity for the robot
	double startW[5] = { 0, 0, 0, 0, 0 };		// goal velocity for the robot
	double epsilon[5] = { 5, 5, 5, 5, 5 };		// epsilon extend values for each iteration
	double startTheta[5] =						// start orientation angle of the robot
	{ PI * 2 / 3, PI * 1 / 4, 3 / 2 * PI, PI / 2,  PI };
	double goalRegionX[5] =						// goal region x-position
	{ 10, 100, 100, 60, 25 };
	double goalRegionY[5] =						// goal region y-position
	{ 10, 60, 90, 10, 10 };
	double goalRegionRad[5] =					// goal region radius
	{ 5, 5, 5, 5, 5 };
	double linAccel = 2;						// constant translational acceleration input
	double rotAccel = PI / 4;					// constant rotational acceleration input
	///////////////////////////////////////////////////////////////////////////////////////////////

	// read in obstacles to workspace graph
	if (!G_workspace.readObstaclesFromFile("obstacles.txt")) { printf("Error reading in obstacles.\n"); }

	// read in vehicle to workspace graph
	if (!G_workspace.readVehicleFromFile("robot.txt")) { printf("Error reading in vehicle.\n"); }

	// compute the volume of the obstacles
	obsVol = G_workspace.computeObsVol();

	// update vehicle position in workspace
	ConfigspaceNode initialVehiclePos;
	initialVehiclePos.x = startX[problemNum - 1];
	initialVehiclePos.y = startY[problemNum - 1];
	initialVehiclePos.theta = startTheta[problemNum - 1];
	G_workspace.vehicles[0].updateState(initialVehiclePos);

	// define goal region in workspace graph
	G_workspace.addGoalRegion(goalRegionX[problemNum - 1], goalRegionY[problemNum - 1], 0.0, 0.0, 0.0, goalRegionRad[problemNum - 1]);

	// define freespace in workspace graph
	G_workspace.defineFreespace(
		0, 0, 0, 0, -PI / 2,
		125, 125, 2 * PI, 2, PI / 2,
		linAccel, rotAccel
	);

	// define freespace in config graph
	G_configspace.defineFreespace(
		0, 0, 0, 0, -PI / 2,
		125, 125, 2 * PI, 2, PI / 2,
		linAccel, rotAccel, dimension, obsVol
	);

	// create start config in config gaph
	G_configspace.createNode(startX[problemNum - 1], startY[problemNum - 1], startTheta[problemNum - 1], startV[problemNum - 1], startW[problemNum - 1], 0);

	// initial count for notifications and biasing
	int count = 1;

#pragma endregion Initializes all upfront variables

#pragma region MainCode

	ConfigspaceNode tempNode, parentNode, testNewNode, newNode, bestNeighbor, remainingNodeParent;
	ConfigspaceNode * nearestNeighbors, *safeNearestNeighbors, *remainingNodes;
	bool goalCheck;
	int remainingCount = 0;
	double cost = 0.0;
	double circleRadius = 0.0;

	// perform main algorithm runs
	// will sample in the config space
	// will do goal check and collision check in the workspace
	do
	{
		if (!(count % 1000))
		{
			double tempCost = 0, finalCost = 100000;
			ConfigspaceNode finalNode = testNewNode;
			for (int i = 0; i < G_configspace.numNodes; i++)
			{
				//if (G_workspace.checkAtGoal(G_configspace.nodes[i]))
				if (G_workspace.checkAtGoal_basic(G_configspace.nodes[i]))
				{
					//printf("Goal reached! Cost of %f\n", G_configspace.nodes[i].cost);
					tempCost = G_configspace.nodes[i].cost;
					if (tempCost < finalCost)
					{
						//printf("This is the final goal now!\n");
						finalCost = tempCost;
						finalNode = G_configspace.nodes[i];
					}
				}
			}

			//printf("Count: %d\n", count);
			//printf("Total number of points: %d\n", G_configspace.numNodes);
			//printf("Total path cost: %f\n", finalCost);
			G_configspace.printData(problemNum + count, finalNode);

			tempCost = 0, finalCost = 100000;
			finalNode = testNewNode;

		}

		tempNode = (count % goalBiasCount != 0) ? G_configspace.generateRandomNode() : G_configspace.generateBiasedNode(G_workspace.goalRegion.x, G_workspace.goalRegion.y);
		//parentNode = G_configspace.findClosestNode(tempNode);
		parentNode = G_configspace.findClosestNode_basic(tempNode); // FOR BASIC RRT STAR

		//if (!G_workspace.checkAtGoal(parentNode))
		if (!G_workspace.checkAtGoal_basic(parentNode))
		{
			//testNewNode = G_workspace.extendToNode(parentNode, tempNode, delta, epsilon[problemNum - 1]);
			//testNewNode.cost = parentNode.cost + G_configspace.computeCost(parentNode, testNewNode);
			testNewNode = G_workspace.extendToNode_basic(parentNode, tempNode, epsilon[problemNum - 1]);
			testNewNode.cost = parentNode.cost + G_configspace.computeCost_basic(parentNode, testNewNode);

			// if the node was able to be added without collision, add 
			// it to the graph and build the edge
			if (testNewNode.id != parentNode.id)
			{
				newNode = testNewNode;

				// do the RRT* stuff
				circleRadius = G_configspace.computeRadius(epsilon[problemNum - 1]);
				//nearestNeighbors = G_configspace.findNeighbors(newNode, circleRadius, k, G_workspace.goalRegion.x, G_workspace.goalRegion.y, G_workspace.goalRegion.radius);
				//safeNearestNeighbors = nearestNeighbors;//G_workspace.checkSafety(newNode, nearestNeighbors);
				safeNearestNeighbors = G_configspace.findNeighbors_basic(newNode, circleRadius, k);


				remainingNodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
				remainingNodes[0].id = NULL;

				// only continue with RRT* optimization if there are safe
				// neighbors to try to extend to (at least one)
				if (safeNearestNeighbors[0].id)
				{
					//bestNeighbor = G_configspace.findBestNeighbor(newNode, safeNearestNeighbors);
					//tempNode = G_workspace.connectNodes(bestNeighbor, newNode, 15);

					bestNeighbor = G_configspace.findBestNeighbor_basic(newNode, safeNearestNeighbors);
					tempNode = G_workspace.connectNodes_basic(bestNeighbor, newNode);

					if (tempNode.parentNodeId)
					{
						//tempNode.cost = bestNeighbor.cost + G_configspace.computeCost(bestNeighbor, tempNode);
						tempNode.cost = bestNeighbor.cost + G_configspace.computeCost_basic(bestNeighbor, tempNode);
						if (tempNode.cost < newNode.cost)
						{
							newNode = tempNode;
							parentNode = bestNeighbor;
							//printf("\nAble to extend to a different node!\n");
						}
					}

					free(remainingNodes);
					remainingNodes = G_configspace.removeNode(safeNearestNeighbors, bestNeighbor);
				}

				//tempNode = G_configspace.addNode(newNode);
				tempNode = G_configspace.addNode_basic(newNode);
				G_configspace.addEdge(parentNode, tempNode);


				remainingCount = 0;
				while (remainingNodes[remainingCount].id)
				{
					//if (remainingNodes[remainingCount].cost > (tempNode.cost + G_configspace.computeCost(remainingNodes[remainingCount], tempNode)))
					if (remainingNodes[remainingCount].cost > (tempNode.cost + G_configspace.computeCost_basic(remainingNodes[remainingCount], tempNode)))
					{

						/*
						printf("\n----------------------------------------------------\n");
						printf("\n\n");
						for (int i = 0; i < remainingNodes[remainingCount].numIterationPoints; i++)
						{
							printf("%f, %f, %f, %f, %f\n",
								remainingNodes[remainingCount].iterationPoints[i].x,
								remainingNodes[remainingCount].iterationPoints[i].y,
								remainingNodes[remainingCount].iterationPoints[i].theta,
								remainingNodes[remainingCount].iterationPoints[i].v,
								remainingNodes[remainingCount].iterationPoints[i].w,
								remainingNodes[remainingCount].iterationPoints[i].t);
						}
						ConfigspaceNode testyTestNode = remainingNodes[remainingCount];
						ConfigspaceNode testyParent = G_configspace.findNodeId(testyTestNode.parentNodeId);
						*/

						//newNode = G_workspace.connectNodes(tempNode, remainingNodes[remainingCount], 15);
						newNode = G_workspace.connectNodes_basic(tempNode, remainingNodes[remainingCount]);

						/*
						printf("\n\n");
						for (int i = 0; i < newNode.numIterationPoints; i++)
						{
							printf("%f, %f, %f, %f, %f\n",
								newNode.iterationPoints[i].x,
								newNode.iterationPoints[i].y,
								newNode.iterationPoints[i].theta,
								newNode.iterationPoints[i].v,
								newNode.iterationPoints[i].w,
								newNode.iterationPoints[i].t);
						}
						printf("\n\n");
						printf("\n----------------------------------------------------\n");
						*/

						if (newNode.parentNodeId)
						{
							//newNode.cost = tempNode.cost + G_configspace.computeCost(remainingNodes[remainingCount], tempNode);
							newNode.cost = tempNode.cost + G_configspace.computeCost_basic(remainingNodes[remainingCount], tempNode);
							newNode.parentNodeId = tempNode.id;
							remainingNodeParent = G_configspace.findNodeId(remainingNodes[remainingCount].parentNodeId);
							G_configspace.removeEdge(remainingNodeParent, remainingNodes[remainingCount]);
							G_configspace.addEdge(tempNode, newNode);
							//G_configspace.replaceNode(remainingNodes[remainingCount], newNode);
							G_configspace.replaceNode_basic(remainingNodes[remainingCount], newNode);

							// propagate the cost updates to the leaves of the tree
							ConfigspaceNode* updatedNode = (ConfigspaceNode*)calloc(2, sizeof(ConfigspaceNode));
							updatedNode[0] = newNode;
							updatedNode[1].id = 0;
							//G_configspace.propagateCost(updatedNode);
							G_configspace.propagateCost_basic(updatedNode);
							//free(nodesToUpdate);

							//printf("\nAble to rewire!\n");
							//printf("Node %d at (%f,%f) previous parent %d at (%f,%f), current parent %d at (%f,%f)!\n",
							//	newNode.id, newNode.x, newNode.y, remainingNodeParent.id, remainingNodeParent.x, remainingNodeParent.y,
							//	newNode.parentNodeId, tempNode.x, tempNode.y);
						}
					}
					remainingCount++;
				}


				free(remainingNodes);
				//free(safeNearestNeighbors);

			}

		}
		else
		{
			goalCheck = false;
		}
		count++;

	} while (count <= maxCount);  //(!goalCheck && count <= maxCount);

	double tempCost = 0, finalCost = 100000;
	ConfigspaceNode finalNode = testNewNode;
	for (int i = 0; i < G_configspace.numNodes; i++)
	{
		if (G_workspace.checkAtGoal(G_configspace.nodes[i]))
		{
			//printf("Goal reached! Cost of %f\n", G_configspace.nodes[i].cost);
			tempCost = G_configspace.nodes[i].cost;
			if (tempCost < finalCost)
			{
				//printf("This is the final goal now!\n");
				finalCost = tempCost;
				finalNode = G_configspace.nodes[i];
			}
		}
	}

#pragma endregion Primary running RRT* code

#pragma region PrintInfo

	if (count <= maxCount)
	{
		/*tempNode = G_configspace.generateBiasedNode(G_workspace.goalRegion.x, G_workspace.goalRegion.y);
		parentNode = G_configspace.findNodeId(nodeIdHold);
		newNode = G_workspace.extendToNode(parentNode, tempNode, delta, epsilon[problemNum - 1]);

		if (newNode.id != parentNode.id && G_workspace.checkAtGoal(newNode))
		{
			ConfigspaceNode* newNodePtr = G_configspace.addNode(newNode);
			G_configspace.addEdge(parentNode, *newNodePtr);
			cost = newNodePtr->cost;
		}*/

		printf("Goal reached!\n");
		printf("Total number of points: %d\n", G_configspace.numNodes);
		printf("Total path cost: %f\n", cost);
		G_configspace.printData(problemNum, finalNode);
	}
	else
	{
		//printf("Too many nodes!\nNo path possible!\n")
		printf("Total number of points: %d\n", G_configspace.numNodes);
		printf("Total path cost: %f\n", finalCost);
		G_configspace.printData(problemNum, finalNode);
	}

#pragma endregion Prints the necessary data and reports success or failure

}
