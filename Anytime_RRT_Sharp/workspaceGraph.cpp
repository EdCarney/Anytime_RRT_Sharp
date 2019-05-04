
#include <string.h>
#include <math.h>
#include <cmath>
#include <memory>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "workspaceGraph.h"
#include "configspaceGraph.h"

void Vehicle::calculateCentroid()
{
	if (nodes)
	{
		centroid.x = 0.0;
		centroid.y = 0.0;
		for (int i = 0; i < numNodes; i++)
		{
			centroid.x += nodes[i].x;
			centroid.y += nodes[i].y;
		}
		centroid.x /= numNodes;
		centroid.y /= numNodes;
	}

	else
	{
		printf("WARN: No vehicle nodes defined, cannot calculate centroid.");
	}
}

void Vehicle::buildVehicle()
{
	printf("Constructing a default vehicle.\n");
	nodes = NULL;
	offsetNodes = NULL;
	numNodes = 0;
	theta = 0.0;
	centroid.id = 0;
	centroid.x = 0.0;
	centroid.y = 0.0;
	centroid.parentNodeId = 0;
	maxPointRadius = 0.0;
}

void Vehicle::updateState(ConfigspaceNode newConfigNode)
{
	// update body nodes based on deltas
	free(nodes);
	nodes = (WorkspaceNode*)calloc(numNodes, sizeof(WorkspaceNode));
	for (int i = 0; i < numNodes; i++)
	{
		nodes[i].x = newConfigNode.x + cos(newConfigNode.theta) * offsetNodes[i].x - sin(newConfigNode.theta) * offsetNodes[i].y;
		nodes[i].y = newConfigNode.y + sin(newConfigNode.theta) * offsetNodes[i].x + cos(newConfigNode.theta) * offsetNodes[i].y;
	}

	// update centroid
	centroid.x = newConfigNode.x;
	centroid.y = newConfigNode.y;
	theta = newConfigNode.theta;
}

void WorkspaceGraph::buildWorkspaceGraph()
{
	printf("Constructing a default empty workspace graph.\n");
	numObstacles = 0;
	numVehicles = 0;
	minX = 0.0;
	minY = 0.0;
	maxX = 0.0;
	maxY = 0.0;
	minTheta = 0.0;
	maxTheta = 0.0;
	minV = 0.0;
	minW = 0.0;
	maxV = 0.0;
	maxW = 0.0;
	maxAbsA = 0.0;
	maxAbsGamma = 0.0;
	obstacles = NULL;
	vehicles = NULL;
	goalRegionReached = false;
}

void WorkspaceGraph::deleteWorkspaceGraph()
{
	printf("Deleting a workspace graph.\n");

	free(obstacles);
	free(vehicles);
	obstacles = NULL;
	vehicles = NULL;
	numObstacles = 0;
	numVehicles = 0;
}

bool WorkspaceGraph::readVehicleFromFile(const char* vehicleFile)
{
	FILE* pFile;

	// open the obstacle file
	pFile = fopen(vehicleFile, "r");

	// check for null pointer
	if (pFile == NULL)
	{
		printf("Unable to open file %s\n", vehicleFile);
		return false;
	}

	// confirm reading in file
	printf("Reading in vehicle data from %s.\n", vehicleFile);

	// increment number of vehicles in graph (assuming one vehicle per file)
	numVehicles++;

	// allocate memory based on vehicle number
	vehicles = (Vehicle*)calloc(numVehicles, sizeof(Vehicle));

	// determine the number of vehicle points in the vehicle file
	int pointCount = 0;
	double x, y;
	while (fscanf(pFile, "%lf,%lf", &x, &y) != EOF) { pointCount++; }

	vehicles[numVehicles - 1].numNodes = pointCount;	// set the graph number of obstacles per the count

	vehicles[numVehicles - 1].nodes =					// allocate memory based on vehicle number
		(WorkspaceNode*)calloc(vehicles[numVehicles - 1].numNodes, sizeof(WorkspaceNode));

	vehicles[numVehicles - 1].offsetNodes =					// allocate memory based on vehicle number
		(WorkspaceNode*)calloc(vehicles[numVehicles - 1].numNodes, sizeof(WorkspaceNode));

	// assign values
	rewind(pFile);
	for (int i = 0; i < pointCount; i++)
	{
		fscanf(pFile, "%lf,%lf", &x, &y);
		vehicles[numVehicles - 1].offsetNodes[i].x = x;
		vehicles[numVehicles - 1].offsetNodes[i].y = y;
		vehicles[numVehicles - 1].offsetNodes[i].id = 0;
		vehicles[numVehicles - 1].offsetNodes[i].parentNodeId = 0;

		vehicles[numVehicles - 1].nodes[i].x = 0.0;
		vehicles[numVehicles - 1].nodes[i].y = 0.0;
		vehicles[numVehicles - 1].nodes[i].id = 0;
		vehicles[numVehicles - 1].nodes[i].parentNodeId = 0;

		vehicles[numVehicles - 1].centroid.x += x;
		vehicles[numVehicles - 1].centroid.y += y;
	}

	// calculate centriod
	vehicles[numVehicles - 1].centroid.x /= pointCount;
	vehicles[numVehicles - 1].centroid.y /= pointCount;

	// set rotation default as zero
	vehicles[numVehicles - 1].theta = 0.0;

	// define radius of circle enscribing the vehicle
	double maxTempRad;
	double maxRad = hypot(
		(vehicles[numVehicles - 1].offsetNodes[0].x - vehicles[numVehicles - 1].centroid.x),
		(vehicles[numVehicles - 1].offsetNodes[0].y - vehicles[numVehicles - 1].centroid.y)
	);

	for (int i = 1; i < vehicles[numVehicles - 1].numNodes; i++)
	{
		maxTempRad = hypot(
			(vehicles[numVehicles - 1].offsetNodes[i].x - vehicles[numVehicles - 1].centroid.x),
			(vehicles[numVehicles - 1].offsetNodes[i].y - vehicles[numVehicles - 1].centroid.y)
		);
		if (maxTempRad > maxRad) { maxRad = maxTempRad; }
	}

	vehicles[numVehicles - 1].maxPointRadius = maxRad;

	// close the file
	fclose(pFile);

	// done reading data
	printf("Completed reading data.\n");
	return true;
}

bool WorkspaceGraph::readObstaclesFromFile(const char* obstacleFile)
{
	FILE* pFile;

	// open the obstacle file
	pFile = fopen(obstacleFile, "r");

	// check for null pointer
	if (pFile == NULL)
	{
		printf("Unable to open file %s\n", obstacleFile);
		return false;
	}

	// confirm reading in file
	printf("Reading in obstacle data from %s.\n", obstacleFile);

	// determine the number of obstacles in the obstacle file
	int obstacleCount = 0;
	double x, y, radius;
	while (fscanf(pFile, "%lf,%lf,%lf", &x, &y, &radius) != EOF)
	{
		obstacleCount++;
	}

	numObstacles = obstacleCount;		// set the graph number of obstacles per the count

	printf("Number of obstacles is %d.\n", numObstacles); // DEBUG STATEMENT TO CHECK # OF OBSTACLES

	// allocate memory based on obstacle number
	obstacles = (Obstacle*)calloc(numObstacles, sizeof(Obstacle));

	// assign values
	rewind(pFile);
	for (int i = 0; i < numObstacles; i++)
	{
		fscanf(pFile, "%lf,%lf,%lf", &x, &y, &radius);
		obstacles[i].x = x;
		obstacles[i].y = y;
		obstacles[i].radius = radius;
	}

	// close the file
	fclose(pFile);

	// done reading data
	printf("Completed reading data.\n");
	return true;
}

double WorkspaceGraph::computeObsVol()
{
	double volume = 0.0;
	for (int i = 0; i < numObstacles; i++)
	{
		volume += 3.14159 * pow(obstacles[i].radius, 2);
	}

	return volume;
}

void WorkspaceGraph::addGoalRegion(double x, double y, double theta, double v, double w, double radius)
{
	goalRegion.x = x;
	goalRegion.y = y;
	goalRegion.theta = theta;
	goalRegion.v = v;
	goalRegion.w = w;
	goalRegion.radius = radius;
}

void WorkspaceGraph::updateGoalRegion(double x, double y, double theta, double v, double w, double radius)
{
	goalRegion.x = x;
	goalRegion.y = y;
	goalRegion.theta = theta;
	goalRegion.v = v;
	goalRegion.w = w;
	goalRegion.radius = radius;
}

void WorkspaceGraph::defineFreespace(double newMinX, double newMinY, double newMinTheta, double newMinV, double newMinW,
	double newMaxX, double newMaxY, double newMaxTheta, double newMaxV, double newMaxW, double newMaxAbsA, double newMaxAbsGamma)
{
	minX = newMinX;
	minY = newMinY;
	minTheta = newMinTheta;
	minV = newMinV;
	minW = newMinW;
	maxX = newMaxX;
	maxY = newMaxY;
	maxTheta = newMaxTheta;
	maxV = newMaxV;
	maxW = newMaxW;
	maxAbsA = newMaxAbsA;
	maxAbsGamma = newMaxAbsGamma;
}

bool WorkspaceGraph::checkCollision(ConfigspaceNode node)
{
	double nodeDistance, centroidDist;

	// iteratate over every vehicle
	for (int i = 0; i < numVehicles; i++)
	{
		// update vehicle state to temp node
		vehicles[i].updateState(node);

		// iterate over every obstacle
		for (int j = 0; j < numObstacles; j++)
		{
			centroidDist = sqrt(
				pow((vehicles[i].centroid.x - obstacles[j].x), 2) +
				pow((vehicles[i].centroid.y - obstacles[j].y), 2)
			);
			//printf("CentDist: %f, ObsRad: %f, VehRad: %f\n", centroidDist, obstacles[j].radius, vehicles[i].maxPointRadius);

			// only iterate over all points if obstacle intersects the ball circumscribing the vehicle
			// or if the same ball extends beyond the freespace
			if (centroidDist < obstacles[j].radius + vehicles[i].maxPointRadius) //||
				//(vehicles[i].centroid.x - vehicles[i].maxPointRadius < minX) ||
				//(vehicles[i].centroid.x + vehicles[i].maxPointRadius > maxX) ||
				//(vehicles[i].centroid.y - vehicles[i].maxPointRadius < minY) ||
				//(vehicles[i].centroid.y + vehicles[i].maxPointRadius > maxY))
			{
				return true;
				/*
				for (int k = 0; k < vehicles[i].numNodes; k++)
				{
					// check if node is outside of freespace
					// if so return previous node, unless initial iteration
					if (vehicles[i].nodes[k].x > maxX || vehicles[i].nodes[k].x < minX || vehicles[i].nodes[k].y > maxY || vehicles[i].nodes[k].y < minY)
					{
						return true;
					}

					// check if node intersects obstacle
					// if so return previous node, unless initial iteration
					nodeDistance = sqrt(pow((vehicles[i].nodes[k].x - obstacles[j].x), 2) + pow((vehicles[i].nodes[k].y - obstacles[j].y), 2));
					if (nodeDistance < obstacles[j].radius)
					{
						return true;
					}
				}
				*/
			}
		}
	}

	return false;
}

ConfigspaceNode WorkspaceGraph::extendToNode(ConfigspaceNode parentNode, ConfigspaceNode newNode, double delta, double epsilon)
{
	ConfigspaceNode currentNode = parentNode;			// initialize a new node to mark current progress
	ConfigspaceNode iterationNode = parentNode;			// initialize a new node to iterate on, starts at parent
	iterationNode.x = parentNode.x;
	iterationNode.y = parentNode.y;
	iterationNode.theta = parentNode.theta;
	iterationNode.v = parentNode.v;
	iterationNode.w = parentNode.w;
	iterationNode.t = parentNode.t;
	iterationNode.parentNodeId = parentNode.id;			// set ID of iteration node to know what is returned from function
	iterationNode.id = 0;
	iterationNode.numIterationPoints = 1;
	iterationNode.iterationPoints = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	iterationNode.iterationPoints[0].x = parentNode.x;
	iterationNode.iterationPoints[0].y = parentNode.y;
	iterationNode.iterationPoints[0].theta = parentNode.theta;
	iterationNode.iterationPoints[0].v = parentNode.v;
	iterationNode.iterationPoints[0].w = parentNode.w;
	iterationNode.iterationPoints[0].t = parentNode.t;

	int numIterations = floor(epsilon / delta);
	double iterationV, iterationW, iterationTimestep, nodeDistance, iterationDelta, iterationLinAccel, iterationRotAccel;
	double checkVal_1, checkVal_2;
	double tol = 0.001;
	double PI = 3.14159265358979323846;

	// determine the acceleration required for the change to take place over 5 seconds
	// we will prioritize speed, and set translationanl velocity to maximum
	// set to max accel values if calculated accel is too high
	double timeStep = 3 / double(numIterations);

	double linAccel = abs(newNode.v - parentNode.v) / double(numIterations) > maxAbsA ? maxAbsA : abs(newNode.v - parentNode.v) / double(numIterations);
	double rotAccel = abs(newNode.w - parentNode.w) / double(numIterations) > maxAbsGamma ? maxAbsGamma : abs(newNode.w - parentNode.w) / double(numIterations);

	// check if new values of translational and rotational velocity require
	// positive or negative acceleration
	iterationLinAccel = (newNode.v < parentNode.v) ? linAccel * -1 : linAccel;
	iterationRotAccel = (newNode.w < parentNode.w) ? rotAccel * -1 : rotAccel;

	// set the control inputs being applied to get to the new node
	iterationNode.a = iterationLinAccel;
	iterationNode.gamma = iterationRotAccel;

	// set the control inputs for the iteration points
	iterationNode.iterationPoints[0].a = iterationLinAccel;
	iterationNode.iterationPoints[0].gamma = iterationRotAccel;

	// set the current node equal to the iteration node
	// the current node will keep track of thelast known safe position
	// the iteration node will be used to iterate forward in time and then checked for collision
	currentNode = iterationNode;

	for (int iteration = 0; iteration < numIterations; iteration++)
	{
		// if checkval is negative and accel is opposite sign of current
		// node velocity, then we will hit zero velocity on this iteration
		// thus update delta size for this iteration
		checkVal_1 = pow(currentNode.v, 2) + 2 * iterationLinAccel * delta;
		if (checkVal_1 < 0 && ((currentNode.v < 0 && iterationLinAccel > 0) || (currentNode.v > 0 && iterationLinAccel < 0)))
		{
			iterationDelta = pow(currentNode.v, 2) / (2 * abs(iterationLinAccel));
		}
		else
		{
			iterationDelta = delta;
		}

		// calculate required change in translational velocity, associated timestep, and
		// corresponding rotational velocity; need to consider cases where translational velocity is negative
		if (iterationLinAccel < 0)
		{
			checkVal_2 = pow(currentNode.v, 2) + 2 * iterationLinAccel * iterationDelta;
			iterationV = checkVal_2 < 0 ? 0.0 : sqrt(checkVal_2);
		}
		else
		{
			iterationV = sqrt(pow(currentNode.v, 2) + 2 * iterationLinAccel * iterationDelta);
		}
		iterationTimestep = abs(iterationV - currentNode.v) / abs(iterationLinAccel);
		iterationW = currentNode.w + iterationRotAccel * iterationTimestep;

		// set iteration velocity to zero if it is within tolerance
		if (abs(iterationV) <= tol) { iterationV = 0; }

		// if limits are exceeded return node as is
		if ((iterationV >= newNode.v && iterationLinAccel > 0) || (iterationV <= newNode.v && iterationLinAccel < 0))
		{
			// free node iterationPoints pointer memory
			//free(iterationNode.iterationPoints);

			if (iteration > 0)
			{
				return currentNode;
			}
			else
			{
				free(currentNode.iterationPoints);
				return parentNode;
			}
		}

		// set iteration node values, we will collision check this prior to
		// making it the current node
		iterationNode.x = currentNode.x + cos(currentNode.theta) * iterationDelta;
		iterationNode.y = currentNode.y + sin(currentNode.theta) * iterationDelta;
		iterationNode.theta = currentNode.theta + currentNode.w * iterationTimestep + 0.5 * rotAccel * iterationTimestep * iterationTimestep;
		iterationNode.v = iterationV;
		iterationNode.w = iterationW;
		iterationNode.t = currentNode.t + iterationTimestep;

		// correct theta value so that it is within valid range
		while (iterationNode.theta > 2 * PI) { iterationNode.theta -= 2 * PI; }
		while (iterationNode.theta < 0) { iterationNode.theta += 2 * PI; }

		// add node iteration info
		// NOTE: do NOT free old iteration points pointer here as the currentNode is still
		// using that memory, only free if currentNode is updated to iterationNode
		ConfigspaceNode* newIterationNodes = (ConfigspaceNode*)calloc(iterationNode.numIterationPoints + 1, sizeof(ConfigspaceNode));
		memcpy(newIterationNodes, iterationNode.iterationPoints, (iterationNode.numIterationPoints) * sizeof(ConfigspaceNode));
		iterationNode.iterationPoints = newIterationNodes;

		iterationNode.iterationPoints[iterationNode.numIterationPoints].x = iterationNode.x;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].y = iterationNode.y;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].theta = iterationNode.theta;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].v = iterationNode.v;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].w = iterationNode.w;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].a = iterationLinAccel;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].gamma = iterationRotAccel;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].t = iterationNode.t;
		iterationNode.numIterationPoints++;

		// check if vehicle collides with any obstacle
		// if so, return current node (last valid node) only if there was
		// at least one successful iteration
		if (checkCollision(iterationNode))
		{
			// free node iterationPoints pointer memory
			//free(iterationNode.iterationPoints);

			if (iteration > 0)
			{
				return currentNode;
			}
			else
			{
				free(currentNode.iterationPoints);
				return parentNode;
			}
		}

		// if extension is valid, update current node and repeat
		// iteration node pointer now points to the same location
		free(currentNode.iterationPoints);
		currentNode = iterationNode;
	}

	// if we make it to the end then we extended a full epsilon!
	// don't free iterationNode pointer values as they are the same
	// for the current node
	return currentNode;
}

ConfigspaceNode WorkspaceGraph::connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode, int numIterations)
{
	ConfigspaceNode currentNode = parentNode;			// initialize a new node to mark current progress
	ConfigspaceNode iterationNode = parentNode;			// initialize a new node to iterate on, starts at parent
	iterationNode.x = parentNode.x;
	iterationNode.y = parentNode.y;
	iterationNode.theta = parentNode.theta;
	iterationNode.v = parentNode.v;
	iterationNode.w = parentNode.w;
	iterationNode.t = parentNode.t;
	iterationNode.parentNodeId = parentNode.id;			// set ID of iteration node to know what is returned from function
	iterationNode.id = 0;
	iterationNode.numIterationPoints = 1;
	iterationNode.iterationPoints = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	iterationNode.iterationPoints[0].x = parentNode.x;
	iterationNode.iterationPoints[0].y = parentNode.y;
	iterationNode.iterationPoints[0].theta = parentNode.theta;
	iterationNode.iterationPoints[0].v = parentNode.v;
	iterationNode.iterationPoints[0].w = parentNode.w;
	iterationNode.iterationPoints[0].t = parentNode.t;

	double delta = hypot((parentNode.x - newNode.x), (parentNode.y - newNode.y)) / double(numIterations);
	double iterationV, iterationW, iterationTimestep, nodeDistance, iterationDelta, iterationLinAccel, iterationRotAccel;
	double delta_1, delta_2, delta_3, delta_4, delta_5;
	double checkVal_1, checkVal_2;
	double tol = 0.001;
	double PI = 3.14159265358979323846;

	bool connectedNodes = false;

	// determine the acceleration required for the change to take place over 5 seconds
	// we will prioritize speed, and set translationanl velocity to maximum
	// set to max accel values if calculated accel is too high
	//double timeStep = 5;
	double timeStep = 3;

	double linAccel = abs(newNode.v - parentNode.v) / double(numIterations) > maxAbsA ? maxAbsA : abs(newNode.v - parentNode.v) / double(numIterations);
	double rotAccel = abs(newNode.w - parentNode.w) / double(numIterations) > maxAbsGamma ? maxAbsGamma : abs(newNode.w - parentNode.w) / double(numIterations);

	// check if new values of translational and rotational velocity require
	// positive or negative acceleration
	iterationLinAccel = (newNode.v < parentNode.v) ? linAccel * -1 : linAccel;
	iterationRotAccel = (newNode.w < parentNode.w) ? rotAccel * -1 : rotAccel;

	// set the control inputs being applied to get to the new node
	iterationNode.a = iterationLinAccel;
	iterationNode.gamma = iterationRotAccel;

	// set the control inputs for the iteration points
	iterationNode.iterationPoints[0].a = iterationLinAccel;
	iterationNode.iterationPoints[0].gamma = iterationRotAccel;

	// set the current node equal to the iteration node
	// the current node will keep track of thelast known safe position
	// the iteration node will be used to iterate forward in time and then checked for collision
	currentNode = iterationNode;

	for (int iteration = 0; iteration < numIterations; iteration++)
	{
		// if checkval is negative and accel is opposite sign of current
		// node velocity, then we will hit zero velocity on this iteration
		// thus update delta size for this iteration
		checkVal_1 = pow(currentNode.v, 2) + 2 * iterationLinAccel * delta;
		if (checkVal_1 < 0 && ((currentNode.v < 0 && iterationLinAccel > 0) || (currentNode.v > 0 && iterationLinAccel < 0)))
		{
			iterationDelta = pow(currentNode.v, 2) / (2 * abs(iterationLinAccel));
		}
		else
		{
			iterationDelta = delta;
		}

		// calculate required change in translational velocity, associated timestep, and
		// corresponding rotational velocity; need to consider cases where translational velocity is negative
		if (iterationLinAccel < 0)
		{
			checkVal_2 = pow(currentNode.v, 2) + 2 * iterationLinAccel * iterationDelta;
			iterationV = checkVal_2 < 0 ? 0.0 : sqrt(checkVal_2);
		}
		else
		{
			iterationV = sqrt(pow(currentNode.v, 2) + 2 * iterationLinAccel * iterationDelta);
		}
		iterationTimestep = abs(iterationV - currentNode.v) / abs(iterationLinAccel);
		iterationW = currentNode.w + iterationRotAccel * iterationTimestep;

		// set iteration velocity to zero if it is within tolerance
		if (abs(iterationV) <= tol) { iterationV = 0; }

		// if limits are exceeded return node as is
		//if ((iterationV >= newNode.v && iterationLinAccel > 0) || (iterationV <= newNode.v && iterationLinAccel < 0))
		//{
		//	// free node iterationPoints pointer memory
		//	//free(iterationNode.iterationPoints);

		//	if (iteration > 0)
		//	{
		//		return currentNode;
		//	}
		//	else
		//	{
		//		free(currentNode.iterationPoints);
		//		return parentNode;
		//	}
		//}

		// set iteration node values, we will collision check this prior to
		// making it the current node
		iterationNode.x = currentNode.x + cos(currentNode.theta) * iterationDelta;
		iterationNode.y = currentNode.y + sin(currentNode.theta) * iterationDelta;
		iterationNode.theta = currentNode.theta + currentNode.w * iterationTimestep + 0.5 * rotAccel * iterationTimestep * iterationTimestep;
		iterationNode.v = iterationV;
		iterationNode.w = iterationW;
		iterationNode.t = currentNode.t + iterationTimestep;

		// correct theta value so that it is within valid range
		while (iterationNode.theta > 2 * PI) { iterationNode.theta -= 2 * PI; }
		while (iterationNode.theta < 0) { iterationNode.theta += 2 * PI; }

		// add node iteration info
		// NOTE: do NOT free old iteration points pointer here as the currentNode is still
		// using that memory, only free if currentNode is updated to iterationNode
		ConfigspaceNode* newIterationNodes = (ConfigspaceNode*)calloc(iterationNode.numIterationPoints + 1, sizeof(ConfigspaceNode));
		memcpy(newIterationNodes, iterationNode.iterationPoints, (iterationNode.numIterationPoints) * sizeof(ConfigspaceNode));
		iterationNode.iterationPoints = newIterationNodes;

		iterationNode.iterationPoints[iterationNode.numIterationPoints].x = iterationNode.x;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].y = iterationNode.y;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].theta = iterationNode.theta;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].v = iterationNode.v;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].w = iterationNode.w;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].a = iterationLinAccel;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].gamma = iterationRotAccel;
		iterationNode.iterationPoints[iterationNode.numIterationPoints].t = iterationNode.t;
		iterationNode.numIterationPoints++;

		// check if vehicle collides with any obstacle
		// if so, return current node (last valid node) only if there was
		// at least one successful iteration
		//if (checkCollision(iterationNode))
		//{
		//	// free node iterationPoints pointer memory
		//	//free(iterationNode.iterationPoints);

		//	if (iteration > 0)
		//	{
		//		return currentNode;
		//	}
		//	else
		//	{
		//		free(currentNode.iterationPoints);
		//		return parentNode;
		//	}
		//}

		// if extension is valid, update current node and repeat
		// iteration node pointer now points to the same location
		free(currentNode.iterationPoints);
		currentNode = iterationNode;

		// compute deltas and check if we are close enough yet
		delta_1 = abs(newNode.x - currentNode.x);
		delta_2 = abs(newNode.y - currentNode.y);
		delta_3 = abs(newNode.theta - currentNode.theta);
		delta_4 = abs(newNode.v - currentNode.v);
		delta_5 = abs(newNode.w - currentNode.w);

		if (delta_1 < 0.25 && delta_2 < 0.25 && delta_3 < 0.1 && delta_4 < 0.1 && delta_5 < 0.1)
		{
			connectedNodes = true;
			break;
		}
	}

	// if the tolerances were met then we were
	// able to connect the nodes and we will return
	// the updated node, otherwise we will return the failure metric
	if (!connectedNodes) { currentNode.parentNodeId = 0; }
	else { currentNode.parentNodeId = parentNode.id; }
	currentNode.id = newNode.id;
	return currentNode;
}

ConfigspaceNode WorkspaceGraph::connectNodesCubicBezier(ConfigspaceNode parentNode, ConfigspaceNode newNode, double timestep, double dt)
{
	WorkspaceNode p0, p1, p2, p3;
	ConfigspaceNode checkNode;
	int numIterations = 0;
	bool connectedNodes = true;
	double tempDist = 0.0;

	// define control points
	// p0 and p3 correspond to the parent and new nodes, respectively
	// p1 and p2 correspond to the control points for the polynomial curves for p1 and p2, respectively
	p0.id = 0; p1.id = 0; p2.id = 0; p3.id = 0;
	p0.parentNodeId = 0; p1.parentNodeId = 0; p2.parentNodeId = 0; p3.parentNodeId = 0;

	p0.x = parentNode.x;
	p0.y = parentNode.y;

	p3.x = newNode.x;
	p3.y = newNode.y;

	p1.x = p0.x + (1.0 / 3.0) * timestep * parentNode.v * cos(parentNode.theta);
	p1.y = p0.y + (1.0 / 3.0) * timestep * parentNode.v * sin(parentNode.theta);
	p2.x = p3.x - (1.0 / 3.0) * timestep * newNode.v * cos(newNode.theta);
	p2.y = p3.y - (1.0 / 3.0) * timestep * newNode.v * sin(newNode.theta);

	// set number of iterations based on the timestep and dt values provided
	numIterations = (int) timestep / dt;

	// create and populate array of dt values for computing path
	double timeArr[numIterations] = { 0 };
	for (int i = 0; i < numIterations; i++)
	{
		timeArr[i] = (double) i * dt;
	}

	// update array for iteration points going to the new node
	newNode.iterationPoints = (ConfigspaceNode*)calloc(numIterations, sizeof(ConfigspaceNode));
	newNode.numIterationPoints = numIterations;
	newNode.iterationPoints[0] = parentNode;
	newNode.dist = 0.0;

	for (int i = 1; i < numIterations; i++)
	{

		newNode.iterationPoints[i].x = pow((1.0 - (timeArr[i] / timestep)), 3) * p0.x + 3.0 * pow((1.0 - (timeArr[i] / timestep)), 2) * (timeArr[i] / timestep) * p1.x +
		3.0 * (1.0 - (timeArr[i] / timestep)) * pow((timeArr[i] / timestep), 2) * p2.x + pow((timeArr[i] / timestep), 3) * p3.x;

		newNode.iterationPoints[i].y = pow((1.0 - (timeArr[i] / timestep)), 3) * p0.y + 3.0 * pow((1.0 - (timeArr[i] / timestep)), 2) * (timeArr[i] / timestep) * p1.y +
		3.0 * (1.0 - (timeArr[i] / timestep)) * pow((timeArr[i] / timestep), 2) * p2.y + pow((timeArr[i] / timestep), 3) * p3.y;

		newNode.iterationPoints[i].dx = 3.0 * pow((1.0 - (timeArr[i] / timestep)), 2) * (p1.x - p0.x) + 6.0 * (1.0 - (timeArr[i] / timestep)) * (timeArr[i] / timestep) * (p2.x - p1.x) +
		3.0 * pow((timeArr[i] / timestep), 2) * (p3.x - p2.x);

		newNode.iterationPoints[i].dy = 3.0 * pow((1.0 - (timeArr[i] / timestep)), 2) * (p1.y - p0.y) + 6.0 * (1.0 - (timeArr[i] / timestep)) * (timeArr[i] / timestep) * (p2.y - p1.y) +
		3.0 * pow((timeArr[i] / timestep), 2) * (p3.y - p2.y);

		newNode.iterationPoints[i].ddx = 6.0 * (1.0 - (timeArr[i] / timestep)) * (p2.x - 2.0 * p1.x + p0.x) + 6.0 * (timeArr[i] / timestep) * (p3.x - 2 * p2.x * p1.x);

		newNode.iterationPoints[i].ddy = 6.0 * (1.0 - (timeArr[i] / timestep)) * (p2.y - 2.0 * p1.y + p0.y) + 6.0 * (timeArr[i] / timestep) * (p3.y - 2 * p2.y * p1.y);

		//printf("dx: %f, dy: %f, ddx: %f, ddy: %f\n", newNode.iterationPoints[i].dx, newNode.iterationPoints[i].dy, newNode.iterationPoints[i].ddx, newNode.iterationPoints[i].ddy);

		// check velocity and acceleration constraints; compute total distance to reach node
		tempDist = hypot(newNode.iterationPoints[i].x - newNode.iterationPoints[i - 1].x, newNode.iterationPoints[i].y - newNode.iterationPoints[i - 1].y);
		newNode.dist += tempDist;
		newNode.iterationPoints[i].v = tempDist / dt;
		newNode.iterationPoints[i].a = (newNode.iterationPoints[i].v - newNode.iterationPoints[i - 1].v) / dt;
		newNode.iterationPoints[i].theta = atan2(newNode.iterationPoints[i].y - newNode.iterationPoints[i - 1].y, newNode.iterationPoints[i].x - newNode.iterationPoints[i - 1].x);
		//printf("Vel: %f, Acc: %f\n", newNode.iterationPoints[i].v, newNode.iterationPoints[i].a);
		if (abs(newNode.iterationPoints[i].a) > maxAbsA || newNode.iterationPoints[i].v < minV || newNode.iterationPoints[i].v > maxV)
		/*if (abs(newNode.iterationPoints[i].ddx) > maxAbsA || abs(newNode.iterationPoints[i].ddy) > maxAbsA ||
			abs(newNode.iterationPoints[i].dx) < minV || abs(newNode.iterationPoints[i].dx) > maxV ||
			abs(newNode.iterationPoints[i].dy) < minV || abs(newNode.iterationPoints[i].dy) > maxV)*/
		{
			//printf("Accel %f, Vel: %f\n",newNode.iterationPoints[i].a, newNode.iterationPoints[i].v);
			connectedNodes = false;
			break;
		}

		// check for collision; if there is a collision, break the loop
		checkNode.x = newNode.iterationPoints[i].x;
		checkNode.y = newNode.iterationPoints[i].y;
		if (checkCollision(checkNode))
		{
			//printf("BOO2\n");
			connectedNodes = false;
			break;
		}
	}

	if (connectedNodes)
	{
		//printf("WOO\n");
		newNode.dx = newNode.iterationPoints[numIterations - 1].dx;
		newNode.dy = newNode.iterationPoints[numIterations - 1].dy;
		newNode.ddx = newNode.iterationPoints[numIterations - 1].ddx;
		newNode.ddy = newNode.iterationPoints[numIterations - 1].ddy;
		newNode.parentNodeId = parentNode.id;
	}
	else { newNode.parentNodeId = 0; }
	return newNode;
}

ConfigspaceNode* WorkspaceGraph::checkSafety(ConfigspaceNode newNode, ConfigspaceNode * neighbors)
{
	// determine number of nodes in neighbors and set the
	// size of the safe neighbors array based on this
	int numNeighbors = 0;
	while (neighbors[numNeighbors].id) { numNeighbors++; }
	ConfigspaceNode* safeNeighbors = (ConfigspaceNode*)calloc(numNeighbors + 1, sizeof(ConfigspaceNode));

	// check the distance of each obstacle to the newNode
	// and the radius of each obstacle to determine possibility
	// of collision
	double nodeCircleRad, obsDist, midpointX, midpointY;
	int safeNeighborCount = 0;
	bool safetyCheck = true;

	for (int i = 0; i < numNeighbors; i++)
	{
		nodeCircleRad = hypot((newNode.x - neighbors[i].x), (newNode.y - neighbors[i].y)) / 2;
		midpointX = (newNode.x + neighbors[i].x) / 2;
		midpointY = (newNode.y + neighbors[i].y) / 2;
		safetyCheck = true;

		// check the given neighbor for all obstacles
		// only add it if collision is not a risk across
		// all obstacles
		for (int j = 0; j < numObstacles; j++)
		{
			obsDist = hypot((midpointX - obstacles[j].x), (midpointY - obstacles[j].y));
			if (obsDist < (nodeCircleRad + obstacles[j].radius))
			{
				safetyCheck = false;
				break;
			}
		}
		if (safetyCheck)
		{
			safeNeighbors[safeNeighborCount] = neighbors[i];
			safeNeighborCount++;
		}
	}

	if (safeNeighborCount < numNeighbors)
	{
		ConfigspaceNode* tempSafeNeighbors;
		tempSafeNeighbors = (ConfigspaceNode*)calloc(safeNeighborCount + 1, sizeof(ConfigspaceNode));
		memcpy(tempSafeNeighbors, safeNeighbors, (safeNeighborCount + 1) * sizeof(ConfigspaceNode));
		free(safeNeighbors);
		tempSafeNeighbors[safeNeighborCount].id = 0;
		return tempSafeNeighbors;
	}
	else
	{
		safeNeighbors[numNeighbors].id = 0;
		return safeNeighbors;
	}
}

bool WorkspaceGraph::obstacleInFreespace(double xObs, double yObs, double radiusObs)
{
	if ((xObs - radiusObs < maxX && xObs + radiusObs > minX) &&
		(yObs - radiusObs < maxY && yObs + radiusObs > minY))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool WorkspaceGraph::checkAtGoal(ConfigspaceNode node)
{
	double nodeDistance;

	// update vehicle state to temp node
	vehicles[0].updateState(node);

	for (int i = 0; i < numVehicles; i++)
	{
		for (int j = 0; j < vehicles[i].numNodes; j++)
		{
			// calculate the euclidean distance from the node to the center
			// of the obstacle for each node of each vehicle
			nodeDistance = sqrt(pow((vehicles[i].nodes[j].x - goalRegion.x), 2) + pow((vehicles[i].nodes[j].y - goalRegion.y), 2));

			// if the distance bewteen the node and the center of the goal region
			// is <= the radius of the obstacle, then the node collides with
			// goal region
			if (nodeDistance <= goalRegion.radius && (abs(goalRegion.v - node.v) < 0.5))
			{
				return true;
			}
		}
	}

	return false;
}

void WorkspaceGraph::addObstacle(double xObs, double yObs, double radiusObs)
{
	// increase memory of node array for new entry
	if (numObstacles > 0)
	{
		Obstacle* newObstacles = (Obstacle*)calloc(numObstacles + 1, sizeof(Obstacle));
		memcpy(newObstacles, obstacles, numObstacles * sizeof(Obstacle));
		free(obstacles);
		obstacles = newObstacles;
		newObstacles = NULL;
	}
	else
	{
		obstacles = (Obstacle*)calloc(1, sizeof(Obstacle));
	}

	obstacles[numObstacles].x = xObs;
	obstacles[numObstacles].y = yObs;
	obstacles[numObstacles].radius = radiusObs;
	numObstacles++;
}

void WorkspaceGraph::addVehicle(double vehiclePointXPosition[4], double vehiclePointYPosition[4], int numVehiclePoints)
{
	double x = 0.0, y = 0.0;

	// increment number of vehicles in graph (assuming one vehicle per file)
	numVehicles++;

	// allocate memory based on vehicle number
	vehicles = (Vehicle*)calloc(numVehicles, sizeof(Vehicle));
	vehicles[numVehicles - 1].numNodes = numVehiclePoints;
	vehicles[numVehicles - 1].nodes = (WorkspaceNode*)calloc(vehicles[numVehicles - 1].numNodes, sizeof(WorkspaceNode));
	vehicles[numVehicles - 1].offsetNodes = (WorkspaceNode*)calloc(vehicles[numVehicles - 1].numNodes, sizeof(WorkspaceNode));

	// assign values
	for (int i = 0; i < numVehiclePoints; i++)
	{
		vehicles[numVehicles - 1].offsetNodes[i].x = vehiclePointXPosition[i];
		vehicles[numVehicles - 1].offsetNodes[i].y = vehiclePointYPosition[i];
		vehicles[numVehicles - 1].offsetNodes[i].id = 0;
		vehicles[numVehicles - 1].offsetNodes[i].parentNodeId = 0;

		vehicles[numVehicles - 1].nodes[i].x = 0.0;
		vehicles[numVehicles - 1].nodes[i].y = 0.0;
		vehicles[numVehicles - 1].nodes[i].id = 0;
		vehicles[numVehicles - 1].nodes[i].parentNodeId = 0;

		vehicles[numVehicles - 1].centroid.x += vehiclePointXPosition[i];
		vehicles[numVehicles - 1].centroid.y += vehiclePointYPosition[i];;
	}

	// calculate centriod
	vehicles[numVehicles - 1].centroid.x /= numVehiclePoints;
	vehicles[numVehicles - 1].centroid.y /= numVehiclePoints;

	// set rotation default as zero
	vehicles[numVehicles - 1].theta = 0.0;

	// define radius of circle enscribing the vehicle
	double maxTempRad;
	double maxRad = hypot(
		(vehicles[numVehicles - 1].offsetNodes[0].x - vehicles[numVehicles - 1].centroid.x),
		(vehicles[numVehicles - 1].offsetNodes[0].y - vehicles[numVehicles - 1].centroid.y)
	);

	for (int i = 1; i < vehicles[numVehicles - 1].numNodes; i++)
	{
		maxTempRad = hypot(
			(vehicles[numVehicles - 1].offsetNodes[i].x - vehicles[numVehicles - 1].centroid.x),
			(vehicles[numVehicles - 1].offsetNodes[i].y - vehicles[numVehicles - 1].centroid.y)
		);
		if (maxTempRad > maxRad) { maxRad = maxTempRad; }
	}

	vehicles[numVehicles - 1].maxPointRadius = maxRad;
	printf("Max Vehicle Radius: %f\n", vehicles[numVehicles - 1].maxPointRadius);
}

bool WorkspaceGraph::atGate(ConfigspaceNode node)
{
	double dist = hypot((node.x - goalRegion.x), (node.y - goalRegion.y));
	if (dist > goalRegion.radius) { return false; }
	else { return true; }
}
///////////////////////////////////////////////////////////////


ConfigspaceNode WorkspaceGraph::extendToNode_basic(ConfigspaceNode parentNode, ConfigspaceNode newNode, double epsilon)
{
	ConfigspaceNode currentNode;
	double dist = hypot((parentNode.x - newNode.x), (parentNode.y - newNode.y));

	currentNode.parentNodeId = parentNode.id;
	currentNode.id = 0;

	if (dist >= epsilon)
	{
		currentNode.x = parentNode.x + ((newNode.x - parentNode.x) / dist) * epsilon;
		currentNode.y = parentNode.y + ((newNode.y - parentNode.y) / dist) * epsilon;
	}
	else
	{
		currentNode.x = newNode.x;
		currentNode.y = newNode.y;
	}

	// if node does not collide with an obstacle, then set its parentNodeId to the parent node
	// else set it to zero
	if (!checkForCollision_basic(currentNode)) {currentNode.parentNodeId = parentNode.id; }
	else { currentNode.parentNodeId = 0; }

	return currentNode;
}

ConfigspaceNode WorkspaceGraph::findBestNeighbor_basic(ConfigspaceNode newNode, ConfigspaceNode *safeNeighbors, double timestep, double dt)
{
	ConfigspaceNode bestNeighbor;
	int numSafeNeighbors = 0;
	double bestCost = 0, tempBestCost = 0;

	// initialize the best neighbor as the first safe neighbor
	// bestNeighbor = safeNeighbors[0];
	bestCost = 1000000.0;

	while (safeNeighbors[numSafeNeighbors].id)
	{
		newNode = connectNodesCubicBezier(safeNeighbors[numSafeNeighbors], newNode, timestep, dt);
		tempBestCost = safeNeighbors[numSafeNeighbors].cost + computeCost_basic(safeNeighbors[numSafeNeighbors], newNode);
		if (newNode.parentNodeId && tempBestCost < bestCost)
		{
			bestCost = tempBestCost;
			bestNeighbor = safeNeighbors[numSafeNeighbors];
		}
		numSafeNeighbors++;
	}

	if (bestCost < 1000000.0)
	{
		return bestNeighbor;	
	}
	else
	{
		return safeNeighbors[0];
	}
}

double WorkspaceGraph::computeCost_basic(ConfigspaceNode node_1, ConfigspaceNode node_2)
{
	/*
	double cost = 0.0;
	cost += hypot((node_1.x - node_2.x), (node_1.y - node_2.y));
	return cost;
	*/

	double cost = node_2.dist;
	return cost;
}

bool WorkspaceGraph::checkForCollision_basic(ConfigspaceNode node)
{
	double nodeDistance;

	for (int i = 0; i < numObstacles; i++)
	{
		// calculate the euclidean distance from the node to the center
		// of the obstacle
		nodeDistance = sqrt(pow((node.x - obstacles[i].x), 2) + pow((node.y - obstacles[i].y), 2));

		// if the distance bewteen the node and the center of the obstacle
		// is <= the radius of the obstacle, then the node collides with
		// the ostacle
		if (nodeDistance <= obstacles[i].radius)
		{
			return true;
		}
	}

	return false;
}

ConfigspaceNode WorkspaceGraph::connectNodes_basic(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{
	newNode.parentNodeId = parentNode.id;
	return newNode;
}

bool WorkspaceGraph::checkAtGoal_basic(ConfigspaceNode node)
{
	double nodeDistance;

	// update vehicle state to temp node
	vehicles[0].updateState(node);

	for (int i = 0; i < numVehicles; i++)
	{
		nodeDistance = hypot((vehicles[i].centroid.x - goalRegion.x), (vehicles[i].centroid.y - goalRegion.y));

		if (nodeDistance < (goalRegion.radius + vehicles[i].maxPointRadius) && 
			(node.theta < goalRegion.theta + (M_PI * 15 / 180) && node.theta > goalRegion.theta - (M_PI * 15 / 180)))
		{
			//printf("Angle: %f, MinAngle: %f, MaxAngle: %f\n", node.theta, goalRegion.theta - (M_PI * 15 / 180), goalRegion.theta + (M_PI * 15 / 180));
			return true;
		}
		// for (int j = 0; j < vehicles[i].numNodes; j++)
		// {
		// 	// calculate the euclidean distance from the node to the center
		// 	// of the obstacle for each node of each vehicle
			
		// 	nodeDistance = sqrt(pow((vehicles[i].nodes[j].x - goalRegion.x), 2) + pow((vehicles[i].nodes[j].y - goalRegion.y), 2));

		// 	// if the distance bewteen the node and the center of the goal region
		// 	// is <= the radius of the obstacle, then the node collides with
		// 	// goal region
		// 	if (nodeDistance <= goalRegion.radius)
		// 	{
		// 		return true;
		// 	}
		// }
	}

	return false;
}
