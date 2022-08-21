#include "workspaceGraph.hpp"

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

	vehicles[numVehicles - 1].offsetNodes =				// allocate memory based on vehicle number
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

	// calculate centroid
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
		++obstacleCount;

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
		volume += 3.14159 * pow(obstacles[i].radius, 2);

	return volume;
}

void WorkspaceGraph::addGoalRegion(double x, double y, double theta, double radius)
{
	goalRegion.x = x;
	goalRegion.y = y;
	goalRegion.theta = theta;
	goalRegion.radius = radius;
}

void WorkspaceGraph::updateGoalRegion(double x, double y, double theta, double radius)
{
	goalRegion.x = x;
	goalRegion.y = y;
	goalRegion.theta = theta;
	goalRegion.radius = radius;
}

void WorkspaceGraph::defineFreespace(double newMinX, double newMinY, double newMinTheta, double newMaxX,
	double newMaxY, double newMaxTheta)
{
	minX = newMinX;
	minY = newMinY;
	minTheta = newMinTheta;
	maxX = newMaxX;
	maxY = newMaxY;
	maxTheta = newMaxTheta;
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
		return true;
	
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
	// increment number of vehicles in graph (assuming one vehicle per file)
	++numVehicles;

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
	if (dist > goalRegion.radius)
		return false;

	return true;
}

ConfigspaceNode WorkspaceGraph::extendToNode(ConfigspaceNode parentNode, ConfigspaceNode newNode, double epsilon)
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

	if (checkForCollision(currentNode))
		currentNode.id = parentNode.id;

	return currentNode;
}

bool WorkspaceGraph::checkForCollision(ConfigspaceNode node)
{
	double nodeDistance;

	for (int i = 0; i < numObstacles; i++)
	{
		// calculate the euclidean distance from the node to the center
		// of the obstacle
		nodeDistance = sqrt(pow((node.x - obstacles[i].x), 2) + pow((node.y - obstacles[i].y), 2));

		// if the distance between the node and the center of the obstacle
		// is <= the radius of the obstacle, then the node collides with
		// the obstacle
		if (nodeDistance <= obstacles[i].radius)
			return true;
	}

	return false;
}

ConfigspaceNode WorkspaceGraph::connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{
	newNode.parentNodeId = parentNode.id;
	return newNode;
}

bool WorkspaceGraph::checkAtGoal(ConfigspaceNode node)
{
	double nodeDistance;

	// update vehicle state to temp node
	vehicles[0].updateState(node);

	for (int i = 0; i < numVehicles; i++)
	{
		nodeDistance = hypot((vehicles[i].centroid.x - goalRegion.x), (vehicles[i].centroid.y - goalRegion.y));

		if (nodeDistance < (goalRegion.radius + vehicles[i].maxPointRadius)) return true;
	}

	return false;
}
