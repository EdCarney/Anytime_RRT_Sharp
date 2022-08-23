#include "workspaceGraph.hpp"

void Vehicle::buildVehicle()
{
	nodes = NULL;
	offsetNodes = NULL;
	numNodes = 0;
	theta = 0.0;
	centroid.x = 0.0;
	centroid.y = 0.0;
	maxPointRadius = 0.0;
}

void Vehicle::updateState(Position position)
{
	// update body nodes based on deltas
	for (int i = 0; i < numNodes; i++)
	{
		nodes[i].x = position.x + cos(position.theta) * offsetNodes[i].x - sin(position.theta) * offsetNodes[i].y;
		nodes[i].y = position.y + sin(position.theta) * offsetNodes[i].x + cos(position.theta) * offsetNodes[i].y;
	}

	// update centroid
	centroid.x = position.x;
	centroid.y = position.y;
	theta = position.theta;
}

void WorkspaceGraph::buildWorkspaceGraph()
{
	printf("Constructing a default empty workspace graph.\n");
	numObstacles = 0;
	minX = 0.0;
	minY = 0.0;
	maxX = 0.0;
	maxY = 0.0;
	minTheta = 0.0;
	maxTheta = 0.0;
	obstacles = NULL;
	goalRegionReached = false;
}

void WorkspaceGraph::deleteWorkspaceGraph()
{
	printf("Deleting a workspace graph.\n");

	free(obstacles);
	obstacles = NULL;
	numObstacles = 0;
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

	// determine the number of vehicle points in the vehicle file
	int pointCount = 0;
	double x, y;
	while (fscanf(pFile, "%lf,%lf", &x, &y) != EOF) { pointCount++; }

	vehicle.numNodes = pointCount;	// set the graph number of obstacles per the count

	vehicle.nodes =					// allocate memory based on vehicle number
		(GraphNode*)calloc(vehicle.numNodes, sizeof(GraphNode));

	vehicle.offsetNodes =				// allocate memory based on vehicle number
		(GraphNode*)calloc(vehicle.numNodes, sizeof(GraphNode));

	// assign values
	rewind(pFile);
	for (int i = 0; i < pointCount; i++)
	{
		fscanf(pFile, "%lf,%lf", &x, &y);
		vehicle.offsetNodes[i].x = x;
		vehicle.offsetNodes[i].y = y;

		vehicle.nodes[i].x = 0.0;
		vehicle.nodes[i].y = 0.0;

		vehicle.centroid.x += x;
		vehicle.centroid.y += y;
	}

	// calculate centroid
	vehicle.centroid.x /= pointCount;
	vehicle.centroid.y /= pointCount;

	// set rotation default as zero
	vehicle.theta = 0.0;

	// define radius of circle enscribing the vehicle
	double maxTempRad;
	double maxRad = hypot(
		(vehicle.offsetNodes[0].x - vehicle.centroid.x),
		(vehicle.offsetNodes[0].y - vehicle.centroid.y)
	);

	for (int i = 1; i < vehicle.numNodes; i++)
	{
		maxTempRad = hypot(
			(vehicle.offsetNodes[i].x - vehicle.centroid.x),
			(vehicle.offsetNodes[i].y - vehicle.centroid.y)
		);
		if (maxTempRad > maxRad) { maxRad = maxTempRad; }
	}

	vehicle.maxPointRadius = maxRad;

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

	ResetArraySize<ConfigspaceNode>(&safeNeighbors, numNeighbors, safeNeighborCount + 1);
	safeNeighbors[numNeighbors].id = 0;
	return safeNeighbors;
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
	ResetArraySize<Obstacle>(&obstacles, numObstacles, numObstacles + 1);
	obstacles[numObstacles++] = { xObs, yObs, radiusObs };
}

void WorkspaceGraph::setVehicle(double vehiclePointXPosition[4], double vehiclePointYPosition[4], int numVehiclePoints)
{
	vehicle.numNodes = numVehiclePoints;
	vehicle.nodes = (GraphNode*)calloc(vehicle.numNodes, sizeof(GraphNode));
	vehicle.offsetNodes = (GraphNode*)calloc(vehicle.numNodes, sizeof(GraphNode));

	// assign values
	for (int i = 0; i < numVehiclePoints; i++)
	{
		vehicle.offsetNodes[i].x = vehiclePointXPosition[i];
		vehicle.offsetNodes[i].y = vehiclePointYPosition[i];

		vehicle.nodes[i].x = 0.0;
		vehicle.nodes[i].y = 0.0;

		vehicle.centroid.x += vehiclePointXPosition[i];
		vehicle.centroid.y += vehiclePointYPosition[i];;
	}

	// calculate centriod
	vehicle.centroid.x /= numVehiclePoints;
	vehicle.centroid.y /= numVehiclePoints;

	// set rotation default as zero
	vehicle.theta = 0.0;

	// define radius of circle enscribing the vehicle
	double maxTempRad;
	double maxRad = hypot(
		(vehicle.offsetNodes[0].x - vehicle.centroid.x),
		(vehicle.offsetNodes[0].y - vehicle.centroid.y)
	);

	for (int i = 1; i < vehicle.numNodes; i++)
	{
		maxTempRad = hypot(
			(vehicle.offsetNodes[i].x - vehicle.centroid.x),
			(vehicle.offsetNodes[i].y - vehicle.centroid.y)
		);
		if (maxTempRad > maxRad) { maxRad = maxTempRad; }
	}

	vehicle.maxPointRadius = maxRad;
	printf("Max Vehicle Radius: %f\n", vehicle.maxPointRadius);
}

bool WorkspaceGraph::atGate(ConfigspaceNode node)
{
	double dist = hypot((node.x - goalRegion.x), (node.y - goalRegion.y));
	return dist <= goalRegion.radius;
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
	// update vehicle state to temp node
	vehicle.updateState(node);

	double distToGoal = hypot((vehicle.centroid.x - goalRegion.x), (vehicle.centroid.y - goalRegion.y));
	return distToGoal < (goalRegion.radius + vehicle.maxPointRadius);
}
