#include "workspaceGraph.hpp"

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
		obstacles[i] = Obstacle(x, y, radius);
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
		volume += 3.14159 * pow(obstacles[i].GetRadius(), 2);

	return volume;
}

void WorkspaceGraph::addGoalRegion(double x, double y, double theta, double radius)
{
	goalRegion = GoalState(x, y, radius, theta);
}

void WorkspaceGraph::updateGoalRegion(double x, double y, double theta, double radius)
{
	goalRegion = GoalState(x, y, radius, theta);
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
	while (neighbors[numNeighbors].GetId()) { numNeighbors++; }
	ConfigspaceNode* safeNeighbors = (ConfigspaceNode*)calloc(numNeighbors + 1, sizeof(ConfigspaceNode));

	// check the distance of each obstacle to the newNode
	// and the radius of each obstacle to determine possibility
	// of collision
	double nodeCircleRad, obsDist, midpointX, midpointY;
	int safeNeighborCount = 0;
	bool safetyCheck = true;

	for (int i = 0; i < numNeighbors; i++)
	{
		nodeCircleRad = hypot((newNode.GetX() - neighbors[i].GetX()), (newNode.GetY() - neighbors[i].GetY())) / 2;
		midpointX = (newNode.GetX() + neighbors[i].GetX()) / 2;
		midpointY = (newNode.GetY() + neighbors[i].GetY()) / 2;
		Obstacle pathObstacle(midpointX, midpointY, nodeCircleRad);
		safetyCheck = true;

		// check the given neighbor for all obstacles
		// only add it if collision is not a risk across
		// all obstacles
		for (int j = 0; j < numObstacles; j++)
		{
			if (obstacles[j].Intersects(pathObstacle))
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
	safeNeighbors[numNeighbors].SetId(0);
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

bool WorkspaceGraph::atGate(ConfigspaceNode node)
{
	double dist = hypot((node.GetX() - goalRegion.GetX()), (node.GetY() - goalRegion.GetY()));
	return dist <= goalRegion.GetRadius();
}

ConfigspaceNode WorkspaceGraph::extendToNode(ConfigspaceNode parentNode, ConfigspaceNode newNode, double epsilon)
{
	ConfigspaceNode currentNode;
	double dist = hypot((parentNode.GetX() - newNode.GetX()), (parentNode.GetY() - newNode.GetY()));

	if (dist >= epsilon)
	{
		double xVal = parentNode.GetX() + ((newNode.GetX() - parentNode.GetX()) / dist) * epsilon;
		double yVal = parentNode.GetY() + ((newNode.GetY() - parentNode.GetY()) / dist) * epsilon;
		currentNode = ConfigspaceNode(xVal, yVal, 0, parentNode.GetId(), 0);
	}
	else
	{
		currentNode = ConfigspaceNode(newNode.GetX(), newNode.GetY(), 0, parentNode.GetId(), 0);
	}

	if (checkForCollision(currentNode))
		currentNode.SetId(parentNode.GetId());

	return currentNode;
}

bool WorkspaceGraph::checkForCollision(ConfigspaceNode node)
{
	for (int i = 0; i < numObstacles; i++)
		if (obstacles[i].Intersects(node))
			return true;
	return false;
}

ConfigspaceNode WorkspaceGraph::connectNodes(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{
	newNode.SetParentId(parentNode.GetId());
	return newNode;
}

bool WorkspaceGraph::checkAtGoal(ConfigspaceNode node)
{
	// update vehicle state to temp node
	State s(node.GetX(), node.GetY(), node.theta);
	vehicle.updateState(s);

	double distToGoal = hypot((vehicle.state().GetX() - goalRegion.GetX()), (vehicle.state().GetY() - goalRegion.GetY()));
	return distToGoal < (goalRegion.GetRadius() + vehicle.boundingRadius());
}
