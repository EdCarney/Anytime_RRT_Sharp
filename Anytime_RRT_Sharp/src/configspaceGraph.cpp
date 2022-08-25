#include "configspaceGraph.hpp"

ConfigspaceNode::ConfigspaceNode()
{
	buildGraphNode();
	cost = 0;
	iterationPoints = NULL;
	numIterationPoints = 0;
}

ConfigspaceNode::ConfigspaceNode(double xVal, double yVal, int idVal, int parentIdVal, double costVal)
{
	buildGraphNode(xVal, yVal, idVal, parentIdVal);
	cost = costVal;
}

void ConfigspaceGraph::buildGraph()
{
	printf("Constructing a default empty configspace graph.\n");
	numNodes = 0;
	numNodeInd = 0;
	numEdges = 0;
	minX = 0;
	minY = 0;
	minTheta = 0;
	maxX = 0;
	maxY = 0;
	maxTheta = 0;
	freeSpaceMeasure = 0;
	zeta = 0;
	dim = 0;
	nodes = NULL;
	edges = NULL;
}

void ConfigspaceGraph::deleteGraph()
{
	printf("Deleting a configspace graph.\n");

	nodes = NULL;
	edges = NULL;

	numEdges = 0;
	numNodes = 0;
}

void ConfigspaceGraph::defineFreespace(double newMinX, double newMinY, double newMinTheta, double newMaxX,
		double newMaxY, double newMaxTheta, int dimension, double obstacleVol)
{
	// set graph parameters
	minX = newMinX;
	minY = newMinY;
	minTheta = newMinTheta;
	maxX = newMaxX;
	maxY = newMaxY;
	maxTheta = newMaxTheta;

	// compute dependent variables
	dim = dimension;
	zeta = 3.14159;
	freeSpaceMeasure = ((maxX - minX) * (maxY - minY)) - obstacleVol;
	gamma_star = 2 * pow((1.0 + 1.0 / dim) * (freeSpaceMeasure / (zeta * dim)), 1.0 / float(dim));
}

void ConfigspaceGraph::addEdge(ConfigspaceNode parentNode, ConfigspaceNode newNode)
{

	// increase memory of edge array for new entry
	if (numEdges > 0)
	{
		Edge* newEdges = (Edge*)calloc(numEdges + 1, sizeof(Edge));
		std::memcpy(newEdges, edges, numEdges * sizeof(Edge));
		free(edges);
		edges = newEdges;
	}
	else
	{
		free(edges);
		edges = (Edge*)calloc(1, sizeof(Edge));;
	}

	edges[numEdges].startNode = parentNode;
	edges[numEdges].endNode = newNode;
	numEdges++;
}

void ConfigspaceGraph::removeEdge(ConfigspaceNode parentToRemove, ConfigspaceNode childToRemove)
{
	Edge* newEdges = (Edge*)calloc(numEdges - 1, sizeof(Edge));
	int newEdgesCount = 0;

	for (int i = 0; i < numEdges; i++)
	{
		if (edges[i].startNode.GetId() != parentToRemove.GetId() || edges[i].endNode.GetId() != childToRemove.GetId())
		{
			newEdges[newEdgesCount] = edges[i];
			newEdgesCount++;
		}
	}
	free(edges);
	edges = newEdges;
	numEdges--;
}

ConfigspaceNode* ConfigspaceGraph::removeNode(ConfigspaceNode *nodeArray, ConfigspaceNode nodeToRemove)
{
	// define variables and get number of
	// new and old nodes
	ConfigspaceNode* newNodeArray;
	int numOldNodes = 0, numNewNodes = 0, newNodeArrayInd = 0;
	while (nodeArray[numOldNodes].GetId()) { numOldNodes++; }
	numNewNodes = numOldNodes - 1;

	// use sizes to set the size of the new array (need extra
	// item to know when we've hit the end of the array
	newNodeArray = (ConfigspaceNode*)calloc(numOldNodes, sizeof(ConfigspaceNode));

	for (int i = 0; i < numOldNodes; i++)
	{
		if (nodeArray[i].GetId() != nodeToRemove.GetId())
		{
			newNodeArray[newNodeArrayInd] = nodeArray[i];
			newNodeArrayInd++;
		}
	}

	newNodeArray[numNewNodes].SetId(0);
	return newNodeArray;
}

void ConfigspaceGraph::removeGraphNodes(ConfigspaceNode *nodesToRemove)
{
	ConfigspaceNode *newNodes;
	Edge *newEdges, *tempNewEdges;
	int numRemoveNodes = 0, newNodeArrayInd = 0, newEdgeArrayInd = 0;
	bool keepFlag = true;

	// get number of nodes to be removed
	while (nodesToRemove[numRemoveNodes].GetId()) { numRemoveNodes++; }

	// use sizes to set the size of the new node array
	newNodes = (ConfigspaceNode*)calloc(numNodes - numRemoveNodes, sizeof(ConfigspaceNode));
	newEdges = (Edge*)calloc(numEdges, sizeof(Edge));

	for (int i = 0; i < numNodes; i++)
	{
		keepFlag = true;

		// check if the current node is one of the nodes to be deleted,
		// if it is, toggle the keepFlag flag
		for (int j = 0; j < numRemoveNodes; j++)
			if (nodes[i].GetId() == nodesToRemove[j].GetId())
				keepFlag = false;

		// if the keepFlag flag is still good, then add the node to the
		// new array of nodes
		if (keepFlag)
		{
			newNodes[newNodeArrayInd] = nodes[i];
			newNodeArrayInd++;
		}
	}
	
	for (int i = 0; i < numEdges; i++)
		{
			keepFlag = true;

			// check if the current edge has a start node that is one of the
			// nodes to be removed, if it is, toggle the flag
			for (int j = 0; j < numRemoveNodes; j++)
				if (edges[i].endNode.GetId() == nodesToRemove[j].GetId()) { keepFlag = false; }

			// if the keepFlag flag is still good, then add the node to the
			// new array of nodes
			if (keepFlag)
			{
				tempNewEdges = (Edge*)calloc(newEdgeArrayInd + 1, sizeof(Edge));
				std::memcpy(tempNewEdges, newEdges, newEdgeArrayInd * sizeof(Edge));
				free(newEdges);
				newEdges = tempNewEdges;
				newEdges[newEdgeArrayInd] = edges[i];
				newEdgeArrayInd++;
			}
		}
	
	// now set the nodes array equal to the new node array
	free(nodes);
	free(edges);
	nodes = newNodes;
	edges = newEdges;
	numNodes = newNodeArrayInd;
	numEdges = numNodes - 1;
}

void ConfigspaceGraph::createNode(double x, double y, double theta, double t)
{
	// increase memory of node array for new entry
	if (numNodes > 0)
	{
		ConfigspaceNode* newNodes = (ConfigspaceNode*)calloc(numNodes + 1, sizeof(ConfigspaceNode));
		std::memcpy(newNodes, nodes, numNodes * sizeof(ConfigspaceNode));
		free(nodes);
		nodes = newNodes;
	}
	else
	{
		free(nodes);
		nodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	}

	nodes[numNodes] = ConfigspaceNode(x, y, numNodes + 1, 0, 0);
	nodes[numNodes].theta = theta;
	nodes[numNodes].iterationPoints = NULL;
	nodes[numNodes].numIterationPoints = 0;
	nodes[numNodes].cost = 0.0;
	++numNodes;
}

ConfigspaceNode ConfigspaceGraph::findNodeId(int nodeId)
{
	for (int i = 0; i < numNodes; i++)
		if (nodes[i].GetId() == nodeId)
			return nodes[i];
	return ConfigspaceNode();
}

int ConfigspaceGraph::findNodePlacement(int nodeId)
{
	for (int i = 0; i < numNodes; i++)
		if (nodes[i].GetId() == nodeId)
			return i;
	return -1;
}

ConfigspaceNode ConfigspaceGraph::generateRandomNode()
{
	double randX, randY;

	randX = minX + (minX + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxX - minX)));
	randY = minY + (minY + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxY - minY)));

	ConfigspaceNode randNode(randX, randY, 0, 0, 0);
	randNode.theta = 0.0;
	randNode.cost = 0.0;

	return randNode;
}

ConfigspaceNode ConfigspaceGraph::generateBiasedNode(double biasedX, double biasedY)
{
	ConfigspaceNode biasedNode(biasedX, biasedY, 0, 0, 0);
	biasedNode.theta = 0.0;
	biasedNode.cost = 0.0;

	return biasedNode;
}

double ConfigspaceGraph::computeRadius(double epsilon)
{
	double percDist = 0.0, circleRadius = 0.0;

	// calculate distance based on percollation theory

	double temp1, temp2, temp3;
	temp1 = log(numNodes) / numNodes;
	temp2 = 1 / float(dim);
	temp3 = pow(temp1, temp2);

	percDist = gamma_star * temp3;

	// set actual extend dist based on the min value
	// of the percDist and the epsilon dist
	circleRadius = percDist < epsilon ? percDist : epsilon;

	return circleRadius;
}

void ConfigspaceGraph::trimTreeChildren(ConfigspaceNode *removeNodes, int saveNodeId)
{
	int removeNodesCount = 0;
	while (removeNodes[removeNodesCount].GetId())
		removeNodesCount++;

	ConfigspaceNode* nodesToRemove = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	nodesToRemove[0].SetId(0);
	int nodeCount = 0;

	// get all nodes that have the removeNodes as a parent node
	for (int i = 0; i < removeNodesCount; i++)
	{
		for (int j = 0; j < numNodes; j++)
		{
			if (removeNodes[i].GetId() == nodes[j].GetParentId())
			{
				ConfigspaceNode *tempNodesToRemove = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
				std::memcpy(tempNodesToRemove, nodesToRemove, (nodeCount) * sizeof(ConfigspaceNode));
				free(nodesToRemove);
				nodesToRemove = tempNodesToRemove;
				nodesToRemove[nodeCount] = nodes[j];
				nodeCount++;
			}
		}
	}

	// add a node to the end of the nodesToRemove with a null id to identify the
	// end of the array
	ConfigspaceNode *tempNodesToRemove = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
	std::memcpy(tempNodesToRemove, nodesToRemove, (nodeCount) * sizeof(ConfigspaceNode));
	free(nodesToRemove);
	nodesToRemove = tempNodesToRemove;
	nodesToRemove[nodeCount].SetId(0);

	// if there are any nodes that have any of the remove nodes as a parent node,
	// then continue until there are no more children
	if (nodeCount > 0)
		trimTreeChildren(nodesToRemove, 0);

	// after all children are obtained, start deleting them in the order of the
	// youngest children first (but save the node if it is the initital node)
	if (removeNodes[0].GetId() != saveNodeId)
		removeGraphNodes(removeNodes);

	free(nodesToRemove);
	free(tempNodesToRemove);
}

void ConfigspaceGraph::printData(int probNum, ConfigspaceNode finalNode)
{
	std::ofstream nodeFile, edgeFile, searchTreeFile, outputPathFile, highFidelityPath;

	// initialize all output files
	nodeFile.open("nodes_" + std::to_string(probNum) + ".txt");
	edgeFile.open("edges_" + std::to_string(probNum) + ".txt");
	searchTreeFile.open("search_tree_" + std::to_string(probNum) + ".txt");
	outputPathFile.open("output_path_" + std::to_string(probNum) + ".txt");

	// print out node file
	nodeFile << numNodes << "\n";

	for (int i = 0; i < numNodes - 1; ++i)
		nodeFile << nodes[i].GetX() << ", " << nodes[i].GetY() << ", " << nodes[i].theta << ", " << nodes[i].GetId() << "\n";

	nodeFile << nodes[numNodes - 1].GetX() << ", " << nodes[numNodes - 1].GetY() << ", "
		<< nodes[numNodes - 1].theta << ", " << nodes[numNodes - 1].GetId() << "\n";

	// print out edge file
	edgeFile << numEdges << "\n";

	for (int i = 0; i < numEdges; ++i)
		edgeFile << edges[i].startNode.GetId() << ", " << edges[i].endNode.GetId() << "\n";

	// print out search tree file
	for (int i = 0; i < numEdges; ++i)
	{
		searchTreeFile << edges[i].startNode.GetId() << ", " << edges[i].startNode.GetX() << ", " << edges[i].startNode.GetY()
			<< ", " << edges[i].endNode.GetId() << ", " << edges[i].endNode.GetX() << ", " << edges[i].endNode.GetY() << "\n";
	}

	// print out output path
	ConfigspaceNode currentNode = finalNode;

	outputPathFile << currentNode.GetX() << ", " << currentNode.GetY() << ", " << currentNode.theta << "\n";
	currentNode = findNodeId(currentNode.GetParentId());

	while (currentNode.GetParentId())
	{
		outputPathFile << currentNode.GetX() << ", " << currentNode.GetY() << ", " << currentNode.theta << "\n";
		currentNode = findNodeId(currentNode.GetParentId());
	}
	outputPathFile << nodes[0].GetX() << ", " << nodes[0].GetY() << ", " << nodes[0].theta << "\n";

	printf("Printing nodes to nodes_%d.txt.\n", probNum);
	printf("Printing edges to edges_%d.txt.\n", probNum);
	printf("Printing search tree to search_tree_%d.txt.\n", probNum);
	printf("Printing output path to output_path_%d.txt.\n", probNum);

	// close files
	nodeFile.close();
	edgeFile.close();
	searchTreeFile.close();
	outputPathFile.close();
}

ConfigspaceNode ConfigspaceGraph::findClosestNode(ConfigspaceNode node)
{
	// initialize distance with first node
	// use euclidean distance of given node from existing nodes
	double shortestDist, dist;
	int closestEntry = 0;
	shortestDist = hypot(nodes[0].GetX() - node.GetX(), nodes[0].GetY() - node.GetY());

	for (int i = 1; i < numNodes; i++)
	{
		dist = hypot(nodes[i].GetX() - node.GetX(), nodes[i].GetY() - node.GetY());;
		if (dist < shortestDist)
		{
			shortestDist = dist;
			closestEntry = i;
		}
	}

	return nodes[closestEntry];
}

double ConfigspaceGraph::computeCost(ConfigspaceNode node_1, ConfigspaceNode node_2)
{
	return hypot((node_1.GetX() - node_2.GetX()), (node_1.GetY() - node_2.GetY()));
}

ConfigspaceNode * ConfigspaceGraph::findNeighbors(ConfigspaceNode centerNode, double radius, int k)
{
	int n = 0;					// variable to count number of neighbors found
	double dist;				// distance between the nodes
	ConfigspaceNode* neighbors;	// pointer to the configuration node array

	neighbors = (ConfigspaceNode*)calloc(k + 1, sizeof(ConfigspaceNode));

	// iterate through all graph nodes and add nodes to the
	// neighbors set if they are within the radius
	for (int i = 0; i < numNodes; ++i)
	{
		dist = hypot((centerNode.GetX() - nodes[i].GetX()), (centerNode.GetY() - nodes[i].GetY()));
		if (dist < radius && centerNode.GetParentId() != nodes[i].GetId())
		{
			neighbors[n++] = nodes[i];
			if (n >= k)
				break;
		}
	}

	// if there were not at least k neighbors found,
	// remove the empty neighbor entries
	if (n < k)
	{
		ConfigspaceNode* tempNeighbors;
		tempNeighbors = (ConfigspaceNode*)calloc(n + 1, sizeof(ConfigspaceNode));
		std::memcpy(tempNeighbors, neighbors, n * sizeof(ConfigspaceNode));
		free(neighbors);
		tempNeighbors[n].SetId(0);
		return tempNeighbors;
	}
	else
	{
		neighbors[k].SetId(0);
		return neighbors;
	}
}

ConfigspaceNode ConfigspaceGraph::findBestNeighbor(ConfigspaceNode newNode, ConfigspaceNode *safeNeighbors)
{
	ConfigspaceNode bestNeighbor;
	int numSafeNeighbors = 1;
	double bestCost = 0, tempBestCost = 0;

	// initialize the best neighbor as the first safe neighbor
	bestNeighbor = safeNeighbors[0];
	bestCost = bestNeighbor.cost + computeCost(newNode, bestNeighbor);

	while (safeNeighbors[numSafeNeighbors].GetId())
	{
		tempBestCost = safeNeighbors[numSafeNeighbors].cost + computeCost(newNode, safeNeighbors[numSafeNeighbors]);
		if (tempBestCost < bestCost)
		{
			bestCost = tempBestCost;
			bestNeighbor = safeNeighbors[numSafeNeighbors];
		}
		++numSafeNeighbors;
	}

	return bestNeighbor;
}

ConfigspaceNode ConfigspaceGraph::addNode(ConfigspaceNode addedNode)
{
	// increase memory of node array for new entry
	if (numNodes > 0)
	{
		ConfigspaceNode* newNodes = (ConfigspaceNode*)calloc(numNodes + 1, sizeof(ConfigspaceNode));
		std::memcpy(newNodes, nodes, numNodes * sizeof(ConfigspaceNode));
		free(nodes);
		nodes = newNodes;
	}
	else
	{
		free(nodes);
		nodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	}

	ConfigspaceNode parentNode = findNodeId(addedNode.GetParentId());

	nodes[numNodes] = ConfigspaceNode(addedNode.GetX(), addedNode.GetY(), ++numNodeInd, addedNode.GetParentId(), parentNode.cost + computeCost(addedNode, parentNode));
	nodes[numNodes].theta = addedNode.theta;
	nodes[numNodes].iterationPoints = addedNode.iterationPoints;
	nodes[numNodes].numIterationPoints = addedNode.numIterationPoints;

	return nodes[numNodes++];
}

void ConfigspaceGraph::propagateCost(ConfigspaceNode * updatedNodes)
{
	int updateNodesCount = 0;
	ConfigspaceNode* tempNodesToUpdate = NULL;

	// get total number of nodes
	while (updatedNodes[updateNodesCount].GetId())
		++updateNodesCount;

	// allocate space for nodes to update cost for
	ConfigspaceNode* nodesToUpdate = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));

	nodesToUpdate[0].SetId(0);
	int nodeCount = 0;

	// go through each updated node and update the cost for any of its
	// child nodes
	for (int i = 0; i < updateNodesCount; ++i)
	{
		for (int j = 0; j < numNodes; ++j)
		{
			if (updatedNodes[i].GetId() == nodes[j].GetParentId())
			{
				tempNodesToUpdate = (ConfigspaceNode*)calloc(nodeCount + 2, sizeof(ConfigspaceNode));
				std::memcpy(tempNodesToUpdate, nodesToUpdate, (nodeCount) * sizeof(ConfigspaceNode));
				free(nodesToUpdate);
				nodesToUpdate = tempNodesToUpdate;
				nodesToUpdate[nodeCount] = nodes[j];
				nodesToUpdate[++nodeCount].SetId(0);
				nodes[j].cost = updatedNodes[i].cost + computeCost(nodes[j], updatedNodes[i]);
			}
		}
	}

	// continue recursion if necessary
	if (nodeCount > 0)
		propagateCost(nodesToUpdate);

	free(tempNodesToUpdate);
}

void ConfigspaceGraph::replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode)
{
	int oldNodePlace = findNodePlacement(oldNode.GetId());
	nodes[oldNodePlace] = newNode;
}
