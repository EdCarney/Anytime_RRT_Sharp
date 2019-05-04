#include <string.h>
#include <math.h>
#include <memory>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "configspaceGraph.h"
#include "workspaceGraph.h"


void ConfigspaceGraph::buildGraph()
{
	printf("Constructing a default empty configspace graph.\n");
	numNodes = 0;
	numNodeInd = 0;
	numEdges = 0;
	minX = 0;
	minY = 0;
	minTheta = 0;
	minV = 0;
	minW = 0;
	maxX = 0;
	maxY = 0;
	maxTheta = 0;
	maxV = 0;
	maxW = 0;
	freeSpaceMeasure = 0;
	zeta = 0;
	dim = 0;
	nodes = NULL;
	edges = NULL;
	maxAbsA = 0;
	maxAbsGamma = 0;
}

void ConfigspaceGraph::deleteGraph()
{
	printf("Deleting a configspace graph.\n");

	nodes = NULL;
	edges = NULL;

	numEdges = 0;
	numNodes = 0;
}

void ConfigspaceGraph::defineFreespace(double newMinX, double newMinY, double newMinTheta, double newMinV, double newMinW,
	double newMaxX, double newMaxY, double newMaxTheta, double newMaxV, double newMaxW, double newMaxAbsA, double newMaxAbsGamma,
	int dimension, double obstacleVol)
{
	// set graph parameters
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
		memcpy(newEdges, edges, numEdges * sizeof(Edge));
		free(edges);
		edges = newEdges;
	}
	else
	{
		edges = (Edge*)calloc(1, sizeof(Edge));;
	}

	edges[numEdges].startNode = parentNode;
	edges[numEdges].endNode = newNode;
	numEdges++;

	//printf("Added edge %d.\n", numEdges);
}

void ConfigspaceGraph::removeEdge(ConfigspaceNode parentToRemove, ConfigspaceNode childToRemove)
{
	Edge* newEdges = (Edge*)calloc(numEdges - 1, sizeof(Edge));
	int newEdgesCount = 0;

	for (int i = 0; i < numEdges; i++)
	{
		if (edges[i].startNode.id != parentToRemove.id || edges[i].endNode.id != childToRemove.id)
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
	while (nodeArray[numOldNodes].id) { numOldNodes++; }
	numNewNodes = numOldNodes - 1;

	// use sizes to set the size of the new array (need extra
	// item to know when we've hit the end of the array
	newNodeArray = (ConfigspaceNode*)calloc(numOldNodes, sizeof(ConfigspaceNode));

	for (int i = 0; i < numOldNodes; i++)
	{
		if (nodeArray[i].id != nodeToRemove.id)
		{
			newNodeArray[newNodeArrayInd] = nodeArray[i];
			newNodeArrayInd++;
		}
	}

	newNodeArray[numNewNodes].id = 0;
	return newNodeArray;
}

void ConfigspaceGraph::removeGraphNodes(ConfigspaceNode *nodesToRemove)
{
	ConfigspaceNode *newNodes;
	Edge *newEdges, *tempNewEdges;
	int numRemoveNodes = 0, newNodeArrayInd = 0, newEdgeArrayInd = 0;
	bool keepFlag = true;

	// get number of nodes to be removed
	while (nodesToRemove[numRemoveNodes].id) { numRemoveNodes++; }

	// use sizes to set the size of the new node array
	newNodes = (ConfigspaceNode*)calloc(numNodes - numRemoveNodes, sizeof(ConfigspaceNode));
	newEdges = (Edge*)calloc(numEdges, sizeof(Edge));

	for (int i = 0; i < numNodes; i++)
	{
		keepFlag = true;

		// check if the current node is one of the nodes to be deleted,
		// if it is, toggle the keepFlag flag
		for (int j = 0; j < numRemoveNodes; j++)
		{
			if (nodes[i].id == nodesToRemove[j].id) { keepFlag = false; }
		}

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
			{
				if (edges[i].endNode.id == nodesToRemove[j].id) { keepFlag = false; }
			}

			// if the keepFlag flag is still good, then add the node to the
			// new array of nodes
			if (keepFlag)
			{
				tempNewEdges = (Edge*)calloc(newEdgeArrayInd + 1, sizeof(Edge));
				memcpy(tempNewEdges, newEdges, newEdgeArrayInd * sizeof(Edge));
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
	numEdges = numNodes - 1;//newEdgeArrayInd;
}

void ConfigspaceGraph::createNode(double x, double y, double theta, double v, double w, double t)
{
	// increase memory of node array for new entry
	if (numNodes > 0)
	{
		ConfigspaceNode* newNodes = (ConfigspaceNode*)calloc(numNodes + 1, sizeof(ConfigspaceNode));
		memcpy(newNodes, nodes, numNodes * sizeof(ConfigspaceNode));
		free(nodes);
		nodes = newNodes;
	}
	else
	{
		free(nodes);
		nodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	}

	nodes[numNodes].x = x;
	nodes[numNodes].y = y;
	nodes[numNodes].theta = theta;
	nodes[numNodes].v = v;
	nodes[numNodes].w = w;
	nodes[numNodes].t = t;
	nodes[numNodes].a = 0.0;
	nodes[numNodes].gamma = 0.0;
	nodes[numNodes].parentNodeId = 0;
	nodes[numNodes].iterationPoints = NULL;
	nodes[numNodes].numIterationPoints = 0;
	nodes[numNodes].id = numNodes + 1;
	nodes[numNodes].cost = 0.0;
	numNodes++;
}

ConfigspaceNode ConfigspaceGraph::addNode(ConfigspaceNode addedNode)
{
	// increase memory of node array for new entry
	if (numNodes > 0)
	{
		ConfigspaceNode* newNodes = (ConfigspaceNode*)calloc(numNodes + 1, sizeof(ConfigspaceNode));
		memcpy(newNodes, nodes, numNodes * sizeof(ConfigspaceNode));
		free(nodes);
		nodes = newNodes;

		newNodes = NULL;
	}
	else
	{
		free(nodes);
		nodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	}

	ConfigspaceNode parentNode = findNodeId(addedNode.parentNodeId);

	nodes[numNodes].x = addedNode.x;
	nodes[numNodes].y = addedNode.y;
	nodes[numNodes].dx = addedNode.dx;
	nodes[numNodes].dy = addedNode.dy;
	nodes[numNodes].ddx = addedNode.ddx;
	nodes[numNodes].ddy = addedNode.ddy;
	nodes[numNodes].theta = addedNode.theta;
	nodes[numNodes].v = addedNode.v;
	nodes[numNodes].w = addedNode.w;
	nodes[numNodes].t = addedNode.t;
	nodes[numNodes].a = addedNode.a;
	nodes[numNodes].gamma = addedNode.gamma;
	nodes[numNodes].parentNodeId = addedNode.parentNodeId;
	nodes[numNodes].iterationPoints = addedNode.iterationPoints;
	nodes[numNodes].numIterationPoints = addedNode.numIterationPoints;
	nodes[numNodes].dist = addedNode.dist;
	nodes[numNodes].cost = numNodes > 0 ? parentNode.cost + computeCost(parentNode, addedNode) : 0.0;
	nodes[numNodes].id = numNodeInd + 1;

	numNodeInd++;
	return nodes[numNodes++]; // IMPORTANT: postfix increment (accesses last node THEN increments)
}

void ConfigspaceGraph::replaceNode(ConfigspaceNode oldNode, ConfigspaceNode newNode)
{
	int oldNodePlace = findNodePlacement(oldNode.id);
	free(nodes[oldNodePlace].iterationPoints);
	nodes[oldNodePlace] = newNode;
	nodes[oldNodePlace].iterationPoints = (ConfigspaceNode*)calloc(newNode.numIterationPoints, sizeof(ConfigspaceNode));
	memcpy(nodes[oldNodePlace].iterationPoints, newNode.iterationPoints, newNode.numIterationPoints * sizeof(ConfigspaceNode));

	//printf("Replace Node Dist: %f\n", nodes[oldNodePlace].dist);
}

ConfigspaceNode ConfigspaceGraph::findClosestNode(ConfigspaceNode node)
{
	// initialize distance with first node
	// use eucledian distance of given node from existing nodes
	double shortestDist, dist;
	int closestEntry = 0;
	shortestDist = 0.5 * (hypot(nodes[0].x - node.x, nodes[0].y - node.y)) +
		abs(nodes[0].theta - node.theta)  + abs(nodes[0].v - node.v);//(abs(atan2((nodes[0].y - node.y), (nodes[0].x - node.x)) - nodes[0].theta));

	for (int i = 1; i < numNodes; i++)
	{
		dist = 0.5 * (hypot(nodes[i].x - node.x, nodes[i].y - node.y)) +
			abs(nodes[i].theta - node.theta) + + abs(nodes[i].v - node.v);//(abs(atan2((nodes[i].y - node.y), (nodes[i].x - node.x)) - nodes[i].theta));
		if (dist < shortestDist)
		{
			shortestDist = dist;
			closestEntry = i;
		}
	}

	return nodes[closestEntry];
}

ConfigspaceNode ConfigspaceGraph::findNodeId(int nodeId)
{
	for (int i = 0; i < numNodes; i++)
	{
		if (nodes[i].id == nodeId)
		{
			return nodes[i];
		}
	}
}

int ConfigspaceGraph::findNodePlacement(int nodeId)
{
	for (int i = 0; i < numNodes; i++)
	{
		if (nodes[i].id == nodeId)
		{
			return i;
		}
	}
}

ConfigspaceNode ConfigspaceGraph::generateRandomNode()
{
	double randX, randY, randTheta, randV, randW;

	randX = minX + (minX + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxX - minX)));
	randY = minY + (minY + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxY - minY)));
	randTheta = minTheta + (minTheta + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxTheta - minTheta)));
	randV = minV + ((minV + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxV - minV))));
	randW = minW * 0.25 + ((minW + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / ((maxW - minW) * 0.25))));

	ConfigspaceNode randNode;
	randNode.x = randX;
	randNode.y = randY;
	randNode.theta = randTheta;
	randNode.v = randV;
	randNode.w = randW;
	randNode.parentNodeId = 0.0;
	randNode.t = 0.0;
	randNode.cost = 0.0;
	randNode.dist = 0.0;
	randNode.id = 0;
	randNode.iterationPoints = NULL;
	randNode.numIterationPoints = 0;

	return randNode;
}

ConfigspaceNode ConfigspaceGraph::generateBiasedNode(double biasedX, double biasedY, double biasedTheta)
{
	double randV, randW;

	randV = minV + ((minV + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / (maxV - minV))));
	randW = minW * 0.25 + ((minW + static_cast <double> (rand())) / (static_cast <double> (RAND_MAX / ((maxW - minW) * 0.25))));

	ConfigspaceNode biasedNode;
	biasedNode.x = biasedX;
	biasedNode.y = biasedY;
	biasedNode.theta = biasedTheta;
	biasedNode.v = 0.0;
	biasedNode.w = 0.0;
	biasedNode.parentNodeId = 0;
	biasedNode.t = 0.0;
	biasedNode.cost = 0.0;
	biasedNode.dist = 0.0;
	biasedNode.id = 0;
	biasedNode.iterationPoints = NULL;
	biasedNode.numIterationPoints = 0;

	return biasedNode;
}

double ConfigspaceGraph::computeCost(ConfigspaceNode node_1, ConfigspaceNode node_2)
{
	double cost = 0.0;

	cost += sqrt(pow(node_1.x - node_2.x, 2) + pow(node_1.y - node_2.y, 2));
	//cost += - abs(node_1.v - node_2.v);
	cost += abs(node_1.theta - node_2.theta);

	return cost;
}

double ConfigspaceGraph::computeRadius(double epsilon)
{
	double percDist = 0.0, circleRadius = 0.0;

	// calculate distance based on percollation theory

	double temp1, temp2, temp3;
	temp1 = log(numNodes * 0.5) / numNodes;
	temp2 = 1 / float(dim);
	temp3 = pow(temp1, temp2);

	percDist = gamma_star * temp3;

	// set actual extend dist based on the min value
	// of the percDist and the epsilon dist
	circleRadius = percDist < epsilon ? percDist : epsilon;

	return circleRadius;
}

ConfigspaceNode* ConfigspaceGraph::findNeighbors(ConfigspaceNode centerNode, double radius, int k)
{
	int n = 0;					// variable to count number of neighbors found
	double dist;				// distance between the nodes
	ConfigspaceNode* neighbors;	// pointer to the configuration node array

	neighbors = (ConfigspaceNode*)calloc(k, sizeof(ConfigspaceNode));

	// iterate through all graph nodes and add nodes to the
	// neighbors set if they are within the radius
	for (int i = 0; i < numNodes; i++)
	{
		dist = hypot((centerNode.x - nodes[i].x), (centerNode.y - nodes[i].y));
		if (dist < radius && centerNode.parentNodeId != nodes[i].id// && (hypot((goalX - nodes[i].x),(goalY - nodes[i].y)) > goalRadius)
			)//&& abs(centerNode.theta - nodes[i].theta) < 3.14159)
		{
			neighbors[n] = nodes[i];
			n++;
			if (n >= k)
			{
				break;
			}
		}
	}

	// if there were not at least k neighbors found,
	// remove the empty neighbor entries
	if (n < k)
	{
		ConfigspaceNode* tempNeighbors;
		tempNeighbors = (ConfigspaceNode*)calloc(n + 1, sizeof(ConfigspaceNode));
		memcpy(tempNeighbors, neighbors, n * sizeof(ConfigspaceNode));
		free(neighbors);
		tempNeighbors[n].id = 0;
		return tempNeighbors;
	}
	else
	{
		neighbors[k].id = 0;
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

	while (safeNeighbors[numSafeNeighbors].id)
	{
		tempBestCost = safeNeighbors[numSafeNeighbors].cost + computeCost(newNode, safeNeighbors[numSafeNeighbors]);
		if (tempBestCost < bestCost)
		{
			bestCost = tempBestCost;
			bestNeighbor = safeNeighbors[numSafeNeighbors];
		}
		numSafeNeighbors++;
	}

	return bestNeighbor;
}

void ConfigspaceGraph::propagateCost(ConfigspaceNode *updatedNodes)
{
	int updateNodesCount = 0;
	while (updatedNodes[updateNodesCount].id) { updateNodesCount++; }

	ConfigspaceNode* nodesToUpdate = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	//nodesToUpdate = ConfigspaceNode[0];
	nodesToUpdate[0].id = 0;
	int nodeCount = 0;

	for (int i = 0; i < updateNodesCount; i++)
	{
		for (int j = 0; j < numNodes; j++)
		{
			if (updatedNodes[i].id == nodes[j].parentNodeId)
			{
				ConfigspaceNode* tempNodesToUpdate = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
				memcpy(tempNodesToUpdate, nodesToUpdate, (nodeCount) * sizeof(ConfigspaceNode));
				free(nodesToUpdate);
				nodesToUpdate = tempNodesToUpdate;
				nodesToUpdate[nodeCount] = nodes[j];
				nodes[j].cost = updatedNodes[i].cost + computeCost(updatedNodes[i], nodes[j]);
				nodeCount++;
			}
		}
	}

	ConfigspaceNode* tempNodesToUpdate = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
	memcpy(tempNodesToUpdate, nodesToUpdate, (nodeCount) * sizeof(ConfigspaceNode));
	free(nodesToUpdate);
	nodesToUpdate = tempNodesToUpdate;
	nodesToUpdate[nodeCount].id = 0;

	if (nodeCount > 0)
	{
		propagateCost(nodesToUpdate);
	}
}

void ConfigspaceGraph::trimTreeChildren(ConfigspaceNode *removeNodes, int saveNodeId)
{
	int removeNodesCount = 0;
	while (removeNodes[removeNodesCount].id) { removeNodesCount++; }

	ConfigspaceNode* nodesToRemove = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	nodesToRemove[0].id = 0;
	int nodeCount = 0;

	// get all nodes that have the removeNodes as a parent node
	for (int i = 0; i < removeNodesCount; i++)
	{
		for (int j = 0; j < numNodes; j++)
		{
			if (removeNodes[i].id == nodes[j].parentNodeId)
			{
				ConfigspaceNode *tempNodesToRemove = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
				memcpy(tempNodesToRemove, nodesToRemove, (nodeCount) * sizeof(ConfigspaceNode));
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
	memcpy(tempNodesToRemove, nodesToRemove, (nodeCount) * sizeof(ConfigspaceNode));
	free(nodesToRemove);
	nodesToRemove = tempNodesToRemove;
	nodesToRemove[nodeCount].id = 0;

	// if there are any nodes that have any of the remove nodes as a parent node,
	// then continue until there are no more children
	if (nodeCount > 0)
	{
		trimTreeChildren(nodesToRemove, 0);
	}

	// after all children are obtained, start deleting them in the order of the
	// youngest children first (but save the node if it is the initital node)
	if (removeNodes[0].id != saveNodeId)
	{
		removeGraphNodes(removeNodes);
	}
}

ConfigspaceNode* ConfigspaceGraph::getLastNodes(ConfigspaceNode finalNode, int m)
{
	ConfigspaceNode *lastNodes = (ConfigspaceNode*)calloc(m, sizeof(ConfigspaceNode));

	lastNodes[m - 1] = finalNode;
	for (int i = m - 2; i >= 0; i--)
	{
		lastNodes[i] = findNodeId(lastNodes[i + 1].parentNodeId);
	}

	return lastNodes;
}

ConfigspaceNode* ConfigspaceGraph::getCostThresholdNodes(ConfigspaceNode finalNode)
{
	ConfigspaceNode parentCheckNode, *nodesToRemove = NULL;
	int nodeCount = 0;

	for (int i = 0; i < numNodes; i++)
	{

		parentCheckNode = findNodeId(nodes[i].parentNodeId);

		if (parentCheckNode.id)
		{
			if ((nodes[i].cost > finalNode.cost && parentCheckNode.cost < finalNode.cost) || nodes[i].id == finalNode.id)
			{
				ConfigspaceNode *tempNodesToRemove = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
				memcpy(tempNodesToRemove, nodesToRemove, nodeCount * sizeof(ConfigspaceNode));
				free(nodesToRemove);
				nodesToRemove = tempNodesToRemove;
				nodesToRemove[nodeCount] = nodes[i];
				nodeCount++;
			}
		}
	}

	// add a node to the end of the nodesToRemove with a null id to identify the
	// end of the array
	ConfigspaceNode *tempNodesToRemove = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
	memcpy(tempNodesToRemove, nodesToRemove, (nodeCount) * sizeof(ConfigspaceNode));
	free(nodesToRemove);
	nodesToRemove = tempNodesToRemove;
	nodesToRemove[nodeCount].id = 0;

	return nodesToRemove;
}

double* ConfigspaceGraph::getBranchBounds(ConfigspaceNode node)
{
	// the branch bounds are equivalent to [xMin, xMax, yMin, yMax]
	double *branchBounds = (double*)calloc(4, sizeof(double));
	branchBounds[0] = node.x; branchBounds[1] = node.x;
	branchBounds[2] = node.y; branchBounds[3] = node.y;

	while (node.parentNodeId)
	{
		node = findNodeId(node.parentNodeId);
		if (node.x < branchBounds[0]) { branchBounds[0] = node.x; }
		if (node.x > branchBounds[1]) { branchBounds[1] = node.x; }
		if (node.y < branchBounds[2]) { branchBounds[2] = node.y; }
		if (node.y > branchBounds[3]) { branchBounds[3] = node.y; }
	}

	return branchBounds;

}

void ConfigspaceGraph::printData(int probNum, ConfigspaceNode finalNode)
{
	std::ofstream nodeFile, edgeFile, searchTreeFile, outputPathFile, highFidelityPath;

	// initialize all output files
	nodeFile.open("nodes_" + std::to_string(probNum) + ".txt");
	edgeFile.open("edges_" + std::to_string(probNum) + ".txt");
	searchTreeFile.open("search_tree_" + std::to_string(probNum) + ".txt");
	outputPathFile.open("output_path_" + std::to_string(probNum) + ".txt");
	highFidelityPath.open("high_fidelity_path_" + std::to_string(probNum) + ".csv");

	// print out node file
	nodeFile << numNodes << "\n";
	for (int i = 0; i < numNodes - 1; i++)
	{
		nodeFile << nodes[i].t << ", " << nodes[i].x << ", " << nodes[i].y << ", " << nodes[i].theta << ", "
			<< nodes[i].v << ", " << nodes[i].w << ", " << nodes[i + 1].a << ", " << nodes[i + 1].gamma << ", " << nodes[i].id << "\n";
	}

	nodeFile << nodes[numNodes - 1].t << ", " << nodes[numNodes - 1].x << ", " << nodes[numNodes - 1].y << ", " << nodes[numNodes - 1].theta << ", "
		<< nodes[numNodes - 1].v << ", " << nodes[numNodes - 1].w << ", " << "0.0" << ", " << "0.0" << ", " << nodes[numNodes - 1].id << "\n";

	// print out edge file
	edgeFile << numEdges << "\n";
	for (int i = 0; i < numEdges; i++)
	{
		edgeFile << edges[i].startNode.id << ", " << edges[i].endNode.id << "\n";
	}

	// print out search tree file
	for (int i = 0; i < numEdges; i++)
	{
		searchTreeFile << edges[i].startNode.id << ", " << edges[i].startNode.x << ", " << edges[i].startNode.y
			<< ", " << edges[i].endNode.id << ", " << edges[i].endNode.x << ", " << edges[i].endNode.y << "\n";
	}

	// print out output path
	ConfigspaceNode currentNode = finalNode;
	double tempLinAccel, tempRotAccel;

	outputPathFile << currentNode.t << ", " << currentNode.x << ", " << currentNode.y << ", " << currentNode.theta << ", "
		<< currentNode.v << ", " << currentNode.w << ", " << "0.0" << ", " << "0.0" << "\n";
	tempLinAccel = currentNode.a;
	tempRotAccel = currentNode.gamma;
	currentNode = findNodeId(currentNode.parentNodeId);

	while (currentNode.parentNodeId)
	{
		outputPathFile << currentNode.t << ", " << currentNode.x << ", " << currentNode.y << ", " << currentNode.theta << ", "
			<< currentNode.v << ", " << currentNode.w << ", " << tempLinAccel << ", " << tempRotAccel << "\n";

		tempLinAccel = currentNode.a;
		tempRotAccel = currentNode.gamma;
		currentNode = findNodeId(currentNode.parentNodeId);
	}
	outputPathFile << nodes[0].t << ", " << nodes[0].x << ", " << nodes[0].y << ", " << nodes[0].theta << ", "
		<< nodes[0].v << ", " << nodes[0].w << ", " << tempLinAccel << ", " << tempRotAccel << "\n";

	// print out high-fidelity path
	currentNode = finalNode;
	double currentTime = 0.0;
	while (currentNode.parentNodeId)
	{
		for (int i = currentNode.numIterationPoints - 1; i >= 0; i--)
		{
			highFidelityPath << currentTime << ", " << currentNode.iterationPoints[i].x << ", " <<
				currentNode.iterationPoints[i].y << ", " << currentNode.iterationPoints[i].theta << ", " <<
				currentNode.iterationPoints[i].dx << ", " << currentNode.iterationPoints[i].dy << ", " <<
				currentNode.iterationPoints[i].ddx << ", " << currentNode.iterationPoints[i].ddy << ", " <<
				currentNode.iterationPoints[i].v << ", " << currentNode.iterationPoints[i].a << "\n";

			currentTime += 0.1;
		}
		currentNode = findNodeId(currentNode.parentNodeId);
	}

	printf("Printing nodes to nodes_%d.txt.\n", probNum);
	printf("Printing edges to edges_%d.txt.\n", probNum);
	printf("Printing search tree to search_tree_%d.txt.\n", probNum);
	printf("Printing output path to output_path_%d.txt.\n", probNum);
	//printf("Printing high-fidelity output path to high_fidelity_path_%d.txt.\n", probNum);

	// close files
	nodeFile.close();
	edgeFile.close();
	searchTreeFile.close();
	outputPathFile.close();
	highFidelityPath.close();
}


/////////////////////////////////////////////////////////////



ConfigspaceNode ConfigspaceGraph::findClosestNode_basic(ConfigspaceNode node)
{
	double shortestDist, dist;
	int closestEntry = 0;
	shortestDist = hypot(nodes[0].x - node.x, nodes[0].y - node.y);

	for (int i = 1; i < numNodes; i++)
	{
		dist = hypot(nodes[i].x - node.x, nodes[i].y - node.y);;
		if (dist < shortestDist)
		{
			shortestDist = dist;
			closestEntry = i;
		}
	}

	return nodes[closestEntry];
}

double ConfigspaceGraph::computeCost_basic(ConfigspaceNode node_1, ConfigspaceNode node_2)
{
	double cost = node_2.dist;
	//double cost = node_2.numIterationPoints;
	//double cost = node_2.dist * node_2.dist / hypot((node_1.x - node_2.x), (node_1.y - node_2.y));
	//double cost = (double) node_2.numIterationPoints / node_2.dist;
	//double cost = node_2.dist - hypot((node_1.x - node_2.x), (node_1.y - node_2.y));
	return cost;
}

ConfigspaceNode * ConfigspaceGraph::findNeighbors_basic(ConfigspaceNode centerNode, double radius, int k)
{
	int n = 0;					// variable to count number of neighbors found
	double dist;				// distance between the nodes
	ConfigspaceNode* neighbors;	// pointer to the configuration node array

	neighbors = (ConfigspaceNode*)calloc(k, sizeof(ConfigspaceNode));

	// iterate through all graph nodes and add nodes to the
	// neighbors set if they are within the radius
	for (int i = 0; i < numNodes; i++)
	{
		dist = hypot((centerNode.x - nodes[i].x), (centerNode.y - nodes[i].y));
		if (dist < radius && centerNode.parentNodeId != nodes[i].id)
		{
			neighbors[n] = nodes[i];
			n++;
			if (n >= k)
			{
				break;
			}
		}
	}

	// if there were not at least k neighbors found,
	// remove the empty neighbor entries
	if (n < k)
	{
		ConfigspaceNode* tempNeighbors;
		tempNeighbors = (ConfigspaceNode*)calloc(n + 1, sizeof(ConfigspaceNode));
		memcpy(tempNeighbors, neighbors, n * sizeof(ConfigspaceNode));
		free(neighbors);
		tempNeighbors[n].id = 0;
		return tempNeighbors;
	}
	else
	{
		neighbors[k].id = 0;
		return neighbors;
	}
}

ConfigspaceNode ConfigspaceGraph::addNode_basic(ConfigspaceNode addedNode)
{
	// increase memory of node array for new entry
	if (numNodes > 0)
	{
		ConfigspaceNode* newNodes = (ConfigspaceNode*)calloc(numNodes + 1, sizeof(ConfigspaceNode));
		memcpy(newNodes, nodes, numNodes * sizeof(ConfigspaceNode));
		free(nodes);
		nodes = newNodes;

		newNodes = NULL;
	}
	else
	{
		free(nodes);
		nodes = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	}

	ConfigspaceNode parentNode = findNodeId(addedNode.parentNodeId);

	nodes[numNodes].x = addedNode.x;
	nodes[numNodes].y = addedNode.y;
	nodes[numNodes].dx = addedNode.dx;
	nodes[numNodes].dy = addedNode.dy;
	nodes[numNodes].ddx = addedNode.ddx;
	nodes[numNodes].ddy = addedNode.ddy;
	nodes[numNodes].theta = addedNode.theta;
	nodes[numNodes].v = addedNode.v;
	nodes[numNodes].w = addedNode.w;
	nodes[numNodes].t = addedNode.t;
	nodes[numNodes].a = addedNode.a;
	nodes[numNodes].gamma = addedNode.gamma;
	nodes[numNodes].parentNodeId = addedNode.parentNodeId;
	nodes[numNodes].iterationPoints = addedNode.iterationPoints;
	nodes[numNodes].numIterationPoints = addedNode.numIterationPoints;
	nodes[numNodes].dist = addedNode.dist;
	nodes[numNodes].cost = parentNode.cost + computeCost_basic(parentNode, addedNode);
	nodes[numNodes].id = numNodeInd + 1;

	numNodeInd++;
	return nodes[numNodes++]; // IMPORTANT: postfix increment (accesses last node THEN increments)
}

void ConfigspaceGraph::propagateCost_basic(ConfigspaceNode * updatedNodes)
{
	int updateNodesCount = 0;
	while (updatedNodes[updateNodesCount].id) { updateNodesCount++; }

	ConfigspaceNode* nodesToUpdate = (ConfigspaceNode*)calloc(1, sizeof(ConfigspaceNode));
	//nodesToUpdate = ConfigspaceNode[0];
	nodesToUpdate[0].id = 0;
	int nodeCount = 0;

	for (int i = 0; i < updateNodesCount; i++)
	{
		for (int j = 0; j < numNodes; j++)
		{
			if (updatedNodes[i].id == nodes[j].parentNodeId)
			{
				ConfigspaceNode* tempNodesToUpdate = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
				memcpy(tempNodesToUpdate, nodesToUpdate, (nodeCount) * sizeof(ConfigspaceNode));
				free(nodesToUpdate);
				nodesToUpdate = tempNodesToUpdate;
				nodesToUpdate[nodeCount] = nodes[j];
				//printf("DEBUG5, j = %d, i = %d, Cost: %f\n", j, i, nodes[j].cost);
				nodes[j].cost = updatedNodes[i].cost + computeCost_basic(updatedNodes[i], nodes[j]);
				nodeCount++;
			}
		}
	}
	ConfigspaceNode* tempNodesToUpdate = (ConfigspaceNode*)calloc(nodeCount + 1, sizeof(ConfigspaceNode));
	memcpy(tempNodesToUpdate, nodesToUpdate, (nodeCount) * sizeof(ConfigspaceNode));
	free(nodesToUpdate);
	nodesToUpdate = tempNodesToUpdate;
	nodesToUpdate[nodeCount].id = 0;
	if (nodeCount > 0)
	{
		propagateCost_basic(nodesToUpdate);
	}
}

void ConfigspaceGraph::replaceNode_basic(ConfigspaceNode oldNode, ConfigspaceNode newNode)
{
	int oldNodePlace = findNodePlacement(oldNode.id);
	nodes[oldNodePlace] = newNode;
}